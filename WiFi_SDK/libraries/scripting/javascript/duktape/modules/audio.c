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

#include "platform_config.h"
#include "wiced.h"
#include "wiced_duktape.h"
#include "wiced_audio.h"
#include "audio_client.h"
#include "audio_client_private.h"
#include "audio.h"
#include "callback_loop.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define POST_AUDIO_MESSAGE(id, handle, data) \
  do { \
    wiced_duktape_callback_queue_element_t message; \
    message.module_id = WDCM_AUDIO; \
    message.event_id = (uint16_t) id; \
    message.module_handle = handle; \
    message.event_data1 = NULL; \
    message.event_data2 = data; \
    wiced_duktape_callback_post_event(&message); \
  } while(0)
/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL                           "duk:audio"
#define LOG_DEBUG_ENABLE                    0
#define DEBUG_CURRENTTIME                   0
#define AUDIO_CLIENT_VOLUME_MAX             (100)

#define AUDIO_CLIENT_NUM_HTTP_BUFFERS       (200)
#define AUDIO_CLIENT_NUM_AUDIO_BUFFERS      (80)
#define AUDIO_CLIENT_SIZE_AUDIO_BUFFERS     (2048)
#define AUDIO_CLIENT_AUDIO_PERIOD_SIZE      (0)

#define AUDIO_CLIENT_HTTP_THRESHOLD_HIGH    (AUDIO_CLIENT_NUM_HTTP_BUFFERS - 5)
#define AUDIO_CLIENT_HTTP_THRESHOLD_LOW     (10)
#define AUDIO_CLIENT_HTTP_PREROLL           (50)
#define AUDIO_CLIENT_HTTP_READ_INHIBIT      (0)

#define PROPERTY_AUTOPLAY                   "autoplay"
#define PROPERTY_CURRENTTIME                "currentTime"
#define PROPERTY_DURATION                   "duration"
#define PROPERTY_ENDED                      "ended"
#define PROPERTY_PAUSED                     "paused"
#define PROPERTY_PLAYED                     "played"
#define PROPERTY_PRELOAD                    "preload"
#define PROPERTY_READYSTATE                 "readyState"
#define PROPERTY_SEEKABLE                   "seekable"
#define PROPERTY_SRC                        "src"
#define PROPERTY_VOLUME                     "volume"

/*
 * Object events
 */

#define EVENT_ABORT                         "abort"
#define EVENT_CANPLAY                       "canplay"
#define EVENT_CANPLAYTHROUGH                "canplaythrough"
#define EVENT_DURATIONCHANGE                "durationchange"
#define EVENT_EMPTIED                       "emptied"
#define EVENT_ENDED                         "ended"
#define EVENT_ERROR                         "error"
#define EVENT_LOADEDDATA                    "loadeddata"
#define EVENT_LOADEDMETADATA                "loadedmetadata"
#define EVENT_LOADSTART                     "loadstart"
#define EVENT_PAUSE                         "pause"
#define EVENT_PLAY                          "play"
#define EVENT_PLAYING                       "playing"
#define EVENT_PROGRESS                      "progress"
#define EVENT_RATECHANGE                    "ratechange"
#define EVENT_SEEKED                        "seeked"
#define EVENT_SEEKING                       "seeking"
#define EVENT_STALLED                       "stalled"
#define EVENT_SUSPEND                       "suspend"
#define EVENT_TIMEUPDATE                    "timeupdate"
#define EVENT_VOLUMECHANGE                  "volumechange"
#define EVENT_WAITING                       "waiting"

#define AUDIO_PLAYBACK_POLLING_INTERVAL     (1000)

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
static void post_error(dpm_audio_handle_t *handle, int errno);
static void post_event_statechange(void *handle, char *name, int state);
static audio_client_ref dpm_audio_client_init( dpm_audio_handle_t* handle );
static void audio_emit_event(dpm_audio_handle_t *handle, const char* event, uint32_t state);
static duk_ret_t set_audio_current_time(dpm_audio_handle_t *handle, double time);
static duk_ret_t set_audio_volume(dpm_audio_handle_t *handle, double volume);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static audio_client_ref duktape_audio_client;
/******************************************************
 *               Function Definitions
 ******************************************************/
static duk_bool_t audio_is_string(duk_context* ctx, duk_idx_t index) {
  return  duk_is_string(ctx, index) ||
          duk_is_undefined(ctx, index);
}

static void audio_emit_event(dpm_audio_handle_t *handle, const char* event, uint32_t state) {
    /* Do not send event if the audio has been asked for shutting down */
    if (duktape_audio_client == NULL)
    {
        LOGW(">>> audio client has been shutdown!!! <<<");
        return;
    }
    duk_push_string(handle->ctx, event);
    duk_push_int(handle->ctx, state);
    duv_emit_event(handle->ctx, handle->data, LWS_AUDIO_EVENT, 2);
}

 static void post_error(dpm_audio_handle_t *handle, int errno)
{
    handle->error = errno;
//  POST_AUDIO_MESSAGE(LWS_AUDIO_CALLBACK_EVENT_ONERROR, handle, errno);
}

static void post_event_statechange(void *handle, char *name, int state)
{
    wiced_duktape_callback_queue_element_t message;
    message.module_id = WDCM_AUDIO;
    message.event_id = LWS_AUDIO_CALLBACK_EVENT_ONSTATECHANGE;
    message.module_handle = handle;
    message.event_data1 = name;
    message.event_data2 = state;
    wiced_duktape_callback_post_event(&message);
}

static wiced_result_t audio_playback_timed_event( void* arg )
{
    dpm_audio_handle_t *handle = (dpm_audio_handle_t *) arg;
    POST_AUDIO_MESSAGE(LWS_AUDIO_CALLBACK_EVENT_PULLING, handle, 0);
    return WICED_SUCCESS;
}

static wiced_result_t audio_client_callback( audio_client_ref client, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg )
{
    wiced_result_t result = WICED_SUCCESS;
    dpm_audio_handle_t *handle = (dpm_audio_handle_t *) userdata;

    LOGD( "AUDIO CALLBACK handle=%p, client=%p-%p, event=%u",
           handle, handle->client, client, event );

    switch ( event )
    {
        case AUDIO_CLIENT_EVENT_ERROR:
        {
            LOGE("AUDIO EVENT: ERROR = %d", (int)arg );

            handle->properties.readyState = LWS_AUDIO_READYSTATE_NOTHING;
            handle->state = LWS_AUDIO_PLAYBACK_UNKNOWN;
            /* Stop timer event */
            wiced_rtos_deregister_timed_event( &handle->timed_event );
            post_event_statechange(handle, EVENT_ERROR, 1);
            post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
            break;
        }
        case AUDIO_CLIENT_EVENT_CONNECTED:
        {
            LOGD( "AUDIO EVENT: CONNECTED" );

            post_event_statechange(handle, EVENT_CANPLAY, 1);
            break;
        }
        case AUDIO_CLIENT_EVENT_AUDIO_FORMAT:
        {
            wiced_audio_config_t*   config;

            LOGD( "AUDIO EVENT: AUDIO FORMAT" );

            config = (wiced_audio_config_t*)arg;
            if (config == NULL)
            {
                LOGD( "Audio format:" );
                LOGD( "  num channels    : %u", config->channels );
                LOGD( "  bits per sample : %u", config->bits_per_sample );
                LOGD( "  sample rate     : %lu", config->sample_rate );
                LOGD( "  frame size      : %u", config->frame_size );
            }
            post_event_statechange(handle, EVENT_CANPLAYTHROUGH, 1);
            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_STARTED:
        {
            LOGD( "AUDIO EVENT: PLAYBACK STARTED" );

            if ( wiced_rtos_register_timed_event( &handle->timed_event,
                                                  WICED_NETWORKING_WORKER_THREAD,
                                                  audio_playback_timed_event,
                                                  AUDIO_PLAYBACK_POLLING_INTERVAL,
                                                  handle )
                 != WICED_SUCCESS )
            {
                LOGE( "Failed to create audio timed event(%d)", result);
            }

            /* Update playback property */
            handle->properties.ended = 0;
            handle->state = LWS_AUDIO_PLAYBACK_STARTED;

            post_event_statechange(handle, EVENT_PLAY, 1);
            post_event_statechange(handle, EVENT_PLAYING, 1);
            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED:
        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
        {
            if ( event == AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED )
            {
                LOGD( "AUDIO EVENT: PLAYBACK STOPPED" );
                handle->state = LWS_AUDIO_PLAYBACK_STOPPED;
            }
            else if ( event == AUDIO_CLIENT_EVENT_PLAYBACK_EOS )
            {
                LOGD( "AUDIO EVENT: PLAYBACK EOS" );
                handle->state = LWS_AUDIO_PLAYBACK_EOS;
            }
            handle->properties.readyState = LWS_AUDIO_READYSTATE_NOTHING;
            /* Stop the timed event */
            if ( wiced_rtos_deregister_timed_event( &handle->timed_event ) !=
                WICED_SUCCESS )
            {
                LOGW( "Failed to stop play thread" );
            }

            handle->properties.ended = 1;
            handle->properties.duration = 0;
            handle->properties.currentTime = 0;
            post_event_statechange(handle, EVENT_ENDED, 1);
            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED:
        {
            LOGD( "AUDIO EVENT: PLAYBACK PAUSED" );

            /* Stop the timed event */
            if ( wiced_rtos_deregister_timed_event( &handle->timed_event ) !=
                 WICED_SUCCESS )
            {
                LOGW( "Failed to stop play thread" );
            }

            handle->state = LWS_AUDIO_PLAYBACK_PAUSED;
            handle->properties.paused = 1;
            post_event_statechange(handle, EVENT_PAUSE, 1);
            break;
        }
        case AUDIO_CLIENT_EVENT_HTTP_COMPLETE:
        {
            LOGD( "AUDIO EVENT: HTTP COMPLETE" );
            break;
        }
        case AUDIO_CLIENT_EVENT_HTTP_REDIRECT:
        {
            LOGD( "AUDIO EVENT: HTTP REDIRECT" );
            break;
        }
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
        {
            LOGD( "AUDIO EVENT: DATA THRESHOLD HIGH" );
            break;
        }
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
        {
            LOGD( "AUDIO EVENT: DATA THRESHOLD LOW" );
            break;
        }
        case AUDIO_CLIENT_EVENT_AUDIO_CODEC:
        {
            LOGD( "AUDIO EVENT: AUDIO_CODEC" );
            break;
        }
        default:
        {
            LOGW( "Unhandled audio_client event: %d", event );
            break;
        }
    }
    return result;
}

static audio_client_ref dpm_audio_client_init( dpm_audio_handle_t* handle )
{
    audio_client_params_t params;

    /* Set up parameters and initialize audio_client */
    memset( &params, 0, sizeof( audio_client_params_t ));
    params.event_cb                    = audio_client_callback;
    params.userdata                    = handle;
    params.interface                   = WICED_STA_INTERFACE;
    params.data_buffer_num             = AUDIO_CLIENT_NUM_HTTP_BUFFERS;
    params.audio_buffer_num            = AUDIO_CLIENT_NUM_AUDIO_BUFFERS;
    params.audio_buffer_size           = AUDIO_CLIENT_SIZE_AUDIO_BUFFERS;
    params.audio_period_size           = AUDIO_CLIENT_AUDIO_PERIOD_SIZE;
    params.data_buffer_preroll         = AUDIO_CLIENT_HTTP_PREROLL;
    params.no_length_disable_preroll   = WICED_FALSE;
    params.data_threshold_high         = AUDIO_CLIENT_HTTP_THRESHOLD_HIGH;
    params.data_threshold_low          = AUDIO_CLIENT_HTTP_THRESHOLD_LOW;
    params.high_threshold_read_inhibit = AUDIO_CLIENT_HTTP_READ_INHIBIT;
    params.device_id                   = PLATFORM_DEFAULT_AUDIO_OUTPUT;
    params.volume                      = AUDIO_CLIENT_VOLUME_MAX;
    params.enable_playback             = WICED_TRUE;

    return audio_client_init( &params );
}

duk_ret_t dpm_new_audio(duk_context *ctx) {
    dpm_audio_handle_t *handle;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    handle = duk_push_fixed_buffer(ctx, sizeof(*handle));
    memset(handle, 0, sizeof(*handle));
    handle->client = dpm_audio_client_init(handle);

    if (handle->client == NULL) {
        LOGE("Failed to create audio client");
        return 0;
    }
    // save the audio client handle for closing
    duktape_audio_client = handle->client;

    handle->ctx = ctx;
    handle->data = duv_setup_handle(ctx);
    LOGD("new_audios: handle=%p client=%p", handle, handle->client);

    return 1;
}

duk_ret_t dpm_audio_load(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_audio_load handle=%p client=%p", handle, handle->client);

    // sanity check
    if ( handle->client == NULL ) {
        LOGE( "No audio client (%p)", handle->client);
        post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
        return 0;
    }
    // does't support preload - set dataReady to have "enough data"
    handle->properties.readyState = LWS_AUDIO_READYSTATE_ENOUGH_DATA;

    if (handle->state == LWS_AUDIO_PLAYBACK_STARTED ||
        handle->state == LWS_AUDIO_PLAYBACK_PAUSING ||
        handle->state == LWS_AUDIO_PLAYBACK_PAUSED) {
        result = audio_client_ioctl(duktape_audio_client,
                          AUDIO_CLIENT_IOCTL_STOP, NULL);
        LOGD( "LOAD: Stopping current playback(%d) state:%d", result, handle->state );
        if ( result == WICED_SUCCESS )
        {
            LOGD( " >>> Waiting for audio playback to stop... <<<" );
            while (handle->state != LWS_AUDIO_PLAYBACK_STOPPED &&
                handle->state != LWS_AUDIO_PLAYBACK_EOS) {
                wiced_rtos_delay_milliseconds(100);
            }
            LOGD( " >>> Audio playback stopped <<<" );
        } else {
            LOGE( "audio_client failed to stop audio playback (%d)", result );
            post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
        }
    }

    LOGW(">>> Force to play since we don't support preload yet <<<");
    return dpm_audio_play(ctx);
}

duk_ret_t dpm_audio_play(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_audio_play handle=%p client=%p", handle, handle->client);

    // sanity check
    if ( handle->client == NULL || handle->properties.src == NULL) {
        LOGE( "No audio client or audio URI configured(%p-%p)",
              handle->client, handle->properties.src );
        post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
        return 0;
    }

    if (handle->state == LWS_AUDIO_PLAYBACK_UNKNOWN  ||
        handle->state == LWS_AUDIO_PLAYBACK_STOPPING ||
        handle->state == LWS_AUDIO_PLAYBACK_STOPPED  ||
        handle->state == LWS_AUDIO_PLAYBACK_EOS) {
        result = audio_client_ioctl(handle->client, AUDIO_CLIENT_IOCTL_PLAY,
                                    (void*) handle->properties.src);
        LOGD( "audio: PLAY IOCTL src=%s, ret=%d", handle->properties.src, result );
        if ( result == WICED_SUCCESS )
        {
            handle->properties.readyState = LWS_AUDIO_READYSTATE_ENOUGH_DATA;
            handle->state = LWS_AUDIO_PLAYBACK_STARTING;
            post_event_statechange(handle, PROPERTY_PAUSED, 0);
        } else {
            LOGE( "audio_client failed to start audio playback (%d)", result );
            post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
        }
    } else {
       /* We are either playing or paused */
        if ( handle->state == LWS_AUDIO_PLAYBACK_PAUSING ||
             handle->state == LWS_AUDIO_PLAYBACK_PAUSED) {
            result = audio_client_ioctl(handle->client,
                        AUDIO_CLIENT_IOCTL_RESUME, NULL);
            LOGD( "audio: RESUME IOCTL (%d)", result );
            if (result == WICED_SUCCESS ) {
                handle->state = LWS_AUDIO_PLAYBACK_STARTING;
                post_event_statechange(handle, PROPERTY_PAUSED, 0);
            } else {
                LOGE( "audio_client failed to resume audio playback!", result);
                post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
            }
        } else {
            LOGD( "audio_client already playing" );
        }
    }
    return 0;
}

duk_ret_t dpm_audio_pause(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_audio_pause handle=%p client=%p", handle, handle->client);

    if (handle->state == LWS_AUDIO_PLAYBACK_STARTING ||
        handle->state == LWS_AUDIO_PLAYBACK_STARTED) {
        result = audio_client_ioctl(handle->client,
                       AUDIO_CLIENT_IOCTL_PAUSE, NULL);
        LOGD("audio: PAUSE IOCTL (%d)", result );
        if ( result == WICED_SUCCESS )
        {
            handle->state = LWS_AUDIO_PLAYBACK_PAUSING;
            post_event_statechange(handle, PROPERTY_PAUSED, 1);
        } else {
            LOGE( "audio_client failed to start audio playback (%d)", result );
            post_error(handle, LWS_AUDIO_ERROR_INTERNAL);
        }
    } else {
        LOGD("dpm_audio_pause: current state(%d)", handle->state);
    }

    return 0;
}

duk_ret_t dpm_audio_shutdown(duk_context *ctx) {

    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    LOGW("AUDIO: shutting down audio client...");
    dpm_audio_handle_t* handle = (dpm_audio_handle_t *)duktape_audio_client->params.userdata;
    wiced_rtos_deregister_timed_event( &handle->timed_event );
    audio_client_deinit(duktape_audio_client);
    duktape_audio_client = NULL;
    LOGW("       audio client shutdown");
    return 0;
}

static duk_ret_t set_audio_current_time(dpm_audio_handle_t *handle, double time) {
    wiced_result_t result;
    uint32_t time_ms = (uint32_t)( time * 1000.0 );

    result = audio_client_ioctl( handle->client,
                                 AUDIO_CLIENT_IOCTL_SEEK,
                                 (void*)time_ms );
    LOGD("audio: PAUSE IOCTL time(%lu) result=%d", time_ms, result );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to seek to %f s", time );
    }
    return 0;
}

static duk_ret_t set_audio_volume(dpm_audio_handle_t *handle, double volume) {
    wiced_result_t result;

    if (volume < 0.0 || volume > 1.0) {
        LOGW("Invalid voluem value %f", volume);
        volume = volume < 0.0 ? 0.0 : 1.0;
    }
    handle->properties.volume = volume;

    result = audio_client_ioctl( handle->client,
                                 AUDIO_CLIENT_IOCTL_SET_VOLUME,
                                 (void*)(uint32_t)(volume *100.0) );
    LOGD("audio: VOLUME IOCTL volume(%f) result=%d", volume, result );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to seek to %f s", time );
    }
    return 0;
}

duk_ret_t dpm_audio_property_setter(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    const char* name;
    double value;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"name", duk_is_string},
        {"volume", duk_is_number},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    name = duk_get_string(ctx, 1);
    value = duk_get_number(ctx, 2);
    LOGD("dpm_audio_setter: handle=%p (%s - %f)", handle, name, value);

    if (strcmp(name, PROPERTY_CURRENTTIME) == 0) {
        return set_audio_current_time(handle, value);
    } else if (strcmp(name, PROPERTY_VOLUME) == 0) {
        return set_audio_volume(handle, value);
    } else {
        LOGW("Write to invalid property: %s!!!", name);
    }
    return 0;
}

duk_ret_t dpm_audio_property_getter(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    const char *name;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"name", duk_is_string},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    name = duk_get_string(ctx, 1);
    LOGD("dpm_audio_property_getter: %s handle=%p", name, handle);

    if (strcmp(name, PROPERTY_SRC) == 0) {
        duk_push_string(handle->ctx, handle->properties.src);
    } else if (strcmp(name, PROPERTY_PRELOAD) == 0) {
        duk_push_string(handle->ctx, handle->properties.preload);
    } else if (strcmp(name, PROPERTY_AUTOPLAY) == 0) {
        duk_push_string(handle->ctx, handle->properties.autoplay);
    } else if (strcmp(name, PROPERTY_CURRENTTIME) == 0) {
        duk_push_number(handle->ctx, handle->properties.currentTime);
    } else if (strcmp(name, PROPERTY_VOLUME) == 0) {
        duk_push_number(handle->ctx, handle->properties.volume);
    } else if (strcmp(name, PROPERTY_DURATION) == 0) {
        duk_push_number(handle->ctx, handle->properties.duration);
    } else if (strcmp(name, PROPERTY_PAUSED) == 0) {
        duk_push_boolean(handle->ctx, handle->properties.paused);
    } else if (strcmp(name, PROPERTY_ENDED) == 0) {
        duk_push_boolean(handle->ctx, handle->properties.ended);
    } else if (strcmp(name, PROPERTY_READYSTATE) == 0) {
        duk_push_int(handle->ctx, handle->properties.readyState);
    } else {
        LOGW("Failed to retrieve property: %s!!!", name);
        return 0;
    }
    return 1;
}

duk_ret_t dpm_audio_attribute(duk_context *ctx) {
    dpm_audio_handle_t* handle;
    const char *name, *value;
    size_t value_len;
    char *buf, **property;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"name", duk_is_string},
        {"value", audio_is_string},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    name = duk_get_string(ctx, 1);
    LOGD("dpm_audio_attribute %s handle=%p:", name, handle);

    value = duk_get_lstring(ctx, 2, &value_len);
    if (strcmp(name, PROPERTY_SRC) == 0) {
        property = &(handle->properties.src);
    } else if (strcmp(name, PROPERTY_PRELOAD) == 0) {
        property = &(handle->properties.preload);
    } else if (strcmp(name, PROPERTY_AUTOPLAY) == 0) {
        property = &(handle->properties.autoplay);
    } else {
        LOGW("Attribute %s is not support yet!!!");
        return 0;
    }

    if (*property != NULL) {
        LOGD("Free the previous(%p)", name, *property);
        free(*property);
        *property = NULL;
    }

    if (value == NULL || value_len == 0) {
        LOGD("RemoveAttribute %s", name);
        return 0;
    }

    value_len += 1;
    buf = malloc(value_len);
    if (buf == NULL) {
        post_error(handle, LWS_AUDIO_ERROR_NOMEM);
        return 0;
    }

    strncpy(buf, value, value_len);
    *property = buf;
    LOGD("Set %s property to %s", name, buf);
    return 0;
}

duk_ret_t dpm_audio_on_event(duk_context *ctx) {
    dpm_audio_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"onEvent", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_audio_on_event: handle=%p, data=%p", handle, handle->data);
    duv_store_handler(ctx, handle->data, LWS_AUDIO_EVENT, 1);
    return 0;
}

void audio_callback_handler(wiced_duktape_callback_queue_element_t *message)
{
    dpm_audio_callback_event_id event_id = (dpm_audio_callback_event_id) message->event_id;
    dpm_audio_handle_t *handle = (dpm_audio_handle_t *)message->module_handle;
    LOGD("AUDIO: callback_handler: handle=%p, event=%d", handle, event_id);

    switch ( event_id )
    {
        case LWS_AUDIO_CALLBACK_EVENT_PULLING:
        {
            audio_client_track_info_t info;
            wiced_result_t result;
            /* Get the current track info */
            result = audio_client_ioctl(handle->client, AUDIO_CLIENT_IOCTL_TRACK_INFO,
                             &info);
            LOGD("audio: TRACK IOCTL result=%d", result);
            if (result != WICED_SUCCESS)
            {
                LOGW( "Failed to get track info");
            } else {
#if DEBUG_CURRENTTIME
                LOGD( "Track info:" );
                LOGD( " current sample : %u", (uint32_t)info.current_sample );
                LOGD( " total samples  : %u", (uint32_t)info.total_samples );
                LOGD( " sample rate    : %u", info.sample_rate );
                LOGD( " channels       : %hhu", info.channels );
                LOGD( " bps            : %hhu", info.bps );
                LOGD( " bitrate        : %u", info.bitrate );
                LOGD( " content length : %u", handle->client->http_content_length);
#endif
                if (handle->properties.duration == 0)
                {
                    if ( info.total_samples != 0 )
                    {
                        handle->properties.duration =
                            (double)info.sample_rate / (double)info.total_samples ;
                    }
                    else if (( info.current_sample > 0 ) && ( info.total_samples == 0 ) &&
                            ( info.bitrate != 0 ))
                    {
                        handle->properties.duration =
                            (double)handle->client->http_content_length /
                                ((double)info.bitrate / 8.0 );
                    }

                    LOGD( " duration       : %f", handle->properties.duration);
                    if ( handle->properties.duration != 0 )
                    {
                        // push the 'durationchange' event
                        LOGD("AUDIO ONSTATECHANGE: handle=%p duv_handle=%p event(%s)",
                        handle, handle->data, EVENT_DURATIONCHANGE);
                        audio_emit_event(handle, EVENT_DURATIONCHANGE, 1);
                    }
                }

                if (info.current_sample > 0) {
                    handle->properties.currentTime =
                        (double)info.current_sample / (double)info.sample_rate;
#if DEBUG_CURRENTTIME
                    LOGD( " currentTime    : %f", handle->properties.currentTime);
                    LOGD("AUDIO ONSTATECHANGE: handle=%p duv_handle=%p event(%s)",
                    handle, handle->data, EVENT_TIMEUPDATE);
#endif
                    // push the 'timeupdate' event
                    audio_emit_event(handle, EVENT_TIMEUPDATE, 1);
                }
            }
            break;
        }
        case LWS_AUDIO_CALLBACK_EVENT_ONSTATECHANGE:
        {
            const char * event = message->event_data1;
            uint32_t state = message->event_data2;
            LOGD("AUDIO ONSTATECHANGE: handle=%p duv_handle=%p event(%s-%d)",
                 handle, handle->data, event, state);

            // push the event ID and state
            audio_emit_event(handle, event, state);
            break;
        }
        default:
            break;
    }
    return;
}


