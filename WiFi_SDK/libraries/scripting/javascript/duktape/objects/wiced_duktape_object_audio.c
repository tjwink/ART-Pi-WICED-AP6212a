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

#include "wiced.h"
#include "wiced_duktape.h"
#include "platform_config.h"
#include "wiced_audio.h"
#include "audio_client.h"
#include "audio_client_private.h"

/* WICED Duktape Audio object
 * Notes:
 *  - Since WICED platforms can only support a single instance of audio
 *    playback, it only makes sense to have a single intance of audio_client
 *  - The audio_client lives in Duktape global stash; it gets initialized by
 *    the first audio object instance
 *  - An instance can reserve the audio_client by calling
 *    audio_object_client_reserve(); when the instance is done with the
 *    audio_client, it should call audio_object_client_release()
 *  - When the audio_client is reserved, the instanced that reserved it gets
 *    saved into the audio_client structure
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL                           "duk:obj:audio"
#define LOG_DEBUG_ENABLE                    0

#define AUDIO_CLIENT_VOLUME_MAX             (100)

#define AUDIO_CLIENT_NUM_HTTP_BUFFERS       (200)
#define AUDIO_CLIENT_NUM_AUDIO_BUFFERS      (80)
#define AUDIO_CLIENT_SIZE_AUDIO_BUFFERS     (2048)
#define AUDIO_CLIENT_AUDIO_PERIOD_SIZE      (0)

#define AUDIO_CLIENT_HTTP_THRESHOLD_HIGH    (AUDIO_CLIENT_NUM_HTTP_BUFFERS - 5)
#define AUDIO_CLIENT_HTTP_THRESHOLD_LOW     (10)
#define AUDIO_CLIENT_HTTP_PREROLL           (50)
#define AUDIO_CLIENT_HTTP_READ_INHIBIT      (0)

/*
 * Object properties
 */

#define GLOBAL_PROPERTY_AUDIO_CLIENT        "__audio_client"

/* Macro for defining internal properties by using the "\xff" prefix */
#define INTERNAL_PROPERTY(name)             ("\xff" name)

#define INTERNAL_PROPERTY_CALLBACKS         INTERNAL_PROPERTY("callbacks")
#define INTERNAL_PROPERTY_CURRENTTIME       INTERNAL_PROPERTY("currentTime")
#define INTERNAL_PROPERTY_PLAYED            INTERNAL_PROPERTY("played")
#define INTERNAL_PROPERTY_VOLUME            INTERNAL_PROPERTY("volume")

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

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct audio_object_client_s
{
    audio_client_ref    ref;
    void*               user;
    wiced_timed_event_t play_thread;
} audio_object_client_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* List of supported events */
static const char* audio_object_events[] =
{
    EVENT_ABORT,
    EVENT_CANPLAY,
    EVENT_CANPLAYTHROUGH,
    EVENT_DURATIONCHANGE,
    EVENT_EMPTIED,
    EVENT_ENDED,
    EVENT_ERROR,
    EVENT_LOADEDDATA,
    EVENT_LOADEDMETADATA,
    EVENT_LOADSTART,
    EVENT_PAUSE,
    EVENT_PLAY,
    EVENT_PLAYING,
    EVENT_PROGRESS,
    EVENT_RATECHANGE,
    EVENT_SEEKED,
    EVENT_SEEKING,
    EVENT_STALLED,
    EVENT_SUSPEND,
    EVENT_TIMEUPDATE,
    EVENT_VOLUMECHANGE,
    EVENT_WAITING,
    NULL
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static audio_object_client_t* audio_object_client_get( duk_context* ctx )
{
    audio_object_client_t* audio_client;

    wiced_assert( "Bad args", ctx != NULL );

    duk_push_global_stash( ctx ); /* -> [global] */

    if ( duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_AUDIO_CLIENT ) != 1 )
    {
        /* -> [global undefined] */

        duk_pop_2( ctx ); /* -> [] */
        return NULL;
    }
    /* -> [global audio_client] */

    audio_client = duk_to_pointer( ctx, -1 );
    duk_pop_2( ctx ); /* -> [] */

    return audio_client;
}

static wiced_result_t audio_object_client_reserve( duk_context* ctx, audio_object_client_t* audio_client )
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_assert( "Bad args", ( ctx != NULL ) && ( audio_client != NULL ));

    duk_push_this( ctx ); /* -> [this] */

    if ( audio_client->user == NULL )
    {
        LOGD( "Reserving audio_client" );
        audio_client->user = duk_get_heapptr( ctx, -1 );
    }
    else if ( audio_client->user == duk_get_heapptr( ctx, -1 ))
    {
        LOGD( "Already reserved audio_client" );
    }
    else
    {
        LOGD( "Another instance has reserved audio_client" );
        result = WICED_ERROR;
    }

    duk_pop( ctx ); /* -> [] */

    return result;
}

static void audio_object_client_release( duk_context* ctx, audio_object_client_t* audio_client, void *this )
{
    wiced_assert( "Bad args", ( ctx != NULL ) && ( audio_client != NULL ));

    if ( this == NULL )
    {
        duk_push_this( ctx ); /* -> [this] */
    }
    else
    {
        /* Not called from any instance, e.g., from callback function, so
         * push the provided pointer onto stack as 'this'
         */
        duk_push_heapptr( ctx, this ); /* -> [this] */
    }

    if ( audio_client->user == NULL )
    {
        LOGW( "Trying to release an unreserved audio_client" );
    }
    else if ( audio_client->user != duk_get_heapptr( ctx, -1 ))
    {
        LOGW( "Trying to release audio_client reserved by another instance!" );
        wiced_assert( "BUG!", audio_client->user != duk_get_heapptr( ctx, -1 ));
    }
    else
    {
        LOGD( "Releasing audio_client" );
        audio_client->user = NULL;
    }

    duk_pop( ctx ); /* -> [] */
}

static wiced_bool_t audio_object_client_is_reserved( duk_context* ctx, audio_object_client_t* audio_client )
{
    wiced_bool_t reserved = WICED_FALSE;

    wiced_assert( "Bad args", ( ctx != NULL ) && ( audio_client != NULL ));

    duk_push_this( ctx ); /* -> [this] */

    if ( audio_client->user != NULL &&
         audio_client->user == duk_get_heapptr( ctx, -1 ))
    {
        reserved = WICED_TRUE;
    }

    duk_pop( ctx );  /* -> [] */

    return reserved;
}

static duk_ret_t audio_object_read_only_setter( duk_context* ctx )
{
    LOGE( "Trying to set value to read-only property" );
    return DUK_RET_TYPE_ERROR;
}

static duk_ret_t audio_object_current_time_getter( duk_context* ctx )
{
    /* Get internal property 'currentTime' from 'this' */
    duk_push_this( ctx ); /* -> [this] */
    duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CURRENTTIME );
    /* -> [this currentTime] */
    duk_remove( ctx, -2 ); /* -> [currentTime] */

    return 1;
}

static duk_ret_t audio_object_current_time_setter( duk_context* ctx )
{
    audio_object_client_t*  audio_client;
    double                  time;
    uint32_t                time_ms;

    time = duk_require_number( ctx, 0 );

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        return DUK_RET_ERROR;
    }

    /* Seek only if we have reserved the audio_client */
    if ( audio_object_client_is_reserved( ctx, audio_client ))
    {
        uint32_t is_idle;

        LOGD( "Seeking to %f s", time );
        if ( audio_client_ioctl( audio_client->ref,
                                 AUDIO_CLIENT_IOCTL_IDLE_CHECK,
                                 &is_idle ) != WICED_SUCCESS )
        {
            LOGE( "Failed to get audio_client state" );
            goto out;
        }

        if ( is_idle != 0 )
        {
            LOGE( "Cannot seek while audio_client is idle" );
            goto out;
        }

        time_ms = (uint32_t)( time * 1000.0 );

        LOGD( "Seeking audio_client to %u ms", time_ms );
        if ( audio_client_ioctl( audio_client->ref,
                                 AUDIO_CLIENT_IOCTL_SEEK,
                                 (void*)time_ms ) != WICED_SUCCESS )
        {
            LOGE( "Failed to seek to %f s", time );
        }
    }

out:
    return 0;
}

static duk_ret_t audio_object_played_getter( duk_context* ctx )
{
    /* Generate the 'played' TimeRanges object from the 'currentTime' property
     * Notes:
     *  - If no audio is loaded, i.e., 'currentTime' is 0.0, then report back no
     *    played ranges
     *  - If audio is loaded, report back a single played range equal to the
     *    'currentTime'
     */

    audio_object_current_time_getter( ctx ); /* -> [currentTime] */

    duk_push_object( ctx ); /* -> [currentTime played] */
    if ( duk_get_number( ctx, -1 ) > 0.0 )
    {
        duk_push_int( ctx, 1 ); /* -> [currentTime played length] */
        duk_put_prop_string( ctx, -2, "length" ); /* -> [currentTime played] */

        duk_push_array( ctx ); /* -> [currentTime played start] */
        duk_push_number( ctx, 0.0 ); /* -> [currentTime played start time] */
        duk_put_prop_index( ctx, -2, 0 ); /* -> [currentTime played start] */
        duk_put_prop_string( ctx, -2, "start" ); /* -> [currentTime played] */

        duk_push_array( ctx ); /* -> [currentTime played end] */
        duk_dup( ctx, -3 ); /* -> [currentTime played end time] */
        duk_put_prop_index( ctx, -2, 0 ); /* -> [currentTime played end] */
        duk_put_prop_string( ctx, -2, "end" ); /* -> [currentTime played] */
    }
    else
    {
        duk_push_int( ctx, 0 ); /* -> [currentTime played length] */
        duk_put_prop_string( ctx, -2, "length" ); /* -> [currentTime played] */
    }

    duk_remove( ctx, -2 ); /* -> [played] */

    return 1;
}

static duk_ret_t audio_object_seekable_getter( duk_context* ctx )
{
    /* Generate the 'seekable' TimeRanges object from the 'duration' property
     * Notes:
     *  - If no audio is loaded, i.e., 'duration' is NaN, then report back no
     *    seekable ranges
     *  - If audio is loaded, report back a single seekable range equal to the
     *    'duration'
     */

    duk_push_this( ctx ); /* -> [this] */
    duk_get_prop_string( ctx, -1, PROPERTY_DURATION ); /* -> [this duration] */
    duk_remove( ctx, -2 ); /* -> [duration] */

    duk_push_object( ctx ); /* -> [duration seekable] */
    if ( duk_is_nan( ctx, -1 ))
    {
        duk_push_int( ctx, 0 ); /* -> [duration seekable length] */
        duk_put_prop_string( ctx, -2, "length" ); /* -> [duration seekable] */
    }
    else
    {
        duk_push_int( ctx, 1 ); /* -> [duration seekable length] */
        duk_put_prop_string( ctx, -2, "length" ); /* -> [duration seekable] */

        duk_push_array( ctx ); /* -> [duration seekable start] */
        duk_push_number( ctx, 0.0 ); /* -> [duration seekable start time] */
        duk_put_prop_index( ctx, -2, 0 ); /* -> [duration seekable start] */
        duk_put_prop_string( ctx, -2, "start" ); /* -> [duration seekable] */

        duk_push_array( ctx ); /* -> [duration seekable end] */
        duk_dup( ctx, -3 ); /* -> [duration seekable end time] */
        duk_put_prop_index( ctx, -2, 0 ); /* -> [duration seekable end] */
        duk_put_prop_string( ctx, -2, "end" ); /* -> [duration seekable] */
    }
    /* -> [duration seekable] */

    duk_remove( ctx, -2 ); /* -> [seekable] */

    return 1;
}

static duk_ret_t audio_object_volume_getter( duk_context* ctx )
{
    /* Get internal property 'volume' from 'this' */
    duk_push_this( ctx ); /* -> [this] */
    duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_VOLUME );
    /* -> [this volume] */
    duk_remove( ctx, -2 ); /* -> [volume] */

    return 1;
}

static duk_ret_t audio_object_volume_setter( duk_context* ctx )
{
    audio_object_client_t*  audio_client;
    duk_double_t            val;

    val = duk_require_number( ctx, 0 );

    LOGD( "New 'volume' value '%f'", val );

    if ( val < 0.0 )
    {
        LOGW( "Invalid 'volume' value '%f'", val );
        val = 0.0;
    }
    else if ( val > 1.0 )
    {
        LOGW( "Invalid 'volume' value '%f'", val );
        val = 1.0;
    }

    /* Save new volume to 'this' */
    duk_push_this( ctx ); /* -> [this] */
    duk_push_number( ctx, val ); /* -> [this volume] */
    duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_VOLUME ); /* -> [this] */

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        return DUK_RET_ERROR;
    }

    /* Set the volume now if we have reserved the audio_client */
    if ( audio_object_client_is_reserved( ctx, audio_client ))
    {
        int volume = (uint32_t)(val * 100.0);

        LOGI( "Setting playback volume to %d", volume );

        if ( audio_client_ioctl( audio_client->ref,
                                 AUDIO_CLIENT_IOCTL_SET_VOLUME,
                                 (void*)volume ) != WICED_SUCCESS )
        {
            LOGE( "Failed to set playback volume" );
        }

        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
        /* -> [this callbacks] */
        if ( duk_has_prop_string( ctx, -1, EVENT_VOLUMECHANGE ))
        {
            duk_get_prop_string( ctx, -1, EVENT_VOLUMECHANGE );
            /* -> [this callbacks volumechange] */
            duk_dup( ctx, -3 );
            /* -> [this callbacks volumechange this] */

            if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
            {
                /* -> [this callbacks] */

                LOGE( "Failed to call volumechange callback (%s)",
                      duk_safe_to_string( ctx, -1 ));
            }
            /* -> [this callbacks retval/err] */

            duk_pop( ctx ); /* -> [this callbacks] */
        }
        duk_pop( ctx ); /* -> [this] */
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static wiced_result_t audio_object_play_thread( void* arg )
{
    duk_context*                ctx;
    wiced_timed_event_t*        play_thread = arg;
    audio_object_client_t*      audio_client;
    wiced_duktape_state_t       state;
    audio_client_track_info_t   info;
    double                      duration = 0;

    ctx = wiced_duktape_get_control( &state );
    if ( ctx == NULL )
    {
        LOGW( "Duktape heap destroyed- stopping play thread" );
        wiced_rtos_deregister_timed_event( play_thread );
        return WICED_SUCCESS;
    }

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        goto control_put;
    }

    /* Get the current track info from audio_client */
    if ( audio_client_ioctl( audio_client->ref, AUDIO_CLIENT_IOCTL_TRACK_INFO,
                             &info ) != WICED_SUCCESS )
    {
        LOGW( "Failed to get track info from audio_client" );
        goto control_put;
    }

    /* Push the stored 'this' on stack */
    duk_push_heapptr( ctx, audio_client->user ); /* -> [this] */

    duk_get_prop_string( ctx, -1, PROPERTY_DURATION ); /* -> [this duration] */
    if ( duk_is_nan( ctx, -1 ))
    {
        if ( info.total_samples != 0 )
        {
            duration = (double)info.sample_rate / (double)info.total_samples ;
        }
        else if (( info.current_sample > 0 ) && ( info.total_samples == 0 ) &&
                 ( info.bitrate != 0 ))
        {
            duration = (double)audio_client->ref->http_params.http_total_content_length /
                       ( (double)info.bitrate / 8.0 );
        }

        if ( duration != 0 )
        {
            duk_push_string( ctx, PROPERTY_DURATION );
            /* -> [this duration propName] */
            duk_push_number( ctx, duration );
            /* -> [this duration propName NEWduration] */
            duk_def_prop( ctx, -4, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this duration] */

            duk_get_prop_string( ctx, -2, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this duration callbacks] */
            if ( duk_has_prop_string( ctx, -1, EVENT_DURATIONCHANGE ))
            {
                duk_get_prop_string( ctx, -1, EVENT_DURATIONCHANGE );
                /* -> [this duration callbacks durationchange] */
                duk_dup( ctx, -4 );
                /* -> [this duration callbacks durationchange this] */

                if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
                {
                    /* -> [this duration callbacks err] */

                    LOGE( "Failed to call durationchange callback (%s)",
                          duk_safe_to_string( ctx, -1 ));
                }
                /* -> [this duration callbacks retval/err] */

                duk_pop( ctx ); /* -> [this duration callbacks] */
            }

            duk_pop( ctx ); /* -> [this duration] */
        }
    }
    /* -> [this duration] */

    duk_pop( ctx ); /* -> [this] */

#ifdef DEBUG
    LOGD( "Track info:" );
    LOGD( "  current sample : %u", (uint32_t)info.current_sample );
    LOGD( "  total samples  : %u", (uint32_t)info.total_samples );
    LOGD( "  sample rate    : %u", info.sample_rate );
    LOGD( "  channels       : %hhu", info.channels );
    LOGD( "  bps            : %hhu", info.bps );
    LOGD( "  bitrate        : %u", info.bitrate );
#endif

    /* Update the current time */
    if ( info.current_sample > 0 )
    {
        double current_time;

        current_time  = (double)info.current_sample / (double)info.sample_rate;

        LOGD( "audio_client playback currentTime=%f s", current_time );

        duk_push_number( ctx, current_time );
        /* -> [this currentTime] */
        duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_CURRENTTIME );
        /* -> [this] */

        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
        /* -> [this callbacks] */
        if ( duk_has_prop_string( ctx, -1, EVENT_TIMEUPDATE ))
        {
            duk_get_prop_string( ctx, -1, EVENT_TIMEUPDATE );
            /* -> [this callbacks timeupdate] */
            duk_dup( ctx, -3 );
            /* -> [this callbacks timeupdate this] */

            if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
            {
                /* -> [this callbacks] */

                LOGE( "Failed to call timeupdate callback (%s)",
                      duk_safe_to_string( ctx, -1 ));
            }
            /* -> [this callbacks retval/err] */

            duk_pop( ctx ); /* -> [this callbacks] */
        }
        duk_pop( ctx ); /* -> [this] */
    }

    duk_pop( ctx ); /* -> [] */

control_put:
    wiced_duktape_put_control( &state );

    return WICED_SUCCESS;
}

static wiced_result_t audio_object_client_callback( audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg )
{
    wiced_result_t              result = WICED_SUCCESS;
    duk_context*                ctx;
    audio_object_client_t*      audio_client;
    duk_idx_t                   this_idx;
    duk_idx_t                   callbacks_idx;
    int                         num_callbacks = 0;
    wiced_duktape_state_t       state;

    (void)handle;

    LOGD( "audio_client callback called (event=%u)", event );

    /* FIXME: We really should not be calling a potentially blocking call here;
     *        ideally we should do all critical tasks here, then schedule
     *        another thread to talk to Duktape
     */
    ctx = wiced_duktape_get_control( &state );
    if ( ctx == NULL )
    {
        LOGW( "Duktape heap destroyed- ignoring audio_client callback" );
        return WICED_SUCCESS;
    }

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        result = WICED_ERROR;
        goto put_control;
    }

    if ( audio_client->user == NULL )
    {
        LOGW( "No active audio_client user- ignoring event" );
        goto put_control;
    }

    /* Push the stored 'this' on stack */
    this_idx = duk_push_heapptr( ctx, audio_client->user ); /* -> [this] */

    switch ( event )
    {
        case AUDIO_CLIENT_EVENT_ERROR:
        {
            LOGE( "Error from audio_client: %d", (int)arg );

            /* Stop the play thread */
            wiced_rtos_deregister_timed_event( &audio_client->play_thread );

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            callbacks_idx = duk_get_top_index( ctx );

            if ( duk_has_prop_string( ctx, -1, EVENT_ERROR ))
            {
                duk_get_prop_string( ctx, -1, EVENT_ERROR );
                /* -> [this callbacks error] */
                num_callbacks++;
            }

            duk_remove( ctx, callbacks_idx ); /* -> [this ...] */
            break;
        }
        case AUDIO_CLIENT_EVENT_CONNECTED:
        {
            LOGD( "AUDIO_CLIENT_EVENT_CONNECTED" );

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            callbacks_idx = duk_get_top_index( ctx );

            if ( duk_has_prop_string( ctx, -1, EVENT_CANPLAY ))
            {
                duk_get_prop_string( ctx, -1, EVENT_CANPLAY );
                /* -> [this callbacks canplay] */
                num_callbacks++;
            }

            duk_remove( ctx, callbacks_idx ); /* -> [this ...] */
            break;
        }
        case AUDIO_CLIENT_EVENT_AUDIO_FORMAT:
        {
            wiced_audio_config_t*   config;

            LOGD( "AUDIO_CLIENT_EVENT_AUDIO_FORMAT" );

            config = (wiced_audio_config_t*)arg;
            if (config == NULL)
            {
                return WICED_BADARG;
            }

            LOGD( "Audio format:" );
            LOGD( "  num channels    : %u", config->channels );
            LOGD( "  bits per sample : %u", config->bits_per_sample );
            LOGD( "  sample rate     : %lu", config->sample_rate );
            LOGD( "  frame size      : %u", config->frame_size );

            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_STARTED:
        {
            LOGD( "AUDIO_CLIENT_EVENT_PLAYBACK_STARTED" );

            /* Either started or resumed playing */

            LOGD( "Started playback- starting play thread" );
            if ( wiced_rtos_register_timed_event( &audio_client->play_thread,
                                                  WICED_DUKTAPE_WORKER_THREAD,
                                                  audio_object_play_thread,
                                                  500,
                                                  &audio_client->play_thread )
                 != WICED_SUCCESS )
            {
                LOGE( "Failed to create audio play thread" );
            }

            /* Update 'ended' property */
            duk_push_string( ctx, PROPERTY_ENDED ); /* -> [this propName] */
            duk_push_boolean( ctx, 0 ); /* -> [this propName ended] */
            duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            callbacks_idx = duk_get_top_index( ctx );

            if ( duk_has_prop_string( ctx, -1, EVENT_PLAY ))
            {
                duk_get_prop_string( ctx, -1, EVENT_PLAY );
                /* -> [this callbacks play] */
                num_callbacks++;
            }

            if ( duk_has_prop_string( ctx, -2, EVENT_PLAYING ))
            {
                duk_get_prop_string( ctx, -2, EVENT_PLAYING );
                /* -> [this callbacks play playing] */
                num_callbacks++;
            }

            duk_remove( ctx, callbacks_idx ); /* -> [this ...] */
            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED:
        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
        {
            if ( event == AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED )
            {
                LOGD( "AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED" );
            }
            else if ( event == AUDIO_CLIENT_EVENT_PLAYBACK_EOS )
            {
                LOGD( "AUDIO_CLIENT_EVENT_PLAYBACK_EOS" );
            }

            /* We are done with audio_client, release it for others to use */
            audio_object_client_release( ctx, audio_client,
                                         audio_client->user );

            /* Stop the play thread */
            if ( wiced_rtos_deregister_timed_event( &audio_client->play_thread ) !=
                 WICED_SUCCESS )
            {
                LOGW( "Failed to stop play thread" );
            }

            /* Reset 'duration' to NaN */
            duk_push_string( ctx, PROPERTY_DURATION );
            /* -> [this propName] */
            duk_push_nan( ctx ); /* -> [this propName duration] */
            duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */

            /* Reset 'currentTime' */
            duk_push_number( ctx, 0.0 ); /* -> [this currentTime] */
            duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_CURRENTTIME );
            /* -> [this] */

            /* Update 'ended' */
            duk_push_string( ctx, PROPERTY_ENDED );
            /* -> [this propName] */
            duk_push_boolean( ctx, 1 ); /* -> [this propName ended] */
            duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            callbacks_idx = duk_get_top_index( ctx );

            if ( duk_has_prop_string( ctx, -1, EVENT_ENDED ))
            {
                duk_get_prop_string( ctx, -1, EVENT_ENDED );
                /* -> [this callbacks ended] */
                num_callbacks++;
            }

            duk_remove( ctx, callbacks_idx ); /* -> [this ...] */
            break;
        }
        case AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED:
        {
            LOGD( "AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED" );

            /* Stop the play thread */
            if ( wiced_rtos_deregister_timed_event( &audio_client->play_thread ) !=
                 WICED_SUCCESS )
            {
                LOGW( "Failed to stop play thread" );
            }

            /* Update 'paused' */
            duk_push_string( ctx, PROPERTY_PAUSED ); /* -> [this propName] */
            duk_push_boolean( ctx, 1 ); /* -> [this propName paused] */
            duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            callbacks_idx = duk_get_top_index( ctx );

            if ( duk_has_prop_string( ctx, -1, EVENT_PAUSE ))
            {
                duk_get_prop_string( ctx, -1, EVENT_PAUSE );
                /* -> [this callbacks pause] */
                num_callbacks++;
            }

            duk_remove( ctx, callbacks_idx ); /* -> [this ...] */
            break;
        }
        case AUDIO_CLIENT_EVENT_HTTP_COMPLETE:
        {
            LOGD( "AUDIO_CLIENT_EVENT_HTTP_COMPLETE" );
            break;
        }
        case AUDIO_CLIENT_EVENT_HTTP_REDIRECT:
        {
            LOGD( "AUDIO_CLIENT_EVENT_HTTP_REDIRECT" );
            break;
        }
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
        {
            LOGD( "AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH" );
            break;
        }
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
        {
            LOGD( "AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW" );
            break;
        }
        case AUDIO_CLIENT_EVENT_AUDIO_CODEC:
        {
            LOGD( "AUDIO_CLIENT_EVENT_AUDIO_CODEC" );
            break;
        }
        default:
        {
            LOGW( "Unhandled audio_client event: %d", event );
            break;
        }
    }

    while ( num_callbacks )
    {
        /* -> [this <callbacks>] */

        duk_dup( ctx, (num_callbacks * -1 )); /* -> [this <callbacks> callback] */
        duk_dup( ctx, this_idx ); /* -> [this <callbacks> callback this] */

        if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
        {
            /* -> [<callbacks> err] */

            LOGE( "Failed to call audio event callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [this <callbacks> retval/err] */

        duk_pop( ctx ); /* -> [this <callbacks>] */
        duk_remove( ctx, (num_callbacks * -1 )); /* -> [this <callbacks>] */

        num_callbacks--;
    }

    /* Remove 'this' off stack */
    duk_remove( ctx, this_idx ); /* -> [] */

put_control:
    wiced_duktape_put_control( &state );

    return result;
}

static duk_ret_t audio_object_client_finalizer( duk_context* ctx )
{
    audio_object_client_t* audio_client;

    /* Arguments:
     * 0 = 'global'
     * 1 = heap destruction flag    (boolean)
     */

    if ( !duk_get_boolean( ctx, 1 ))
    {
        /* Free resources only when heap is destroyed */
        return 0;
    }

    if ( duk_has_prop_string( ctx, 0, GLOBAL_PROPERTY_AUDIO_CLIENT ))
    {
        duk_get_prop_string( ctx, 0, GLOBAL_PROPERTY_AUDIO_CLIENT );
        /* -> [audio_client] */

        LOGD( "Finalizer called ('audio_client'=%p)",
              duk_to_pointer( ctx, -1 ));

        audio_client = duk_to_pointer( ctx, -1 );
        duk_pop( ctx ); /* -> [] */

        if ( audio_client != NULL )
        {
            LOGD( "Stopping any playback threads" );
            wiced_rtos_deregister_timed_event( &audio_client->play_thread );

            LOGD( "Stopping audio playback" );
            audio_client_ioctl( audio_client->ref, AUDIO_CLIENT_IOCTL_STOP,
                                NULL );

            LOGD( "Deinitializing audio_client" );
            audio_client_deinit( audio_client->ref );

            LOGD( "Freeing audio_client memory" );
            free( audio_client );
        }
    }

    return 0;
}

static wiced_result_t audio_object_client_init( duk_context* ctx )
{
    audio_object_client_t*  audio_client;
    audio_client_params_t           params;

    /* Allocate memory for the audio_client structure */
    audio_client = calloc( 1, sizeof( audio_object_client_t ));
    if ( audio_client == NULL )
    {
        LOGE( "Failed to allocate memory for audio_client structure" );
        return WICED_ERROR;
    }

    LOGD( "Allocated memory for audio_client (%p)", audio_client );

    /* Set up parameters and initialize audio_client */
    memset( &params, 0, sizeof( audio_client_params_t ));
    params.event_cb                    = audio_object_client_callback;
    params.userdata                    = NULL;
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

    if (( audio_client->ref = audio_client_init( &params )) == NULL )
    {
        LOGE( "Unable to initialize audio_client" );
        return WICED_ERROR;
    }

    /* Save the audio_client structure in global stash */
    duk_push_global_stash( ctx ); /* -> [global] */
    duk_push_pointer( ctx, audio_client ); /* -> [global audio_client] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_AUDIO_CLIENT );
    /* -> [global] */

    /* Set the finalizer for 'global' */
    duk_push_c_function( ctx, audio_object_client_finalizer, 2 );
    /* -> [global finalizer] */
    duk_set_finalizer( ctx, -2 ); /* -> [global] */

    duk_pop( ctx ); /* -> [] */

    return WICED_SUCCESS;
}

static duk_ret_t audio_object_play( duk_context* ctx )
{
    duk_ret_t               ret = 0;
    audio_object_client_t*  audio_client;
    wiced_bool_t            is_idle = WICED_FALSE;

    /* Arguments:
     * None
     */

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        return DUK_RET_ERROR;
    }

    /* Check if we have already reserved the audio_client */
    if ( !audio_object_client_is_reserved( ctx, audio_client ))
    {
        is_idle = WICED_TRUE;

        /* Reserve the audio_client for use */
        if ( audio_object_client_reserve( ctx, audio_client ) != WICED_SUCCESS )
        {
            LOGE( "audio_client in use by another audio instance" );

            ret = DUK_RET_ERROR;
            goto out;
        }
    }

    if ( is_idle )
    {
        const char* src;
        duk_size_t  src_len;
        int         volume;

        /* Grab the 'src' property from 'this' */
        duk_push_this( ctx ); /* -> [this] */
        duk_get_prop_string( ctx, -1, PROPERTY_SRC ); /* -> [this src] */

        if ( duk_is_null( ctx, -1 ))
        {
            LOGE( "Missing 'src'" );
            ret = DUK_RET_ERROR;

            duk_pop_2( ctx ); /* -> [] */
            goto client_release;
        }

        src = duk_to_lstring( ctx, -1, &src_len );
        if ( src_len == 0 )
        {
            LOGE( "Invalid 'src'" );
            ret = DUK_RET_ERROR;

            duk_pop_2( ctx ); /* -> [] */
            goto client_release;
        }

        LOGD( "Sending PLAY IOCTL to audio_client" );
        if ( audio_client_ioctl( audio_client->ref, AUDIO_CLIENT_IOCTL_PLAY,
                                 (void*)src ) == WICED_SUCCESS )
        {
            duk_pop( ctx ); /* -> [this] */

            /* Update 'paused' */
            duk_push_string( ctx, PROPERTY_PAUSED ); /* -> [this propName] */
            duk_push_boolean( ctx, 0 ); /* -> [this propName paused] */
            duk_def_prop( ctx, -3,
                          DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */
        }
        else
        {
            LOGE( "audio_client failed to start audio playback" );
            ret = DUK_RET_ERROR;

            duk_pop( ctx ); /* -> [this] */

            /* Call 'error' event callback */
            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            if ( duk_has_prop_string( ctx, -1, EVENT_ERROR ))
            {
                duk_get_prop_string( ctx, -1, EVENT_ERROR );
                /* -> [this callbacks error] */
                duk_push_this( ctx ); /* -> [this callbacks error this] */

                if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
                {
                    /* -> [this callbacks err] */

                    LOGE( "Failed to call audio event callback (%s)",
                          duk_safe_to_string( ctx, -1 ));
                }
                /* -> [this callbacks retval/err] */

                duk_pop( ctx ); /* -> [this callbacks] */
            }
            /* -> [this callbacks] */

            duk_pop_2( ctx ); /* -> [] */
            goto client_release;
        }

        /* -> [this] */

        /* Get internal 'volume' from 'this' */
        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_VOLUME );
        /* -> [this volume] */

        volume = (uint32_t)( duk_to_number( ctx, -1) * 100.0 );

        LOGI( "Setting playback volume to %d", volume );
        if ( audio_client_ioctl( audio_client->ref,
                                 AUDIO_CLIENT_IOCTL_SET_VOLUME,
                                 (void*)volume ) != WICED_SUCCESS )
        {
            LOGE( "Failed to set playback volume" );
        }

        duk_pop_2( ctx ); /* -> [] */
    }
    else
    {
        /* We are either playing or paused */

        duk_push_this( ctx ); /* -> [this] */
        duk_get_prop_string( ctx, -1, PROPERTY_PAUSED ); /* -> [this paused] */

        if ( duk_get_boolean( ctx, -1 ))
        {
            duk_pop( ctx ); /* -> [this] */

            LOGD( "Sending RESUME IOCTL to audio_client" );
            if ( audio_client_ioctl( audio_client->ref,
                                     AUDIO_CLIENT_IOCTL_RESUME,
                                     NULL ) == WICED_SUCCESS )
            {
                /* Update 'paused' */
                duk_push_string( ctx, PROPERTY_PAUSED );
                /* -> [this propName] */
                duk_push_boolean( ctx, 0 ); /* -> [this propName paused] */
                duk_def_prop( ctx, -3,
                              DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
                /* -> [this] */
            }
            else
            {
                LOGE( "audio_client failed to resume audio playback!" );

                duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
                /* -> [this callbacks] */
                if ( duk_has_prop_string( ctx, -1, EVENT_ERROR ))
                {
                    duk_get_prop_string( ctx, -1, EVENT_ERROR );
                    /* -> [this callbacks error] */
                    duk_push_this( ctx ); /* -> [this callbacks error this] */

                    if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
                    {
                        /* -> [this callbacks err] */

                        LOGE( "Failed to call audio event callback (%s)",
                              duk_safe_to_string( ctx, -1 ));
                    }
                    /* -> [this callbacks retval/err] */

                    duk_pop( ctx ); /* -> [this callbacks] */
                }

                duk_pop( ctx ); /* -> [this] */
            }
        }
        else
        {
            LOGW( "audio_client already playing" );

            duk_pop( ctx ); /* -> [this] */
        }

        duk_pop( ctx ); /* -> [] */
    }

client_release:
    if ( ret != 0 )
    {
        audio_object_client_release( ctx, audio_client, NULL );
    }

out:
    return ret;
}

static duk_ret_t audio_object_load( duk_context* ctx )
{
    audio_object_client_t*  audio_client;

    /* Arguments:
     * None
     */

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        return DUK_RET_ERROR;
    }

    if ( !audio_object_client_is_reserved( ctx, audio_client ))
    {
        goto out;
    }

    LOGD( "Stopping audio playback" );
    audio_client_ioctl( audio_client->ref, AUDIO_CLIENT_IOCTL_STOP, NULL );

    audio_object_client_release( ctx, audio_client, NULL );

out:
    return audio_object_play( ctx );
}

static duk_ret_t audio_object_pause( duk_context* ctx )
{
    duk_ret_t               ret = 0;
    audio_object_client_t*  audio_client;

    /* Arguments:
     * None
     */

    /* Grab audio_client from global stash */
    audio_client = audio_object_client_get( ctx );
    if ( audio_client == NULL )
    {
        LOGE( "Failed to get audio_client" );
        return DUK_RET_ERROR;
    }

    if ( !audio_object_client_is_reserved( ctx, audio_client ))
    {
        LOGW( "Trying to pause audio_client that is not reserved" );
        ret = DUK_RET_ERROR;
        goto out;
    }

    /* We are either playing or paused */

    duk_push_this( ctx ); /* -> [this] */
    duk_get_prop_string( ctx, -1, PROPERTY_PAUSED ); /* -> [this paused] */

    if ( duk_get_boolean( ctx, -1 ))
    {
        LOGW( "audio_client already paused" );

        duk_pop_2( ctx ); /* -> [] */
    }
    else
    {
        duk_pop( ctx ); /* -> [this] */

        LOGD( "Sending PAUSE IOCTL to audio_client" );
        if ( audio_client_ioctl( audio_client->ref,
                                 AUDIO_CLIENT_IOCTL_PAUSE,
                                 NULL ) == WICED_SUCCESS )
        {
            /* Update 'paused' */
            duk_push_string( ctx, PROPERTY_PAUSED );
            /* -> [this propName] */
            duk_push_boolean( ctx, 1 ); /* -> [this propName paused] */
            duk_def_prop( ctx, -3,
                          DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this] */

            duk_pop( ctx ); /* -> [] */
        }
        else
        {
            LOGE( "audio_client failed to pause audio playback!" );

            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */
            if ( duk_has_prop_string( ctx, -1, EVENT_ERROR ))
            {
                duk_get_prop_string( ctx, -1, EVENT_ERROR );
                /* -> [this callbacks error] */
                duk_push_this( ctx ); /* -> [this callbacks error this] */

                if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
                {
                    /* -> [this callbacks err] */

                    LOGE( "Failed to call audio event callback (%s)",
                          duk_safe_to_string( ctx, -1 ));
                }
                /* -> [this callbacks retval/err] */

                duk_pop( ctx ); /* -> [this callbacks] */
            }

            duk_pop_2( ctx ); /* -> [] */
        }
    }

out:
    return ret;
}

static duk_ret_t audio_object_set_attribute( duk_context* ctx )
{
    const char* key;

    key = duk_require_string( ctx, 0 );
    duk_require_object_coercible( ctx, 1 );

    LOGD( "Setting attribute for '%s'", key );

    duk_push_this( ctx ); /* -> [this] */

    /* Duplicate the value from arguments */
    duk_dup( ctx, 1 ); /* -> [this val] */
    duk_put_prop_string( ctx, -2, key ); /* -> [this] */

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t audio_object_add_event_listener( duk_context* ctx )
{
    const char**    supported_event = audio_object_events;
    const char*     event;
    duk_size_t      event_len;

    /* Arguments:
     * 0 = event    (string, required)
     * 1 = callback (function, required)
     * 2 = options  (boolean, ignored for now FIXME)
     */

    event = duk_require_lstring( ctx, 0, &event_len );
    duk_require_function( ctx, 1 );

    while ( *supported_event != NULL )
    {
        if (( strlen( *supported_event) == event_len ) &&
            ( strstr( *supported_event, event ) != NULL ))
        {
            /* Save callback to internal property 'callbacks' on 'this' */
            duk_push_this( ctx ); /* -> [this] */
            duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this callbacks] */

            if ( duk_has_prop_string( ctx, -1, *supported_event ))
            {
                LOGW( "Overriding callback for event '%s'", *supported_event );
            }

            duk_dup( ctx, 1 ); /* -> [this callbacks callback] */
            duk_put_prop_string( ctx, -2, *supported_event );
            /* -> [this callbacks] */

            duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_CALLBACKS );
            /* -> [this] */

            LOGD( "Saved callback for event '%s'", *supported_event );

            duk_pop( ctx ); /* -> [] */

            return 0;
        }

        supported_event++;
    }

    LOGW( "Ignoring registering listener for unsupported event '%s'", event );

    return 0;
}

static duk_function_list_entry audio_object_funcs[] =
{
      /* Name */            /* Function */                      /* nargs */
    { "load",               audio_object_load,                  0 },
    { "play",               audio_object_play,                  0 },
    { "pause",              audio_object_pause,                 0 },
    { "setAttribute",       audio_object_set_attribute,         2 },
    { "addEventListener",   audio_object_add_event_listener,    2 },
    { NULL, NULL, 0 }
};

static duk_ret_t audio_object_constructor( duk_context* ctx )
{
    /* Arguments:
     * 0 = src  (optional)
     */

    if ( !duk_is_constructor_call( ctx ))
    {
        LOGE( "Called as a normal function (non-constructor)" );

        /* Reject non-constructor call */
        return DUK_RET_TYPE_ERROR;
    }

    /* Initialize global instance of audio_client if needed
     * Notes:
     *  - Only one instance of the audio_client is supported so we place it in
     *    the global stash
     *  - We don't check for instances of audio_client outside of this Duktape
     *    heap, so it's possible it may fail
     *  - If the rest of the object constructor fails, we can still leave the
     *    audio_client in the global stash for the next instance of this object
     */
    LOGD( "Initializing an instance of audio_client" );
    duk_push_global_stash( ctx ); /* -> [global] */
    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_AUDIO_CLIENT ))
    {
        LOGD( "An audio_client instance already exists" );
    }
    else
    {
        /* Initialize instance of audio_client */
        if ( audio_object_client_init( ctx ) != WICED_SUCCESS )
        {
            duk_pop( ctx ); /* -> [] */

            LOGE( "Failed to create audio_client" );
            return DUK_RET_ERROR;
        }
    }

    duk_pop( ctx ); /* -> [] */

    LOGD( "Initializing object properties" );

    /* Initialize object properties to default values
     * Notes:
     *  - Only a subset of possible properties are implemented at this time
     *  - Properties that trigger immediate actions upon modifications, e.g.
     *    currentTime to jump to position in time, should be implemented as
     *    accessor (getter/setter) properties
     *  - All other properties should be plain properties
     *  - RO accessor properties should use a setter that returns a type error
     *  - RO plain properties should set the property to be non-writable, and
     *    use force the property value update internally when needed
     */

    duk_push_this( ctx ); /* -> [this] */

    /* [RW] autoplay: whether audio should play as soon as it's loaded */
    duk_push_string( ctx, PROPERTY_AUTOPLAY ); /* -> [this autoplay] */
    duk_push_boolean( ctx, 0 ); /* -> [this autoplay bool] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_ENUMERABLE |
                  DUK_DEFPROP_SET_WRITABLE ); /* -> [this] */

    /* [RW] currentTime: position of audio playback in seconds */
    duk_push_string( ctx, PROPERTY_CURRENTTIME ); /* -> [this propName] */
    duk_push_c_function( ctx, audio_object_current_time_getter, 0 );
    /* -> [this propName currentTimeGetter] */
    duk_push_c_function( ctx, audio_object_current_time_setter, 1 );
    /* -> [this propName currentTimeGetter currentTimeSetter] */
    duk_def_prop( ctx, -4,
                  DUK_DEFPROP_HAVE_GETTER | DUK_DEFPROP_HAVE_SETTER |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] duration: length of current audio in seconds */
    duk_push_string( ctx, PROPERTY_DURATION ); /* -> [this propName] */
    duk_push_nan( ctx ); /* -> [this propName duration] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] ended: true if playback has ended */
    duk_push_string( ctx, PROPERTY_ENDED ); /* -> [this propName] */
    duk_push_boolean( ctx, 0 ); /* -> [this propName ended] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] paused: whether audio is paused */
    duk_push_string( ctx, PROPERTY_PAUSED ); /* -> [this propName] */
    duk_push_boolean( ctx, 0 ); /* -> [this propName paused] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] played: TimeRanges object of audio that have been played */
    duk_push_string( ctx, PROPERTY_PLAYED ); /* -> [this propName] */
    duk_push_c_function( ctx, audio_object_played_getter, 0 );
    /* -> [this propName playedGetter] */
    duk_push_c_function( ctx, audio_object_read_only_setter, 1 );
    /* -> [this propName playedGetter readonlySetter] */
    duk_def_prop( ctx, -4,
                  DUK_DEFPROP_HAVE_GETTER | DUK_DEFPROP_HAVE_SETTER |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RW] preload: whether audio is loaded as soon as page loads */
    duk_push_string( ctx, PROPERTY_PRELOAD ); /* -> [this propName] */
    duk_push_string( ctx, "none" ); /* -> [this propName preload] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] readyState: current ready state of audio */
    duk_push_string( ctx, PROPERTY_READYSTATE ); /* -> [this propName] */
    duk_push_int( ctx, 0 ); /* -> [this propName readyState] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] seekable: TimeRanges object of audio available for seeking */
    duk_push_string( ctx, PROPERTY_SEEKABLE ); /* -> [this propName] */
    duk_push_c_function( ctx, audio_object_seekable_getter, 0 );
    /* -> [this propName seekableGetter] */
    duk_push_c_function( ctx, audio_object_read_only_setter, 1 );
    /* -> [this propName seekableGetter readonlySetter] */
    duk_def_prop( ctx, -4,
                  DUK_DEFPROP_HAVE_GETTER | DUK_DEFPROP_HAVE_SETTER |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RW] src: source of audio */
    duk_push_string( ctx, PROPERTY_SRC ); /* -> [this propName] */
    if ( duk_is_string( ctx, 0 ))
    {
        duk_dup( ctx, 0 ); /* -> [this propName src] */
    }
    else
    {
        duk_push_null( ctx ); /* -> [this propName src] */
    }
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RW] volume: volume of audio [0.0 - 1.0] */
    duk_push_string( ctx, PROPERTY_VOLUME ); /* -> [this propName] */
    duk_push_c_function( ctx, audio_object_volume_getter, 0 );
    /* -> [this propName volumeGetter] */
    duk_push_c_function( ctx, audio_object_volume_setter, 1 );
    /* -> [this propName volumeGetter volumeSetter] */
    duk_def_prop( ctx, -4,
                  DUK_DEFPROP_HAVE_GETTER | DUK_DEFPROP_HAVE_SETTER |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* The following are internal properties that are not enumerable by
     * Ecmascript
     */

    /* [INTERNAL] callbacks: object containing callbacks */
    duk_push_object( ctx ); /* -> [this callbacks] */
    duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_CALLBACKS ); /* -> [this] */

    /* [INTERNAL] currentTime: current playback time in seconds */
    duk_push_number( ctx, 0.0 ); /* -> [this currentTime] */
    duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_CURRENTTIME );
    /* -> [this] */

    /* [INTERNAL] volume: volume of audio */
    duk_push_number( ctx, 1.0 ); /* -> [this volume] */
    duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_VOLUME ); /* -> [this] */

    duk_pop( ctx ); /* -> [] */

    LOGD( "Done initializing object properties" );

    return 0;
}

void wiced_duktape_object_audio_init( duk_context* ctx )
{
    duk_push_c_function( ctx, audio_object_constructor, 1 ); /* -> [func] */

    duk_push_object( ctx ); /* -> [func obj] */
    duk_put_function_list( ctx, -1, audio_object_funcs );
    duk_put_prop_string( ctx, -2, "prototype" ); /* -> [func] */

    duk_put_global_string( ctx, "Audio" ); /* -> [] */
}
