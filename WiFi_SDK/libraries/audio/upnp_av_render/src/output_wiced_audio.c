/*
 * $ Copyright Broadcom Corporation $
 */

/* output_wiced_audio.c - Output module using the WICED audio framework
 * derived from output_gstreamer.c - Output module for GStreamer
 *
 * Copyright (C) 2005-2007   Ivo Clarysse
 *
 * Adapted to gstreamer-0.10 2006 David Siorpaes
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wiced_rtos.h"
#include "upnp_av_render.h"

#include "logging.h"
#include "upnp_connmgr.h"
#include "output_module.h"
#include "output_wiced_audio.h"
#include "audio_client_private.h"

#define NSECS_PER_SEC_FLOAT                   (1000000000.0)
#define NSECS_PER_MSEC                        (1000000LL)
#define AUDIO_CLIENT_IOCTL_STOP_TIMEOUT_MSECS (10000)
#define UPNP_CONTROL_VOL_MIN_DB               (-60.0)
#define UPNP_CONTROL_VOL_MID_DB               (-20.0)
#define UPNP_CONTROL_VOL_MAX_DB               (+0.0)
#define UPNP_CONTROL_VOL_MIN                  (0)
#define UPNP_CONTROL_VOL_MID                  (50)
#define UPNP_CONTROL_VOL_MAX                  (100)

#define AUDIO_PLAYER_NUM_HTTP_BUFFERS         (200)
#define AUDIO_PLAYER_NUM_AUDIO_BUFFERS        (80)
#define AUDIO_PLAYER_SIZE_AUDIO_BUFFERS       (2048)

struct track_time_info
{
	int64_t duration;
	int64_t position;
};

typedef struct
{
    char                         *uri;
    char                         *next_uri;
    struct SongMetaData           song_meta;
    output_transition_cb_t        play_trans_callback;
    output_update_meta_cb_t       meta_update_callback;
    struct track_time_info        last_known_time;
    audio_client_params_t         audio_client_params;
    audio_client_ref              audio_client_handle;
    AUDIO_CLIENT_EVENT_T          playback_status;
    wiced_bool_t                  audio_configured;
    double                        volume_in_db;
    int                           is_muted;
    audio_client_event_cb_t       caller_event_cbf;
    audio_client_buf_get_cb_t     caller_buf_get_cbf;
    audio_client_buf_release_cb_t caller_buf_release_cbf;
    void                         *caller_userdata;
    upnpavrender_event_cb_t       gm_cbf;
    void                         *gm_user_context;
} wiced_audio_output_ctx_t;

static wiced_audio_output_ctx_t output_ctx;

static wiced_result_t output_audio_client_event_callback(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg)
{
    wiced_result_t            result         = WICED_SUCCESS;
    wiced_audio_output_ctx_t *output_ctx_ptr = (wiced_audio_output_ctx_t *)userdata;

    UNUSED_PARAMETER(handle);

    if ( output_ctx_ptr->caller_event_cbf != NULL )
    {
        result = output_ctx_ptr->caller_event_cbf(NULL, output_ctx_ptr->caller_userdata, event, arg);
    }

    switch (event)
    {
        case AUDIO_CLIENT_EVENT_AUDIO_FORMAT:
            Log_info("wiced_audio", ">>> audio_client CB AUDIO_FORMAT (res=%d)", (int)result);
            if ( result != WICED_SUCCESS )
            {
                output_ctx_ptr->audio_configured = WICED_FALSE;
            }
            else
            {
                output_ctx_ptr->audio_configured = WICED_TRUE;
            }
            break;

        case AUDIO_CLIENT_EVENT_ERROR:
            output_ctx_ptr->playback_status = AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED;
            Log_error("wiced_audio", ">>> audio_client CB error %d", (int)arg);
            if ( output_ctx_ptr->play_trans_callback != NULL )
            {
                output_ctx_ptr->play_trans_callback(PLAY_STOPPED);
            }
            break;

        case AUDIO_CLIENT_EVENT_CONNECTED:
            Log_info("wiced_audio", ">>> audio_client CB CONNECTED");
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STARTED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", ">>> audio_client CB STARTED (res=%d)", (int)result);
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", ">>> audio_client CB STOPPED (res=%d)", (int)result);
            if ( output_ctx_ptr->audio_configured == WICED_FALSE )
            {
                if ( output_ctx_ptr->play_trans_callback != NULL )
                {
                    output_ctx_ptr->play_trans_callback(PLAY_STOPPED);
                }
            }
            break;

        case AUDIO_CLIENT_EVENT_DECODE_COMPLETE:
            output_ctx_ptr->playback_status = AUDIO_CLIENT_EVENT_PLAYBACK_STARTED;
            Log_info("wiced_audio", ">>> audio_client CB DECODE_COMPLETE (res=%d)", (int)result);
            if ( output_ctx_ptr->next_uri == NULL )
            {
                Log_info("wiced_audio", ">>> audio_client CB we DON't have the NEXT URI");
                if ( output_ctx_ptr->play_trans_callback != NULL )
                {
                    output_ctx_ptr->play_trans_callback(PLAY_STOPPED);
                }
            }
            else
            {
                Log_info("wiced_audio", ">>> audio_client CB we DO have the NEXT URI");
                free(output_ctx_ptr->uri);
                output_ctx_ptr->uri = output_ctx_ptr->next_uri;
                output_ctx_ptr->next_uri = NULL;
                result = audio_client_ioctl(output_ctx_ptr->audio_client_handle, AUDIO_CLIENT_IOCTL_PLAY, output_ctx_ptr->uri);
                if ( result != WICED_SUCCESS )
                {
                    Log_error("wiced_audio", ">>> audio_client CB AUDIO_CLIENT_IOCTL_PLAY failed with %d !", result);
                }
                else if ( output_ctx_ptr->play_trans_callback != NULL )
                {
                    output_ctx_ptr->play_trans_callback(PLAY_STARTED_NEXT_STREAM);
                }
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
            Log_info("wiced_audio", ">>> audio_client CB EOS (res=%d)", (int)result);
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", ">>> audio_client CB PAUSED");
            break;

        case AUDIO_CLIENT_EVENT_HTTP_COMPLETE:
            Log_info("wiced_audio", ">>> audio_client CB HTTP COMPLETE");
            break;

        default:
            break;
    }

    return result;
}

static wiced_result_t output_audio_client_buf_get_callback(audio_client_ref handle, void* userdata, audio_client_buf_t* ac_buf, uint32_t timeout_ms)
{
    wiced_result_t            result         = WICED_ERROR;
    wiced_audio_output_ctx_t *output_ctx_ptr = (wiced_audio_output_ctx_t *)userdata;

    UNUSED_PARAMETER(handle);

    if ( output_ctx_ptr->caller_buf_get_cbf != NULL )
    {
        result = output_ctx_ptr->caller_buf_get_cbf(NULL, output_ctx_ptr->caller_userdata, ac_buf, timeout_ms);
    }

    return result;
}

static wiced_result_t output_audio_client_buf_release_callback(audio_client_ref handle, void* userdata, audio_client_buf_t* ac_buf)
{
    wiced_result_t            result         = WICED_ERROR;
    wiced_audio_output_ctx_t *output_ctx_ptr = (wiced_audio_output_ctx_t *)userdata;

    UNUSED_PARAMETER(handle);

    if ( output_ctx_ptr->caller_buf_release_cbf != NULL )
    {
        result = output_ctx_ptr->caller_buf_release_cbf(NULL, output_ctx_ptr->caller_userdata, ac_buf);
    }

    return result;
}

static void output_wiced_audio_set_next_uri(const char *uri)
{
    Log_info("wiced_audio", "Next URI is '%s'", uri);
    free(output_ctx.next_uri);
    output_ctx.next_uri = ( (uri != NULL) && (uri[0] != '\0') ) ? strdup(uri) : NULL;
}

static void output_wiced_audio_set_uri(const char *uri, output_update_meta_cb_t meta_cb)
{
    Log_info("wiced_audio", "Current URI is '%s'", uri);
    free(output_ctx.uri);
    output_ctx.uri = ( (uri != NULL) && (uri[0] != '\0') ) ? strdup(uri) : NULL;
    output_ctx.meta_update_callback = meta_cb;
    SongMetaData_clear(&output_ctx.song_meta);
}

static int output_wiced_audio_play(output_transition_cb_t callback)
{
    int                  rc     = 0;
    wiced_result_t       result;
    AUDIO_CLIENT_IOCTL_T cmd;
    void                 *arg;

    Log_info("wiced_audio", ">> PLAY");

    output_ctx.play_trans_callback = callback;

    if (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED)
    {
        cmd = AUDIO_CLIENT_IOCTL_RESUME;
        arg = NULL;

        if ( output_ctx.gm_cbf != NULL )
        {
            output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_RESUME, NULL);
        }
    }
    else
    {
        if ( output_ctx.playback_status != AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED )
        {
            result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_STOP, NULL);
            if (result != WICED_SUCCESS)
            {
                Log_error("wiced_audio", "AUDIO_CLIENT_IOCTL_STOP failed with %d !", result);
            }
        }

        cmd = AUDIO_CLIENT_IOCTL_PLAY;
        arg = output_ctx.uri;

        if ( output_ctx.gm_cbf != NULL )
        {
            output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_PLAY, NULL);
        }
    }

    result = audio_client_ioctl(output_ctx.audio_client_handle, cmd, arg);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "%s failed with %d !",
                  ((cmd == AUDIO_CLIENT_IOCTL_PLAY) ? "AUDIO_CLIENT_IOCTL_PLAY" : "AUDIO_CLIENT_IOCTL_PAUSE"),
                  (int)result);
    }

    return rc;
}

static int output_wiced_audio_stop(void)
{
    int            rc     = 0;
    wiced_result_t result;

    Log_info("wiced_audio", ">> STOP");

    if ( output_ctx.gm_cbf != NULL )
    {
        output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_STOP, NULL);
    }

    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_STOP, NULL);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "AUDIO_CLIENT_IOCTL_STOP failed with %d !", result);
        rc = -1;
    }

    return rc;
}

static int output_wiced_audio_pause(void)
{
    int            rc     = 0;
    wiced_result_t result;

    Log_info("wiced_audio", ">> PAUSE");

    if ( output_ctx.gm_cbf != NULL )
    {
        output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_PAUSE, NULL);
    }

    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_PAUSE, NULL);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "AUDIO_CLIENT_IOCTL_PAUSE failed with %d !", result);
        rc = -1;
    }

    return rc;
}

static int output_wiced_audio_seek(int64_t position_nanos)
{
    int rc = 0;
    wiced_result_t result;
    uint32_t position_ms = 0;

    position_ms = position_nanos / NSECS_PER_MSEC;

    /* FIXME: using the current toolchain, printf/snprintf can't deal with 64-bit integers !! */
    Log_error( "wiced_audio", "SEEK to %ld msecs", (int32_t) ( (int64_t) ( position_nanos / (int64_t) NSECS_PER_MSEC ) ) );

    result = audio_client_ioctl( output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_SEEK, (void*) position_ms );

    if ( result != WICED_SUCCESS )
    {
        Log_error( "wiced_audio", "AUDIO_CLIENT_IOCTL_SEEK failed with %d !", result );
        rc = -1;
    }
    else if ( output_ctx.gm_cbf != NULL )
    {
        upnpavrender_event_data_t event_data;

        event_data.seek_position_msecs = (uint32_t) ( position_ms );
        output_ctx.gm_cbf( output_ctx.gm_user_context, UPNPAVRENDER_EVENT_SEEK, &event_data );
    }

    return rc;
}

static int output_wiced_audio_get_position(int64_t *track_duration, int64_t *track_pos)
{
    int rc = 0;

    *track_duration = output_ctx.last_known_time.duration;
    *track_pos      = output_ctx.last_known_time.position;

    if ( (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_STARTED) || (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED) )
    {
        wiced_result_t            result;
        audio_client_track_info_t info;

        result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_TRACK_INFO, &info);
        if (result != WICED_SUCCESS)
        {
            Log_error("wiced_audio", "AUDIO_CLIENT_IOCTL_TRACK_INFO failed with %d !", result);
        }
        else
        {
            if ( info.total_samples != 0 && info.sample_rate != 0 )
            {
                *track_duration = (int64_t)((double)((NSECS_PER_SEC_FLOAT / (double)info.sample_rate) * (double)info.total_samples));
            }
            else if ( info.total_samples == 0 && info.bitrate != 0 )
            {
                *track_duration = (int64_t)((double)(((output_ctx.audio_client_handle->http_params.http_total_content_length ) / (double)( info.bitrate / 8 )) * NSECS_PER_SEC_FLOAT));
            }
            else
            {
#ifdef DEBUG_POSITION_DURATION
                Log_error("wiced_audio", "Got 0 (duration) !");
#endif
                *track_duration = 0;
            }

            if ( info.current_sample != 0 && info.sample_rate != 0 )
            {
                *track_pos      = (int64_t)((double)((NSECS_PER_SEC_FLOAT / (double)info.sample_rate) * (double)info.current_sample));
            }
            else
            {
#ifdef DEBUG_POSITION_DURATION
                Log_error("wiced_audio", "Got 0 (position) !");
#endif
                *track_pos      = 0;
            }
        }
    }

    output_ctx.last_known_time.duration = *track_duration;
    output_ctx.last_known_time.position = *track_pos;

    if ( output_ctx.gm_cbf != NULL )
    {
        upnpavrender_event_data_t event_data;

        event_data.playback.position_msecs = (uint32_t)(output_ctx.last_known_time.position / NSECS_PER_MSEC);
        event_data.playback.duration_msecs = (uint32_t)(output_ctx.last_known_time.duration / NSECS_PER_MSEC);
        output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_PLAYBACK_PROGRESS, &event_data);
    }

    return rc;
}

static int output_wiced_audio_get_volume(float *v)
{
    int rc = 0;

    *v = exp(output_ctx.volume_in_db / 20 * log(10));
    Log_info("wiced_audio", "Queried volume fraction: %f", *v);

    return rc;
}

static int output_wiced_audio_set_volume(float value, int level)
{
    int            rc     = 0;
    wiced_result_t result;

    if ( output_ctx.gm_cbf != NULL )
    {
        upnpavrender_event_data_t event_data;

        event_data.volume = (uint8_t)level;
        output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_VOLUME, &event_data);
    }

    output_ctx.volume_in_db = 20 * log(value) / log(10);
    output_ctx.audio_client_params.volume = level;
    Log_info("wiced_audio", "Set volume fraction/dB to %f/%f/%d", value, output_ctx.volume_in_db, output_ctx.audio_client_params.volume);
    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_SET_VOLUME, (void *)output_ctx.audio_client_params.volume);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "AUDIO_CLIENT_IOCTL_SET_VOLUME failed with %d !", result);
    }

    return rc;
}

static int output_wiced_audio_get_mute(int *m)
{
    int rc = 0;

    *m = output_ctx.is_muted;

    return rc;
}

static int output_wiced_audio_set_mute(int m)
{
    int rc = 0;

    if ( output_ctx.gm_cbf != NULL )
    {
        upnpavrender_event_data_t event_data;

        event_data.mute_enabled = ((m != 0) ? WICED_TRUE : WICED_FALSE);
        output_ctx.gm_cbf(output_ctx.gm_user_context, UPNPAVRENDER_EVENT_MUTE, &event_data);
    }

    Log_info("wiced_audio", "Set mute to %s", m ? "on" : "off");
    output_ctx.is_muted = ((m != 0) ? 1 : 0);
    if ( output_ctx.is_muted != 0 )
    {
        audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_SET_VOLUME, (void *)0);
    }
    else
    {
        audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_SET_VOLUME, (void *)output_ctx.audio_client_params.volume);
    }

    return rc;
}

static int output_wiced_audio_add_options(void *ctx)
{
    upnpavrender_service_params_t* params = (upnpavrender_service_params_t *)ctx;
    /*
     *  output_wiced_audio_add_options() is going to be called _before_ output_wiced_audio_init;
     *  therefore, we have to initialize the output context struct here
     */
    memset(&output_ctx, 0, sizeof(output_ctx));

    memcpy(&output_ctx.audio_client_params, &params->audio_client_params, sizeof(output_ctx.audio_client_params));
    output_ctx.gm_cbf          = params->event_cb;
    output_ctx.gm_user_context = params->user_context;

    if ( output_ctx.audio_client_params.data_buffer_num == 0 )
    {
        output_ctx.audio_client_params.data_buffer_num = AUDIO_PLAYER_NUM_HTTP_BUFFERS;
    }

    if ( output_ctx.audio_client_params.audio_buffer_num == 0 )
    {
        output_ctx.audio_client_params.audio_buffer_num = AUDIO_PLAYER_NUM_AUDIO_BUFFERS;
    }

    if ( output_ctx.audio_client_params.audio_buffer_size == 0 )
    {
        output_ctx.audio_client_params.audio_buffer_size = AUDIO_PLAYER_SIZE_AUDIO_BUFFERS;
    }

    /* preserve caller original callback function and user data */
    output_ctx.caller_userdata                    = output_ctx.audio_client_params.userdata;
    output_ctx.caller_event_cbf                   = output_ctx.audio_client_params.event_cb;
    output_ctx.caller_buf_get_cbf                 = output_ctx.audio_client_params.buffer_get;
    output_ctx.caller_buf_release_cbf             = output_ctx.audio_client_params.buffer_release;
    output_ctx.audio_client_params.userdata       = &output_ctx;
    output_ctx.audio_client_params.event_cb       = output_audio_client_event_callback;
    output_ctx.audio_client_params.buffer_get     = output_audio_client_buf_get_callback;
    output_ctx.audio_client_params.buffer_release = output_audio_client_buf_release_callback;

    return 0;
}

static void* output_wiced_audio_get_audio_handle(void)
{
    return (void *)output_ctx.audio_client_handle;
}

static double volume_level_to_decibel(int volume)
{
    if (volume < UPNP_CONTROL_VOL_MIN)
    {
        volume = UPNP_CONTROL_VOL_MIN_DB;
    }

    if (volume > UPNP_CONTROL_VOL_MAX)
    {
        volume = UPNP_CONTROL_VOL_MAX_DB;
    }

    if (volume < UPNP_CONTROL_VOL_MAX / 2)
    {
        return UPNP_CONTROL_VOL_MIN_DB + (UPNP_CONTROL_VOL_MID_DB - UPNP_CONTROL_VOL_MIN_DB) / UPNP_CONTROL_VOL_MID * volume;
    }
    else
    {
        return UPNP_CONTROL_VOL_MID_DB + ((UPNP_CONTROL_VOL_MAX_DB - UPNP_CONTROL_VOL_MID_DB) / (UPNP_CONTROL_VOL_MAX - UPNP_CONTROL_VOL_MID) * (volume - UPNP_CONTROL_VOL_MID));
    }
}

static int output_wiced_audio_init(void)
{
    int rc = 0;

    SongMetaData_init(&output_ctx.song_meta);

    /* register supported MIME types */
    /* For .mp3 files*/
    register_mime_type("audio/mpeg");
    register_mime_type("audio/mp3");
    register_mime_type("audio/x-mp3");
    register_mime_type("audio/mpeg3");

    /* For .flac files*/
    register_mime_type("audio/flac");

    /* For .wav files */
    register_mime_type("audio/x-wav");
    register_mime_type("audio/wav");
    register_mime_type("audio/L8");
    register_mime_type("audio/L16");
    register_mime_type("audio/L20");
    register_mime_type("audio/L24");

    /* For MP4/Quicktime audio files .m4a */
    register_mime_type("audio/mp4");
    register_mime_type("audio/m4a");
    register_mime_type("audio/x-m4a");

    output_ctx.playback_status     = AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED;
    output_ctx.audio_client_handle = audio_client_init(&output_ctx.audio_client_params);
    if (output_ctx.audio_client_handle == NULL)
    {
        Log_error("wiced_audio", "audio_client_init() failed !");
        rc = -1;
        goto _exit;
    }

    output_wiced_audio_set_mute(0);
    output_ctx.volume_in_db = volume_level_to_decibel(output_ctx.audio_client_params.volume);
    output_wiced_audio_set_volume(exp(output_ctx.volume_in_db / 20 * log(10)), output_ctx.audio_client_params.volume);

 _exit:
    return rc;
}

static int output_wiced_audio_deinit(void)
{
    int rc = 0;

    free(output_ctx.uri);
    output_ctx.uri = NULL;
    free(output_ctx.next_uri);
    output_ctx.next_uri = NULL;
    audio_client_deinit(output_ctx.audio_client_handle);
    memset(&output_ctx, 0, sizeof(output_ctx));
    output_ctx.playback_status = AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED;

    return rc;
}

struct output_module wiced_audio_output =
{
    .shortname    = "wiced_audio",
	.description  = "WICED audio framework",
	.init         = output_wiced_audio_init,
	.deinit       = output_wiced_audio_deinit,
	.add_options  = output_wiced_audio_add_options,
	.set_uri      = output_wiced_audio_set_uri,
	.set_next_uri = output_wiced_audio_set_next_uri,
	.play         = output_wiced_audio_play,
	.stop         = output_wiced_audio_stop,
	.pause        = output_wiced_audio_pause,
	.seek         = output_wiced_audio_seek,
	.get_position = output_wiced_audio_get_position,
	.get_volume   = output_wiced_audio_get_volume,
	.set_volume   = output_wiced_audio_set_volume,
	.get_mute     = output_wiced_audio_get_mute,
	.set_mute     = output_wiced_audio_set_mute,
	.get_audio_handle = output_wiced_audio_get_audio_handle,
};
