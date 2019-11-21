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
 * Audio client library
 *
 */

#include "wiced_result.h"
#include "wiced_platform.h"
#include "platform_audio.h"
#include "wiced_log.h"

#include "audio_client.h"
#include "audio_client_private.h"
#include "audio_client_flac.h"
#include "audio_client_wav.h"
#include "audio_client_aac.h"
#include "audio_client_mp3.h"
#include "audio_client_ts.h"
#include "audio_client_hls.h"
#include "audio_client_utils.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/*
 * Audio render uses a small period buffer size by default
 * for best low latency performance. HTTP streaming can use
 * a larger size.
 */

#define AUDIO_CLIENT_PERIOD_SIZE            (512)

#define AUDIO_CLIENT_START_SEEK_THRESHOLD   (3 * 1000)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    int buflen;
    int maxlen;
    objhandle_t buf_handle;
    uint8_t buf[0];
} audio_buf_t;

/******************************************************
 *                    Structures
 ******************************************************/

static lookup_t playlist_types[] =
{
    { ".pls",           AUDIO_CLIENT_PLAYLIST_PLS   },
    { ".m3u",           AUDIO_CLIENT_PLAYLIST_M3U   },
    { NULL,             0                           }       /* Must be last! */
};

lookup_t mime_types[] =
{
    { "audio/wav",      AUDIO_CLIENT_CODEC_WAV      },
    { "audio/x-wav",    AUDIO_CLIENT_CODEC_WAV      },
    { "audio/flac",     AUDIO_CLIENT_CODEC_FLAC     },
    { "video/mp4",      AUDIO_CLIENT_CODEC_AAC_M4A  },
    { "audio/m4a",      AUDIO_CLIENT_CODEC_AAC_M4A  },
    { "audio/x-m4a",    AUDIO_CLIENT_CODEC_AAC_M4A  },
    { "audio/aac",      AUDIO_CLIENT_CODEC_AAC_ADTS },
    { "audio/aacp",     AUDIO_CLIENT_CODEC_AAC_ADTS },
    { "audio/x-aac",    AUDIO_CLIENT_CODEC_AAC_ADTS },
    { "audio/mpeg",     AUDIO_CLIENT_CODEC_MP3      },
    { "audio/mp3",      AUDIO_CLIENT_CODEC_MP3      },
    { NULL,             0                           }       /* Must be last! */
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t audio_client_start_decoder(audio_client_t* client, AUDIO_CLIENT_CODEC_T codec, AUDIO_CLIENT_ERROR_T *err);
static wiced_result_t audio_client_stop_decoder(audio_client_t* client, AUDIO_CLIENT_CODEC_T codec);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t audio_client_null_decoder_start(audio_client_t* client)
{
    (void)client;
    return WICED_SUCCESS;
}


static wiced_result_t audio_client_null_decoder_stop(audio_client_t* client)
{
    (void)client;
    return WICED_SUCCESS;
}


static wiced_result_t audio_client_null_decoder_ioctl(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg)
{
    (void)client;
    (void)ioctl;
    (void)arg;

    return WICED_UNSUPPORTED;
}


static wiced_result_t audio_client_null_filter_create(audio_client_t* client)
{
    (void)client;
    return WICED_SUCCESS;
}


static wiced_result_t audio_client_null_filter_destroy(audio_client_t* client)
{
    (void)client;
    return WICED_SUCCESS;
}


static wiced_result_t audio_client_null_filter_ioctl(audio_client_t* client, FILTER_IOCTL_T ioctl, void* arg)
{
    (void)client;
    (void)ioctl;
    (void)arg;

    return WICED_UNSUPPORTED;
}


static wiced_result_t audio_render_buffer_release(audio_render_buf_t* buf, void* userdata)
{
    audio_client_t* client = (audio_client_t*)userdata;

    /*
     * Ignore the tag valid/invalid flag since the flag gets set to invalid when we are shutting down.
     */

    if (client == NULL || buf == NULL)
    {
        return WICED_ERROR;
    }

    if (client->buf_pool && buf->opaque)
    {
        bufmgr_pool_free_buf(client->buf_pool, (objhandle_t)buf->opaque);
        buf->opaque = NULL;
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_render_event(audio_render_ref handle, void* userdata, AUDIO_RENDER_EVENT_T event, void* arg)
{
    audio_client_t* client = (audio_client_t*)userdata;

    if (client == NULL || client->tag != AUDIO_CLIENT_TAG_VALID)
    {
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Audio client: audio render event received: %d\n", event);

    switch (event)
    {
        case AUDIO_RENDER_EVENT_EOS:
            wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_PLAYBACK_COMPLETE);
            break;

        case AUDIO_RENDER_EVENT_CONFIG_ERROR:
            client->audio_configured  = WICED_FALSE;
            wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_STOP);
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized audio render event: %d\n", event);
            break;
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_effect(audio_client_t* client)
{
    wiced_result_t result;

    if (client->state != AUDIO_CLIENT_STATE_IDLE && client->audio)
    {
        /*
         * Tell the audio render to set effect mode.
         */

        result = audio_render_command(client->audio, AUDIO_RENDER_CMD_SET_EFFECT, (void*)client->effect_mode);
        if (result == WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio effect mode: %d\n", client->effect_mode);
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set audio effect mode [mode: %d][error: %d]\n", client->effect_mode, result);
        }
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_add_play_request(audio_client_t* client, audio_client_play_request_t* request)
{
    audio_client_play_request_t* ptr;
    wiced_bool_t bad_request;

    /*
     * Check for valid request.
     */

    if (!request || (request->stream_uri == NULL && request->push_audio_codec == AUDIO_CLIENT_CODEC_NULL))
    {
        return WICED_ERROR;
    }

    /*
     * Lock the play queue.
     */

    if (wiced_rtos_lock_mutex(&client->play_mutex) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock play request mutex\n");
        return WICED_ERROR;
    }

    /*
     * If this is a push buffer request then we need to make sure that we don't have any preceding
     * HTTP requests. We can't handle accepting buffer requests while HTTP streaming is active.
     */

    if (request->push_audio_codec != AUDIO_CLIENT_CODEC_NULL)
    {
        bad_request = WICED_FALSE;

        for (ptr = client->play_queue; ptr != NULL; ptr = ptr->next)
        {
            if (ptr->stream_uri != NULL)
            {
                bad_request = WICED_TRUE;
                break;
            }
        }

        if (client->current_play && client->current_play->stream_uri)
        {
            bad_request = WICED_TRUE;
        }

        if (bad_request)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to add push request\n");
            wiced_rtos_unlock_mutex(&client->play_mutex);
            return WICED_ERROR;
        }

        client->accept_push_buffer = WICED_TRUE;
    }

    /*
     * Clear to add this request to the queue.
     */

    if (client->play_queue == NULL)
    {
        client->play_queue = request;
    }
    else
    {
        for (ptr = client->play_queue; ptr->next != NULL; ptr = ptr->next)
        {
            /* Empty */
        }
        ptr->next = request;
    }
    request->next = NULL;

    wiced_rtos_unlock_mutex(&client->play_mutex);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Play request added to queue: %s\n", request->stream_uri ? request->stream_uri : "Push request");

    return WICED_SUCCESS;
}


wiced_result_t audio_client_audio_config(audio_client_t* client, wiced_audio_config_t* config)
{
    wiced_result_t result;

    if (client == NULL || client->tag != AUDIO_CLIENT_TAG_VALID || config == NULL)
    {
        return WICED_ERROR;
    }

    if (client->audio)
    {
        result = audio_render_configure(client->audio, config);
    }
    else
    {
        result = client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_AUDIO_FORMAT, config);
    }

    if (result == WICED_SUCCESS)
    {
        client->audio_configured = WICED_TRUE;

        if (client->audio && (client->effect_mode != AUDIO_CLIENT_EFFECT_MODE_NONE))
        {
            audio_client_effect(client);
        }
    }
    else
    {
        client->audio_configured = WICED_FALSE;
    }

    return result;
}


wiced_result_t audio_client_buffer_get(audio_client_t* client, audio_client_buf_t* buf, uint32_t timeout_ms)
{
    objhandle_t buf_handle;
    audio_buf_t* audio_buf;
    uint32_t buf_size;
    wiced_result_t result = WICED_SUCCESS;

    if (client == NULL || client->tag != AUDIO_CLIENT_TAG_VALID)
    {
        return WICED_ERROR;
    }

    if (client->audio)
    {
        if (bufmgr_pool_alloc_buf(client->buf_pool, &buf_handle, (char **)&audio_buf, &buf_size, timeout_ms) != BUFMGR_OK)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Unable to allocate audio packet\n");
            return WICED_ERROR;
        }

        buf->opaque = buf_handle;
        buf->buf    = audio_buf->buf;
        buf->buflen = client->params.audio_buffer_size;
        buf->curlen = 0;
        buf->offset = 0;
        buf->flags  = 0;
    }
    else
    {
        result = client->params.buffer_get(client, client->params.userdata, buf, AUDIO_CLIENT_AUDIO_BUF_TIMEOUT);
    }

    return result;
}


wiced_result_t audio_client_buffer_release(audio_client_t* client, audio_client_buf_t* buf)
{
    audio_render_buf_t render_buf;
    wiced_result_t result = WICED_SUCCESS;

    if (client == NULL)
    {
        return WICED_ERROR;
    }

    if (client->audio)
    {
        /*
         * Pass this buffer on to audio render.
         */

        if (buf->buf && buf->curlen > 0)
        {
            render_buf.pts         = 0;
            render_buf.data_buf    = buf->buf;
            render_buf.data_offset = buf->offset;
            render_buf.data_length = buf->curlen;
            render_buf.opaque      = buf->opaque;

            result = audio_render_push(client->audio, &render_buf);

            if (result != WICED_SUCCESS)
            {
                /*
                 * Audio render didn't accept the buffer, make sure that we don't leak it.
                 */

                if (client->buf_pool && buf->opaque)
                {
                    bufmgr_pool_free_buf(client->buf_pool, (objhandle_t)buf->opaque);
                    buf->opaque = NULL;
                }
            }
            else
            {
                client->audio_pushed = WICED_TRUE;
            }
        }
        else if (client->buf_pool && buf->opaque)
        {
            /*
             * The decoder is releasing this buffer without writing any data to it.
             */

            bufmgr_pool_free_buf(client->buf_pool, (objhandle_t)buf->opaque);
            buf->opaque = NULL;
        }

        if (buf->flags & AUDIO_CLIENT_BUF_FLAG_EOS)
        {
            /*
             * Tell audio render that there's no more audio coming for this stream.
             */

            result = audio_render_push(client->audio, NULL);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_render_push(NULL) failed !\n");
            }
        }
    }
    else
    {
        result = client->params.buffer_release(client, client->params.userdata, buf);
    }

    return result;
}


wiced_bool_t audio_client_check_start_offset_seek(audio_client_t* client)
{
    wiced_result_t result;
    uint32_t arg;

    /*
     * If the server doesn't support seeking or the time is less than
     * 3 ms than no seeking.
     */

    if (client->http_params.http_range_requests == WICED_FALSE || client->http_params.http_total_content_length == 0 || client->start_offset_ms < AUDIO_CLIENT_START_SEEK_THRESHOLD)
    {
        return WICED_FALSE;
    }

    /*
     * Ask the decoder where in the stream we need to seek.
     */

    arg    = client->start_offset_ms;
    result = client->decoders[client->audio_codec].decoder_ioctl(client, DECODER_IOCTL_GET_SEEK_POSITION, &arg);

    /*
     * If the decoder is happy than we are good to seek.
     */

    if (result == WICED_SUCCESS)
    {
        client->seek_position = arg;
        wiced_rtos_set_event_flags(&client->http_params.http_events, HTTP_EVENT_SEEK);
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Initiating start offset seek %lu\n", client->start_offset_ms);
    }

    return (result == WICED_SUCCESS ? WICED_TRUE : WICED_FALSE);
}


static wiced_result_t audio_client_play_raw(audio_client_t* client, audio_client_push_buf_t* push_buf)
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t push_bytes;
    uint32_t offset = 0;
    audio_client_buf_t audio_buf;
    audio_client_buf_t* audio_buf_ptr = NULL;
    uint16_t* src;
    uint16_t* dst;
    uint32_t i;

    if ((client == NULL) || (push_buf == NULL))
    {
        return WICED_BADARG;
    }

    if (client->audio_configured != WICED_TRUE)
    {
        return WICED_ERROR;
    }

    if (!client->audio_pushed)
    {
        client->state        = AUDIO_CLIENT_STATE_PLAY;
        client->audio_pushed = WICED_TRUE;

        /*
         * Tell the app that we've statred playback.
         */

        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_STARTED, NULL);
    }

    if (push_buf->buflen == 0)
    {
        /*
         * Tell the audio render that no more data is coming for this stream.
         */

        memset(&audio_buf, 0, sizeof(audio_client_buf_t));
        audio_buf.flags = AUDIO_CLIENT_BUF_FLAG_EOS;
        audio_client_buffer_release(client, &audio_buf);

        return WICED_SUCCESS;
    }

    push_bytes = push_buf->buflen;
    if (client->raw_mono_to_stereo)
    {
        push_bytes *= 2;
    }

    while (push_bytes > 0)
    {
        result = audio_client_buffer_get(client, &audio_buf, AUDIO_CLIENT_AUDIO_BUF_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            /* we probably didn't get a buffer because the buffers are full */
            continue;
        }

        /* keep a reference to audio_buf */
        audio_buf_ptr = &audio_buf;

        audio_buf.offset = 0;

        if (push_bytes < audio_buf.buflen)
        {
            audio_buf.curlen = push_bytes;
        }
        else
        {
            audio_buf.curlen = audio_buf.buflen;
        }

        if (client->raw_mono_to_stereo)
        {
            audio_buf.curlen = audio_buf.curlen & ~(0x3);
            src = (uint16_t*)&push_buf->buf[offset];
            dst = (uint16_t*)audio_buf.buf;

            for (i = 0; i < audio_buf.curlen; i += 4)
            {
                *dst++ = *src;
                *dst++ = *src++;
            }
            push_bytes -= audio_buf.curlen;
            offset     += (audio_buf.curlen / 2);
        }
        else
        {
            memcpy(audio_buf.buf, &push_buf->buf[offset], audio_buf.curlen);

            push_bytes -= audio_buf.curlen;
            offset     += audio_buf.curlen;
        }

        while (1)
        {
            result = audio_client_buffer_release(client, &audio_buf);
            if (result == WICED_SUCCESS)
            {
                audio_buf_ptr = NULL;
                break;
            }
            if (result == WICED_BADARG)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Error pushing audio buffer to the audio_render\n", __func__);
                goto _bail;
            }
        }
    }

  _bail:
    /* we still have a reference to audio_buf; we need to release it */
    if (audio_buf_ptr != NULL)
    {
        audio_client_buffer_release(client, audio_buf_ptr);
    }

    return result;
}


/*
 * This routine starts the decoder pipeline for push buffer decoding requests.
 * client->current_play should already be initialized with the item to be played.
 */

static wiced_result_t audio_client_start_push_decoding(audio_client_t* client, wiced_bool_t early_start, AUDIO_CLIENT_ERROR_T* err)
{
    uint32_t initial_buffer_count;
    wiced_result_t result;

    if (client->current_play->push_audio_codec == AUDIO_CLIENT_CODEC_NULL)
    {
        return WICED_BADARG;
    }

    client->http_params.stream_uri     = NULL;
    client->start_offset_ms            = client->current_play->start_offset_ms;
    client->http_params.http_load_file = WICED_FALSE;

    *err = AUDIO_CLIENT_ERROR_SUCCESS;

    /*
     * Make sure the audio configured flag is reset before starting a new stream.
     */

    client->audio_configured = WICED_FALSE;
    client->audio_pushed     = WICED_FALSE;

    /*
     * Let user know what the incoming codec is.
     */

    client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_AUDIO_CODEC, (void*)client->current_play->push_audio_codec);

    result = audio_client_start_decoder(client, client->current_play->push_audio_codec, err);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /*
     * Starting a push buffer request. If we're dealing with an early start, preroll disabled,
     * or there are already buffers waiting than we want to disable the preroll processing.
     */

    if (early_start || client->params.no_length_disable_preroll || client->data_buf_ridx != client->data_buf_widx)
    {
        initial_buffer_count = client->params.data_buffer_preroll + 1;
    }
    else
    {
        initial_buffer_count = 0;
    }
    client->http_params.initial_buffer_count = initial_buffer_count;
    client->threshold_high_sent              = WICED_FALSE;
    client->threshold_low_sent               = WICED_FALSE;
    client->seek_in_progress                 = WICED_FALSE;
    client->decoder_first_kick               = WICED_TRUE;
    client->http_params.http_range_requests  = WICED_FALSE;

    /*
     * If there is data already waiting to be decoded than kick the decoder. If we're playing
     * back to back push buffer requests than the second clip may be entirely written to our
     * buffers before we start decoding.
     */

    if (client->data_buf_ridx != client->data_buf_widx)
    {
        wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_STARTED, NULL);
        client->decoder_first_kick = WICED_FALSE;
    }

    return WICED_SUCCESS;
}


static void audio_client_complete_playback_housekeeping(audio_client_t* client, audio_client_http_params_t* params)
{
    audio_client_play_request_t* ptr;

    /*
     * At this point playback has finished and we are going back to the idle state.
     * We don't send any events since we have no idea what triggered the playback
     * complete (error, stop, normal EOS, decoder end when not playing). This is the
     * common cleanup/reset functionality for all cases.
     */

    if (client->post_decode != NULL)
    {
        /*
         * Post decode list has something on it. This means that we've already started playback
         * of another item. Don't go to idle, just remove an entry from the post decode list
         * and carry on.
         */

        ptr = client->post_decode;
        client->post_decode = client->post_decode->next;

        free(ptr);
    }
    else
    {
        client->state      = AUDIO_CLIENT_STATE_IDLE;
        params->stream_uri = NULL;
        if (client->current_play != NULL)
        {
            free(client->current_play);
            client->current_play = NULL;
        }

        /*
         * Do we have another play request to try and process?
         */

        if (client->play_queue != NULL)
        {
            wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_PLAY);
        }
        else
        {
            client->accept_push_buffer = WICED_FALSE;
            client->accept_raw_buffer  = WICED_FALSE;
        }
    }
}


static wiced_result_t audio_client_common_start_playback(audio_client_t* client, audio_client_http_params_t* params, audio_client_play_request_t* request, wiced_bool_t early_start)
{
    AUDIO_CLIENT_CODEC_T codec;
    AUDIO_CLIENT_ERROR_T err;
    wiced_result_t result;
    int extlen;
    int len;
    int i;

    client->current_play = request;
    request->next        = NULL;

    params->stream_uri         = request->stream_uri;
    client->start_offset_ms    = request->start_offset_ms;
    params->http_load_file     = request->load_file;
    params->stream_uri_storage = STREAM_URI_STORAGE_TYPE_PLAY_REQUEST;

    err = AUDIO_CLIENT_ERROR_SUCCESS;

    /*
     * Make sure the audio configured flag is reset before starting a new stream.
     */

    client->audio_configured = WICED_FALSE;
    client->audio_pushed     = WICED_FALSE;

    if (client->current_play->push_audio_codec != AUDIO_CLIENT_CODEC_NULL)
    {
        audio_client_start_push_decoding(client, early_start, &err);
    }
    else
    {
        codec = AUDIO_CLIENT_CODEC_NULL;
        if (client->current_play->load_file == WICED_FALSE)
        {
            /*
             * Is this a format we like & understand?
             */

            codec = audio_client_check_uri_for_audio_codec(params->stream_uri);
            if (codec == AUDIO_CLIENT_CODEC_NULL)
            {
                if (client->params.load_playlist_files)
                {
                    /*
                     * Check for a playlist extension here. If so, ask the app if they want
                     * to convert it to a load file operation.
                     */

                    len = strlen(params->stream_uri);
                    for (i = 0; playlist_types[i].name != NULL; i++)
                    {
                        extlen = strlen(playlist_types[i].name);
                        if (strncasecmp(&params->stream_uri[len - extlen], playlist_types[i].name, extlen) == 0)
                        {
                            break;
                        }
                    }

                    if (playlist_types[i].name != NULL)
                    {
                        result = client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYLIST_EXTENSION, (void*)playlist_types[i].value);
                        if (result == WICED_SUCCESS)
                        {
                            params->http_load_file = WICED_TRUE;
                        }
                    }
                }

                if (!params->http_load_file)
                {
                    /*
                     * We don't recognize the extension. Don't error out here, we'll
                     * try and pick out the audio mime type from the HTTP response.
                     * The NULL decoder will be a no op for the decoder start.
                     */

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "No extension match. Check mime type in HTTP response\n");
                }
            }
        }

        if (codec != AUDIO_CLIENT_CODEC_NULL)
        {
            /*
             * Found a codec. Let user know what the incoming codec is.
             */

            client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_AUDIO_CODEC, (void*)codec);

            /*
             * And start the decoder.
             */

            result = audio_client_start_decoder(client, codec, &err);
            if (result != WICED_SUCCESS)
            {
                goto _bail;
            }
        }

        /*
         * Now spawn off the HTTP reader thread.
         */

        if (audio_client_http_reader_start(client, params) != WICED_SUCCESS)
        {
            if (codec != AUDIO_CLIENT_CODEC_NULL)
            {
                audio_client_stop_decoder(client, codec);
            }
            err = AUDIO_CLIENT_ERROR_HTTP_INIT;
        }
    }

  _bail:
    if (err == AUDIO_CLIENT_ERROR_SUCCESS)
    {
        client->state = AUDIO_CLIENT_STATE_PLAY;

        result = WICED_SUCCESS;
    }
    else
    {
        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)err);
        client->current_play = NULL;

        result = WICED_ERROR;
    }

    return result;
}


static void audio_client_check_for_early_playback_start(audio_client_t* client, audio_client_http_params_t* params)
{
    audio_client_play_request_t* request;
    audio_client_play_request_t* ptr;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Checking for early playback start (play_queue: %p)\n", client->play_queue);

    if (client->play_queue == NULL || client->current_play == NULL)
    {
        return;
    }

    /*
     * Check the next play request to see if it qualifies.
     */

    if (wiced_rtos_lock_mutex(&client->play_mutex) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock play request mutex\n");
        return;
    }

    if (client->play_queue && (client->play_queue->push_audio_codec != AUDIO_CLIENT_CODEC_NULL || client->play_queue->contiguous))
    {
        /*
         * Looks good. Let's grab it.
         */

        request            = client->play_queue;
        client->play_queue = client->play_queue->next;

        /*
         * Add the current play onto the post decode list.
         */

        if (client->post_decode == NULL)
        {
            client->post_decode = client->current_play;
        }
        else
        {
            for (ptr = client->post_decode; ptr->next != NULL; ptr = ptr->next)
            {
                /* Empty */
            }
            ptr->next = client->current_play;
        }
        client->current_play = NULL;
        params->stream_uri   = NULL;
    }
    else
    {
        request = NULL;
    }
    wiced_rtos_unlock_mutex(&client->play_mutex);

    if (request == NULL)
    {
        return;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Beginning early playback start\n");

    if (audio_client_common_start_playback(client, params, request, WICED_TRUE) != WICED_SUCCESS)
    {
        free(request);
    }
}


static wiced_result_t audio_client_seek(audio_client_t* client)
{
    wiced_result_t result = WICED_UNSUPPORTED;
    uint32_t arg;

    /*
     * Ask the decoder where in the stream we need to seek.
     */

    arg = client->seek_request_ms;
    result = client->decoders[client->audio_codec].decoder_ioctl(client, DECODER_IOCTL_GET_SEEK_POSITION, &arg);

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Seek not supported for current codec\n");
        client->params.event_cb( client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*) AUDIO_CLIENT_ERROR_SEEK_ERROR );
        return WICED_SUCCESS;
    }

    client->seek_position = arg;
    wiced_rtos_set_event_flags(&client->http_params.http_events, HTTP_EVENT_SEEK);

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_pause(audio_client_t* client)
{
    if (client->state == AUDIO_CLIENT_STATE_PLAY)
    {
        /*
         * Tell the audio render to pause playback.
         */
        if (client->audio)
        {
            audio_render_command(client->audio, AUDIO_RENDER_CMD_PAUSE, NULL);
        }
        client->state = AUDIO_CLIENT_STATE_PAUSE;

        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED, NULL);
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_resume(audio_client_t* client)
{
    if (client->state == AUDIO_CLIENT_STATE_PAUSE)
    {
        /*
         * Tell the audio render to pause playback.
         */

        if (client->audio)
        {
            audio_render_command(client->audio, AUDIO_RENDER_CMD_RESUME, NULL);
        }
        client->state = AUDIO_CLIENT_STATE_PLAY;

        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_STARTED, NULL);
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_stop(audio_client_t* client, wiced_bool_t stop_all)
{
    audio_client_play_request_t* next;
    uint32_t events;

    client->stop_in_progress = WICED_TRUE;

    /*
     * Shutdown the HTTP reader thread.
     */

    audio_client_http_reader_stop(client, &client->http_params);

    /*
     * Shutdown the decoder thread.
     * Flush out any pending HTTP_HEADER_COMPLETE event as it may trigger an undesirable restart of
     * that decoder thread.
     */

    events = 0;
    wiced_rtos_wait_for_event_flags(&client->events, AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
    audio_client_stop_decoder(client, client->audio_codec);

    /*
     * And stop the audio render playback.
     */

    if (client->audio)
    {
        audio_render_command(client->audio, AUDIO_RENDER_CMD_STOP, NULL);
    }

    /*
     * Shutdown HLS processing
     */

    audio_client_hls_flush_list(client);

    /*
     * Make sure that we clear out our buffers.
     */

    client->data_buf_ridx = 0;
    client->data_buf_widx = 0;
    memset(client->data_bufs, 0, client->params.data_buffer_num * sizeof(data_buf_t));

    client->stop_in_progress = WICED_FALSE;

    /*
     * Do we need to tell the app?
     */

    if (client->state == AUDIO_CLIENT_STATE_PLAY || client->state == AUDIO_CLIENT_STATE_PAUSE)
    {
        if (!client->http_params.http_load_file)
        {
            /*
             * Since we've just told the decoder to shut down, eat any DECODER_THREAD_DONE event.
             * Otherwise there's a window where a new play command may come in before the main loop
             * processes the DECODER_THREAD_DONE event.
             */

            events = 0;
            wiced_rtos_wait_for_event_flags(&client->events, AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);

            client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED, NULL);
        }
    }

    audio_client_complete_playback_housekeeping(client, &client->http_params);
    if (stop_all)
    {
        client->accept_push_buffer = WICED_FALSE;
        client->accept_raw_buffer  = WICED_FALSE;
        if (wiced_rtos_lock_mutex(&client->play_mutex) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock play request mutex\n");
            return WICED_ERROR;
        }

        while (client->play_queue != NULL)
        {
            next = client->play_queue->next;
            free(client->play_queue);
            client->play_queue = next;
        }

        while (client->post_decode != NULL)
        {
            next = client->post_decode->next;
            free(client->post_decode);
            client->post_decode = next;
        }

        wiced_rtos_unlock_mutex(&client->play_mutex);
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_start_decoder(audio_client_t* client, AUDIO_CLIENT_CODEC_T codec, AUDIO_CLIENT_ERROR_T *err)
{
    if (client->audio_codec != AUDIO_CLIENT_CODEC_NULL && client->decoder_handle)
    {
        /*
         * Under normal circumstances this shouldn't happen. But if we got here it did.
         * Log the situation and try our best to recover.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Decoder already active - codec: %d, requested codec: %d (%p)\n",
                      client->audio_codec, codec, client->decoder_handle);
        if (client->audio_codec == codec)
        {
            return WICED_SUCCESS;
        }

        /*
         * Stop the existing decoder.
         */

        if (client->decoders[client->audio_codec].decoder_stop(client) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to stop decoder\n");
            return WICED_ERROR;
        }

        client->audio_codec = AUDIO_CLIENT_CODEC_NULL;
    }

    /*
     * Make sure that we have a decoder configured.
     */

    if (client->decoders[codec].decoder_start == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No decoder configured for %d\n", codec);
        *err = AUDIO_CLIENT_ERROR_BAD_CODEC;
        return WICED_ERROR;
    }

    /*
     * And fire off the decoder.
     */

    if (client->decoders[codec].decoder_start(client) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize decoder\n");
        *err = AUDIO_CLIENT_ERROR_DECODER_ERROR;
        return WICED_ERROR;
    }

    /*
     * Save the active codec.
     */

    client->audio_codec = codec;

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_stop_decoder(audio_client_t* client, AUDIO_CLIENT_CODEC_T codec)
{
    if (client->decoders[codec].decoder_stop == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No decoder configured for %d\n", codec);
        return WICED_ERROR;
    }

    if (client->decoders[codec].decoder_stop(client) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to stop decoder !\n");
        return WICED_ERROR;
    }

    client->audio_codec = AUDIO_CLIENT_CODEC_NULL;

    return WICED_SUCCESS;
}


static void audio_client_start_playback(audio_client_t* client)
{
    audio_client_play_request_t* request;

    /*
     * Are we currently playing? If so don't do anything.
     */

    if (client->state != AUDIO_CLIENT_STATE_IDLE || client->current_play)
    {
        return;
    }

    /*
     * Get the next play request.
     */

    if (wiced_rtos_lock_mutex(&client->play_mutex) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to lock play request mutex\n");
        return;
    }

    request = client->play_queue;
    if (client->play_queue != NULL)
    {
        client->play_queue = client->play_queue->next;
    }
    wiced_rtos_unlock_mutex(&client->play_mutex);

    if (request == NULL)
    {
        /*
         * No request waiting...bail.
         */

        return;
    }

    if (audio_client_common_start_playback(client, &client->http_params, request, WICED_FALSE) != WICED_SUCCESS)
    {
        free(request);

        /*
         * Do we have another play request to try and process?
         */

        if (client->play_queue != NULL)
        {
            wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_PLAY);
        }
    }
}


static wiced_result_t audio_client_process_header_complete(audio_client_t* client)
{
    AUDIO_CLIENT_ERROR_T error = AUDIO_CLIENT_ERROR_SUCCESS;
    AUDIO_CLIENT_CODEC_T codec = AUDIO_CLIENT_CODEC_NULL;
    int i;

    if (client->audio_codec != AUDIO_CLIENT_CODEC_NULL || client->http_params.http_load_file)
    {
        /*
         * Nothing to see here...move along.
         */

        return WICED_SUCCESS;
    }

    /*
     * Check to see if it's a mime type we support.
     */

    for (i = 0; mime_types[i].name != NULL; i++)
    {
        if (strncasecmp(mime_types[i].name, client->http_params.http_content_type, strlen(mime_types[i].name)) == 0 &&
            strlen(mime_types[i].name) == strlen(client->http_params.http_content_type))
        {
            codec = mime_types[i].value;
            break;
        }
    }

    if (codec == AUDIO_CLIENT_CODEC_NULL)
    {
        /*
         * We didn't find a match. Do we have data to try probing the stream?
         */

        if (!client->data_bufs[client->data_buf_ridx].inuse)
        {
            /*
             * Nope, not yet. Let's ask to be triggered again when the first data buffer comes in.
             */

            client->http_params.http_need_extra_header_check = WICED_TRUE;
            return WICED_SUCCESS;
        }

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Probing for audio codec...\n");
        codec = audio_client_probe_for_audio_codec(client, client->http_params.stream_uri);

        if (codec == AUDIO_CLIENT_CODEC_NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No decoder configured for mime type: %s\n", client->http_params.http_content_type);
            error = AUDIO_CLIENT_ERROR_BAD_CODEC;
        }
    }

    if (codec != AUDIO_CLIENT_CODEC_NULL)
    {
        /*
         * Found a match. Let user know what the incoming codec is.
         */

        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_AUDIO_CODEC, (void*)codec);

        audio_client_start_decoder(client, codec, &error);
    }

    /*
     * If we encountered an error we need to shut down the pipeline.
     */

    if (error != AUDIO_CLIENT_ERROR_SUCCESS)
    {
        /*
         * Set the state to IDLE first so that the stop action doesn't send an event
         * to the app. We'll send the proper error event ourselves.
         */

        client->state = AUDIO_CLIENT_STATE_IDLE;
        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)error);
        audio_client_stop(client, WICED_FALSE);
    }

    return WICED_SUCCESS;
}


static wiced_result_t process_pushed_buffer(audio_client_t* client, audio_client_push_buf_t* push_buf)
{
    wiced_result_t result = WICED_SUCCESS;
    data_buf_t*    dbuf   = NULL;
    uint16_t       size   = 0;

    if (push_buf->buf == NULL || push_buf->buflen == 0)
    {
        /*
         * Assume that no more data are coming in; notify the decoder
         */

        while (!client->stop_in_progress)
        {
            dbuf = &client->data_bufs[client->data_buf_widx];

            if (dbuf->inuse == 0)
            {
                dbuf->buflen = 0;
                dbuf->inuse  = 1;
                client->data_buf_widx = (client->data_buf_widx + 1) % client->params.data_buffer_num;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Push empty buffer to decoder\n");
                wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
                break;
            }

            wiced_rtos_delay_milliseconds(1);
        }
    }
    else
    {
        while (push_buf->pushedlen < push_buf->buflen)
        {
            dbuf = &client->data_bufs[client->data_buf_widx];

            if (dbuf->inuse)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Receive buffers full (%d)\n", client->data_buf_widx);
                /* Let caller decide how to handle the wait */
                result = WICED_PENDING;
                break;
            }

            size = MIN(push_buf->buflen - push_buf->pushedlen, AUDIO_CLIENT_DATA_BUF_SIZE);
            write_data_buf(client, &client->http_params, dbuf, &push_buf->buf[push_buf->pushedlen], size);
            push_buf->pushedlen += size;
        }
    }

    return result;
}


static wiced_result_t audio_client_process_suspend(audio_client_t* client)
{
    audio_client_stream_info_t info;
    audio_client_suspend_t* suspend;
    wiced_result_t result = WICED_SUCCESS;
    uint32_t current_ms = 0;

    if (client->hls_playlist_active)
    {
        if (audio_client_hls_suspend(client, &suspend) != WICED_SUCCESS)
        {
            goto _exit;
        }
        else
        {
            goto _stop_exit;
        }
    }

    if (client->state == AUDIO_CLIENT_STATE_IDLE || client->http_params.stream_uri == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid state for suspend: %d %p\n", client->state, client->http_params.stream_uri);
        result = WICED_ERROR;
        goto _exit;
    }

    if ((suspend = calloc(1, sizeof(audio_client_suspend_t) + strlen(client->http_params.stream_uri) + 1)) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate suspend structure\n");
        result = WICED_OUT_OF_HEAP_SPACE;
        goto _exit;
    }

    /*
     * Cache the information we need to resume playback.
     */

    suspend->uri = (char*)suspend->data;
    strcpy(suspend->uri, client->http_params.stream_uri);
    suspend->live_stream = (client->http_params.http_total_content_length == 0 ? WICED_TRUE : WICED_FALSE);
    suspend->codec       = client->audio_codec;

    if (client->audio_codec != AUDIO_CLIENT_CODEC_NULL)
    {
        result = client->decoders[client->audio_codec].decoder_ioctl(client, DECODER_IOCTL_INFO, &info);
        if (result == WICED_SUCCESS)
        {
            current_ms = (uint32_t)(1000.0 / ((double)info.stream_sample_rate / (double)info.stream_current_sample));
        }
    }
    suspend->time_playing = current_ms;

  _stop_exit:

    /*
     * Stop the current playback.
     * Set the state to idle first so we don't send any notification to the app.
     */

    client->state = AUDIO_CLIENT_STATE_IDLE;
    audio_client_stop(client, WICED_TRUE);

    /*
     * And attach the suspend structure so that the other thread will grab the info.
     */

    client->suspend = suspend;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Audio client playing: %s\n", suspend->uri);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Stream is %slive\n", suspend->live_stream ? "" : "NOT ");
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Current codec is %d\n", client->audio_codec);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Stream has been playing for %3lu.%03lu seconds\n", suspend->time_playing / 1000, suspend->time_playing % 1000);

  _exit:
    /*
     * Tell the other thread that we have completed the suspend processing.
     */

    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_SUSPEND_COMPLETE);

    return result;
}


static void audio_client_thread_main(uint32_t context)
{
    audio_client_t* client = (audio_client_t *)context;
    wiced_result_t  result;
    uint32_t        events;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client thread begin\n");

    if (client == NULL)
    {
        return;
    }

    while (client->tag == AUDIO_CLIENT_TAG_VALID)
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&client->events, AUDIO_CLIENT_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & AUDIO_CLIENT_EVENT_SHUTDOWN)
        {
            break;
        }

        if (events & AUDIO_CLIENT_EVENT_PLAY)
        {
            audio_client_start_playback(client);
        }

        if (events & (AUDIO_CLIENT_EVENT_STOP | AUDIO_CLIENT_EVENT_HTTP_ERROR))
        {
            audio_client_stop(client, (events & AUDIO_CLIENT_EVENT_STOP) ? WICED_TRUE : WICED_FALSE);
            if (client->http_params.http_load_file)
            {
                if (client->http_params.http_file_data)
                {
                    free(client->http_params.http_file_data);
                    client->http_params.http_file_data = NULL;
                }
                client->http_params.http_load_file = WICED_FALSE;
            }
        }

        if (events & AUDIO_CLIENT_EVENT_PAUSE)
        {
            audio_client_pause(client);
        }

        if (events & AUDIO_CLIENT_EVENT_RESUME)
        {
            audio_client_resume(client);
        }

        if (events & AUDIO_CLIENT_EVENT_VOLUME)
        {
            client->params.volume = client->new_volume;
            if (client->audio)
            {
                audio_render_command(client->audio, AUDIO_RENDER_CMD_SET_VOLUME, (void*)client->params.volume);
            }
        }

        if (events & AUDIO_CLIENT_EVENT_SEEK)
        {
            audio_client_seek(client);
        }

        if (events & AUDIO_CLIENT_EVENT_EFFECT)
        {
            audio_client_effect(client);
        }

        if (events & AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE)
        {
            audio_client_process_header_complete(client);
        }

        if (events & AUDIO_CLIENT_SUSPEND)
        {
            audio_client_process_suspend(client);
        }

        if (events & AUDIO_CLIENT_EVENT_HTTP_THREAD_DONE)
        {
            audio_client_http_reader_stop(client, &client->http_params);
            if (client->http_params.http_load_file && client->http_params.http_file_data)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio client transition to idle in response to HTTP_THREAD_DONE\n");
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_HTTP_COMPLETE, client->http_params.http_file_data);
                audio_client_complete_playback_housekeeping(client, &client->http_params);
            }
        }

        if (events & AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE)
        {
            audio_client_stop_decoder(client, client->audio_codec);

            if (client->audio_configured == WICED_FALSE || client->audio_pushed == WICED_FALSE)
            {
                /*
                 * The audio was never configured successfully so the decoder must have shutdown before
                 * we played any audio. That means that HTTP might still be active and we'll never get
                 * a playback complete message from audio render. We need to make sure that we trigger
                 * a proper cleanup now.
                 */

                audio_client_stop(client, WICED_FALSE);
            }
            else
            {
                if (client->audio == NULL)
                {
                    /*
                     * If we're not in charge of playback then we go to idle when the decoder is done.
                     */

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio client transition to idle in response to DECODER_THREAD_DONE\n");
                    audio_client_complete_playback_housekeeping(client, &client->http_params);
                }
                else
                {
                    audio_client_check_for_early_playback_start(client, &client->http_params);
                }

                /*
                 * Tell the app that the decoder has finished.
                 */

                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_DECODE_COMPLETE, NULL);
            }
        }

        if (events & AUDIO_CLIENT_EVENT_PLAYBACK_COMPLETE)
        {
            if (client->state == AUDIO_CLIENT_STATE_PLAY)
            {
                if (client->post_decode == NULL)
                {
                    /*
                     * Make sure that the HTTP thread is shut down. If the decoder finished because
                     * of an error, it's possible to get here while the HTTP thread is still running.
                     */

                    audio_client_http_reader_stop(client, &client->http_params);
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio client transition to idle in response to PLAYBACK_COMPLETE\n");
                }
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_EOS, NULL);
                audio_client_complete_playback_housekeeping(client, &client->http_params);
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "PLAYBACK_COMPLETE: no action\n");
            }
        }
    }   /* while */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client thread end\n");

    WICED_END_OF_CURRENT_THREAD();
}


static void audio_client_shutdown(audio_client_t* client)
{
    audio_client_play_request_t* next;

    if (client == NULL)
    {
        return;
    }

    /*
     * Stop the main thread if it is running.
     */

    client->decoder_done = WICED_TRUE;
    client->tag          = AUDIO_CLIENT_TAG_INVALID;
    if (client->client_thread_ptr != NULL)
    {
        /*
         * We need to make sure everything is stopped in case we're
         * shutting down while a streaming session is active.
         */

        audio_client_stop(client, WICED_TRUE);

        wiced_rtos_thread_force_awake(&client->client_thread);
        wiced_rtos_thread_join(&client->client_thread);
        wiced_rtos_delete_thread(&client->client_thread);
        client->client_thread_ptr = NULL;
    }

    if (client->audio != NULL)
    {
        audio_render_deinit(client->audio);
        client->audio = NULL;
    }

    if (client->buf_pool)
    {
        bufmgr_pool_destroy(client->buf_pool);
        client->buf_pool = NULL;
    }

    wiced_rtos_deinit_event_flags(&client->events);
    wiced_rtos_deinit_event_flags(&client->http_params.http_events);
    wiced_rtos_deinit_event_flags(&client->decoder_events);
    wiced_rtos_deinit_mutex(&client->play_mutex);

    if (client->current_play)
    {
        free(client->current_play);
        client->current_play = NULL;
    }

    while (client->play_queue != NULL)
    {
        next = client->play_queue->next;
        free(client->play_queue);
        client->play_queue = next;
    }

    while (client->post_decode != NULL)
    {
        next = client->post_decode->next;
        free(client->post_decode);
        client->post_decode = next;
    }

    if (client->http_params.socket_ptr != NULL)
    {
        wiced_tcp_delete_socket(client->http_params.socket_ptr);
        client->http_params.socket_ptr = NULL;
    }

    if (client->data_bufs != NULL)
    {
        free(client->data_bufs);
        client->data_bufs = NULL;
    }

    audio_client_hls_deinit(client);

    if (client->suspend)
    {
        free(client->suspend);
        client->suspend = NULL;
    }

    free(client);
}


static audio_client_t* audio_client_initialize(audio_client_params_t* params)
{
    audio_client_t* client;
    audio_render_params_t render_params;
    wiced_result_t result;

    /*
     * Some basic validation of our parameters.
     */

    if (params->enable_playback)
    {
        if (params->device_id == AUDIO_DEVICE_ID_NONE || params->audio_buffer_num == 0 || params->audio_buffer_size == 0)
        {
            return NULL;
        }
    }
    else
    {
        if (params->buffer_get == NULL || params->buffer_release == NULL)
        {
            return NULL;
        }
    }

    if (params->high_threshold_read_inhibit && (params->data_threshold_high > 0 && params->data_buffer_preroll > params->data_threshold_high))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "HTTP buffer settings will cause read deadlock\n");
        return NULL;
    }

    /*
     * Allocate the main audio client structure.
     */

    if ((client = calloc(1, sizeof(audio_client_t))) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate audio client structure\n");
        return NULL;
    }

    /*
     * Copy the configuration parameters.
     */

    memcpy(&client->params, params, sizeof(audio_client_params_t));

    /*
     * Allocate the audio data buffers.
     */

    if ((client->data_bufs = calloc(1, client->params.data_buffer_num * sizeof(data_buf_t))) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate audio client data buffers\n");
        free(client);
        return NULL;
    }

    /*
     * Create the event flags.
     */

    result  = wiced_rtos_init_event_flags(&client->events);
    result |= wiced_rtos_init_event_flags(&client->http_params.http_events);
    result |= wiced_rtos_init_event_flags(&client->decoder_events);
    result |= wiced_rtos_init_mutex(&client->play_mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing audio client event flags\n");
        free(client->data_bufs);
        free(client);
        return NULL;
    }

    if (params->enable_playback)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio_client: enabling playback support\n");

        /*
         * Create the buffer pool.
         */

        if (bufmgr_pool_create(&client->buf_pool, "audio_client_bufs", sizeof(audio_buf_t) + params->audio_buffer_size,
                               params->audio_buffer_num, params->audio_buffer_num, 0, sizeof(uint32_t)) != BUFMGR_OK)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bufmgr_pool_create failed\n");
            goto _bail;
        }

        /*
         * Create the audio render component.
         */

        memset(&render_params, 0, sizeof(audio_render_params_t));
        render_params.device_id              = params->device_id;
        render_params.buffer_nodes           = params->audio_buffer_num;
        render_params.release_device_on_stop = params->release_device_on_stop;
        render_params.userdata               = client;
        render_params.buf_release_cb         = audio_render_buffer_release;
        render_params.event_cb               = audio_render_event;

        if (params->audio_period_size != 0)
        {
            render_params.period_size = params->audio_period_size;
        }
        else
        {
            render_params.period_size = AUDIO_CLIENT_PERIOD_SIZE;
        }

        client->audio = audio_render_init(&render_params);
        if (client->audio == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize audio render\n");
            goto _bail;
        }
    }

    /*
     * Make sure HTTP thread keeps a reference to client
     */

    client->http_params.client = client;

    /*
     * Init HLS linked list
     */

    audio_client_hls_init(client);

    /*
     * Set up the decoder functions.
     */

    client->decoders[AUDIO_CLIENT_CODEC_NULL].decoder_start      = audio_client_null_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_NULL].decoder_stop       = audio_client_null_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_NULL].decoder_ioctl      = audio_client_null_decoder_ioctl;
    client->decoders[AUDIO_CLIENT_CODEC_FLAC].decoder_start      = audio_client_flac_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_FLAC].decoder_stop       = audio_client_flac_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_FLAC].decoder_ioctl      = audio_client_flac_decoder_ioctl;
    client->decoders[AUDIO_CLIENT_CODEC_WAV].decoder_start       = audio_client_wav_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_WAV].decoder_stop        = audio_client_wav_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_WAV].decoder_ioctl       = audio_client_wav_decoder_ioctl;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_M4A].decoder_start   = audio_client_aac_m4a_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_M4A].decoder_stop    = audio_client_aac_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_M4A].decoder_ioctl   = audio_client_aac_decoder_ioctl;
    client->decoders[AUDIO_CLIENT_CODEC_MP3].decoder_start       = audio_client_mp3_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_MP3].decoder_stop        = audio_client_mp3_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_MP3].decoder_ioctl       = audio_client_mp3_decoder_ioctl;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_ADTS].decoder_start  = audio_client_aac_adts_decoder_start;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_ADTS].decoder_stop   = audio_client_aac_decoder_stop;
    client->decoders[AUDIO_CLIENT_CODEC_AAC_ADTS].decoder_ioctl  = audio_client_aac_decoder_ioctl;

    client->filters[AUDIO_CLIENT_CONTAINER_NULL].filter_create   = audio_client_null_filter_create;
    client->filters[AUDIO_CLIENT_CONTAINER_NULL].filter_destroy  = audio_client_null_filter_destroy;
    client->filters[AUDIO_CLIENT_CONTAINER_NULL].filter_ioctl    = audio_client_null_filter_ioctl;
    client->filters[AUDIO_CLIENT_CONTAINER_TS].filter_create     = audio_client_ts_filter_create;
    client->filters[AUDIO_CLIENT_CONTAINER_TS].filter_destroy    = audio_client_ts_filter_destroy;
    client->filters[AUDIO_CLIENT_CONTAINER_TS].filter_ioctl      = audio_client_ts_filter_ioctl;


    return client;

_bail:
    audio_client_shutdown(client);
    return NULL;
}


static wiced_result_t audio_client_suspend(audio_client_t* client, audio_client_suspend_t** suspend)
{
    uint32_t events;

    if (client == NULL || client->tag != AUDIO_CLIENT_TAG_VALID || suspend == NULL)
    {
        return WICED_BADARG;
    }
    *suspend = NULL;

    /*
     * If we're idle then there's nothing to suspend.
     */

    if (client->state == AUDIO_CLIENT_STATE_IDLE)
    {
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio client suspend playback\n");

    /*
     * Tell the main loop we want to suspend.
     */

    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_SUSPEND);

    /*
     * And wait for the response.
     */

    events = 0;
    wiced_rtos_wait_for_event_flags(&client->events, AUDIO_CLIENT_EVENT_SUSPEND_COMPLETE, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);

    /*
     * Do we have a suspend structure to give back to the caller?
     */

    *suspend = client->suspend;
    client->suspend = NULL;

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_suspend_resume(audio_client_t* client, audio_client_suspend_t* suspend)
{
    audio_client_play_request_t* request;

    if (client == NULL || client->tag != AUDIO_CLIENT_TAG_VALID || suspend == NULL)
    {
        return WICED_BADARG;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio client resume playback\n");

    request = (audio_client_play_request_t*)calloc(1, sizeof(audio_client_play_request_t) + strlen(suspend->uri) + 1);
    if (request == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate play request\n");
        free(suspend);
        return WICED_ERROR;
    }

    request->stream_uri = request->data;
    request->contiguous = WICED_FALSE;
    request->load_file  = WICED_FALSE;
    if (!suspend->live_stream)
    {
        request->start_offset_ms = suspend->time_playing;
    }
    strcpy(request->stream_uri, suspend->uri);
    free(suspend);

    if (audio_client_add_play_request(client, request) != WICED_SUCCESS)
    {
        free(request);
        return WICED_ERROR;
    }
    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_PLAY);

    return WICED_SUCCESS;
}


/** Initialize the audio client library.
 *
 * @param[in] params      : Pointer to the configuration parameters.
 *
 * @return Pointer to the audio client instance or NULL
 */

audio_client_ref audio_client_init(audio_client_params_t* params)
{
    audio_client_t* client;
    wiced_result_t result;

    /*
     * An event callback is required.
     */

    if (params == NULL || params->event_cb == NULL)
    {
        return NULL;
    }

    if ((client = audio_client_initialize(params)) == NULL)
    {
        return NULL;
    }

    /*
     * Now create the main audio client thread.
     */

    client->tag   = AUDIO_CLIENT_TAG_VALID;
    client->state = AUDIO_CLIENT_STATE_IDLE;

    result = wiced_rtos_create_thread_with_stack(&client->client_thread, AUDIO_CLIENT_THREAD_PRIORITY, "Audio client thread", audio_client_thread_main,
                                                 client->client_thread_stack_buffer, AUDIO_CLIENT_THREAD_STACK_SIZE, client);
    if (result == WICED_SUCCESS)
    {
        client->client_thread_ptr = &client->client_thread;
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio client thread\n");
        audio_client_shutdown(client);
        client = NULL;
    }

    return client;
}


/** Deinitialize the audio client library.
 *
 * @param[in] apollo_client : Pointer to the audio client instance.
 *
 * @return    Status of the operation.
 */

wiced_result_t audio_client_deinit(audio_client_ref audio_client)
{
    if (audio_client == NULL || audio_client->tag != AUDIO_CLIENT_TAG_VALID)
    {
        return WICED_BADARG;
    }

    audio_client_shutdown(audio_client);

    return WICED_SUCCESS;
}


/** Send an IOCTL to the audio client session.
 *
 * @param[in] audio_client  : Pointer to the audio client instance.
 * @param[in] cmd           : IOCTL command to process.
 * @param[inout] arg        : Pointer to argument for IOTCL.
 *
 * @return    Status of the operation.
 */

wiced_result_t audio_client_ioctl(audio_client_ref audio_client, AUDIO_CLIENT_IOCTL_T cmd, void* arg)
{
    audio_client_play_request_t* request;
    audio_client_play_offset_t* play_offset;
    audio_client_track_info_t* track_info;
    wiced_audio_config_t config;
    wiced_result_t result;
    int len;

    if (audio_client == NULL || audio_client->tag != AUDIO_CLIENT_TAG_VALID)
    {
        return WICED_BADARG;
    }

    switch (cmd)
    {
        case AUDIO_CLIENT_IOCTL_STOP:
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_STOP);
            break;

        case AUDIO_CLIENT_IOCTL_LOAD_FILE:
        case AUDIO_CLIENT_IOCTL_PLAY:
        case AUDIO_CLIENT_IOCTL_PLAY_OFFSET:
        case AUDIO_CLIENT_IOCTL_SET_CODEC:
        case AUDIO_CLIENT_IOCTL_PLAY_CONTIGUOUS:
            if (arg == NULL)
            {
                return WICED_ERROR;
            }

            play_offset = NULL;
            if (cmd != AUDIO_CLIENT_IOCTL_SET_CODEC)
            {
                if (cmd == AUDIO_CLIENT_IOCTL_PLAY_OFFSET)
                {
                    play_offset = (audio_client_play_offset_t*)arg;
                    if (play_offset->uri == NULL)
                    {
                        return WICED_ERROR;
                    }
                    len = strlen(play_offset->uri) + 1;
                }
                else
                {
                    len = strlen((char*)arg) + 1;
                }
            }
            else
            {
                len = 0;
            }

            request = (audio_client_play_request_t*)calloc(1, sizeof(audio_client_play_request_t) + len);
            if (request == NULL)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate play request\n");
                return WICED_ERROR;
            }

            if (cmd != AUDIO_CLIENT_IOCTL_SET_CODEC)
            {
                request->stream_uri = request->data;
                request->contiguous = (cmd == AUDIO_CLIENT_IOCTL_PLAY_CONTIGUOUS ? WICED_TRUE : WICED_FALSE);

                if (play_offset != NULL)
                {
                    strcpy(request->stream_uri, play_offset->uri);
                    request->start_offset_ms = play_offset->offset_ms;
                }
                else
                {
                    strcpy(request->stream_uri, (char*)arg);
                }
            }
            else
            {
                request->push_audio_codec = (AUDIO_CLIENT_CODEC_T)arg;
            }
            request->load_file = (cmd == AUDIO_CLIENT_IOCTL_LOAD_FILE ? WICED_TRUE : WICED_FALSE);

            if (audio_client_add_play_request(audio_client, request) != WICED_SUCCESS)
            {
                free(request);
                return WICED_ERROR;
            }
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_PLAY);
            break;

        case AUDIO_CLIENT_IOCTL_PAUSE:
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_PAUSE);
            break;

        case AUDIO_CLIENT_IOCTL_RESUME:
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_RESUME);
            break;

        case AUDIO_CLIENT_IOCTL_SET_VOLUME:
            audio_client->new_volume = (int)arg;
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_VOLUME);
            break;

        case AUDIO_CLIENT_IOCTL_TRACK_INFO:
            if (arg != NULL)
            {
                memset(arg, 0, sizeof(audio_client_track_info_t));
                if (audio_client->state != AUDIO_CLIENT_STATE_IDLE && audio_client->audio_codec != AUDIO_CLIENT_CODEC_NULL)
                {
                    audio_client_stream_info_t info;

                    result = audio_client->decoders[audio_client->audio_codec].decoder_ioctl(audio_client, DECODER_IOCTL_INFO, &info);
                    if (result == WICED_SUCCESS)
                    {
                        track_info = (audio_client_track_info_t*)arg;

                        track_info->current_sample = info.stream_current_sample;
                        track_info->total_samples  = info.stream_total_samples;
                        track_info->sample_rate    = info.stream_sample_rate;
                        track_info->channels       = info.stream_channels;
                        track_info->bps            = info.stream_bps;
                        track_info->bitrate        = info.bit_rate;
                    }
                }
            }
            break;

        case AUDIO_CLIENT_IOCTL_SEEK:
            if (audio_client->state == AUDIO_CLIENT_STATE_IDLE)
            {
                return WICED_ERROR;
            }
            if (audio_client->http_params.http_range_requests == WICED_FALSE || audio_client->http_params.http_total_content_length == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Range requests not supported\n");
                return WICED_UNSUPPORTED;
            }

            audio_client->seek_request_ms = (uint32_t)arg;
            audio_client->start_offset_ms = 0;
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_SEEK);
            break;

        case AUDIO_CLIENT_IOCTL_EFFECT:
            audio_client->effect_mode = (uint32_t)arg;
            wiced_rtos_set_event_flags(&audio_client->events, AUDIO_CLIENT_EVENT_EFFECT);
            break;

        case AUDIO_CLIENT_IOCTL_GET_QUEUE_WEIGHT:
            return audio_render_command(audio_client->audio, AUDIO_RENDER_CMD_GET_QUEUE_WEIGHT, arg);

        case AUDIO_CLIENT_IOCTL_IDLE_CHECK:
            if (arg != NULL)
            {
                if (audio_client->state == AUDIO_CLIENT_STATE_IDLE)
                {
                    *((uint32_t*)arg) = 1;
                }
                else
                {
                    *((uint32_t*)arg) = 0;
                }
            }
            break;

        case AUDIO_CLIENT_IOCTL_PUSH_BUFFER:
            if (!audio_client->accept_push_buffer)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Not accepting push buffers\n");
                return WICED_ERROR;
            }
            return process_pushed_buffer(audio_client, (audio_client_push_buf_t*)arg);

        case AUDIO_CLIENT_IOCTL_SET_RAW_CONFIG:
            /*
             * Raw playback only allowed when we're idle.
             */
            if (audio_client->state != AUDIO_CLIENT_STATE_IDLE || audio_client->play_queue)
            {
                return WICED_ERROR;
            }
            memcpy(&config, (wiced_audio_config_t*)arg, sizeof(wiced_audio_config_t));
            if (config.bits_per_sample == 16 && config.channels == 1)
            {
                config.channels = 2;
                audio_client->raw_mono_to_stereo = WICED_TRUE;
            }
            else
            {
                audio_client->raw_mono_to_stereo = WICED_FALSE;
            }
            if ((result = audio_client_audio_config(audio_client, &config)) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to set raw config\n");
                return result;
            }
            else
            {
                audio_client->accept_raw_buffer = WICED_TRUE;
                audio_client->audio_pushed      = WICED_FALSE;
            }
            break;

        case AUDIO_CLIENT_IOCTL_PUSH_RAW:
            if (!audio_client->accept_raw_buffer)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Not accepting raw push buffers\n");
                return WICED_ERROR;
            }
            else
            {
                audio_client_play_raw(audio_client, (audio_client_push_buf_t*)arg);
            }
            break;

        case AUDIO_CLIENT_IOCTL_SUSPEND:
            return audio_client_suspend(audio_client, (audio_client_suspend_t**)arg);

        case AUDIO_CLIENT_IOCTL_SUSPEND_RESUME:
            return audio_client_suspend_resume(audio_client, (audio_client_suspend_t*)arg);

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized ioctl: %d\n", cmd);
            break;
    }

    return WICED_SUCCESS;
}
