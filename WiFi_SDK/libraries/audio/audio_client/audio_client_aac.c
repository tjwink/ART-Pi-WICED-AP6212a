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

/** @file Audio Client AAC Decode Routines
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_log.h"
#include "wwd_assert.h"

#include "audio_client_private.h"
#include "audio_client_aac.h"
#include "audio_client_aac_private.h"

/******************************************************
 *                      Macros
 ******************************************************/


/******************************************************
 *                    Constants
 ******************************************************/

#define AAC_TAG_VALID               ( 0x61EDBA17 )
#define AAC_TAG_INVALID             ( 0xDEADBEEF )

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

int aac_start_offset_callback(aac_decoder_t* dec, uint32_t start_offset_ms, uint32_t start_offset_bytes)
{
    int mins;

    /*
     * Does the current stream support range operations?
     */

    if (dec->audio_client->http_params.http_range_requests == WICED_FALSE || dec->audio_client->http_params.http_total_content_length == 0)
    {
        return -1;
    }

    /*
     * Looks like we are good to go.
     */

    dec->audio_client->seek_position = start_offset_bytes;
    wiced_rtos_set_event_flags(&dec->audio_client->http_params.http_events, HTTP_EVENT_SEEK);

    mins = start_offset_ms / 1000 / 60;
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Initiating AAC start offset seek %lu (time: %02d:%02d.%03d)\n",
                  start_offset_bytes, mins, (start_offset_ms - (mins * 1000 * 60)) / 1000, start_offset_ms % 1000);

    /*
     * Prevent the output processing from doing a double seek.
     */

    dec->no_skip_output = WICED_TRUE;

    return 0;
}


wiced_result_t aac_pcm_output_push(aac_decoder_t* dec, aac_pcm_info_t* pcm_info, uint8_t* buf, uint32_t buf_len)
{
    audio_client_buf_t         audio_buf;
    wiced_result_t             result;
    uint8_t*                   inptr;
    uint8_t*                   outptr;
    uint32_t                   output_bytes = 0;
    int                        frames_to_copy;
    int                        output_framesize;
    int                        num_frames;
    int                        frames;
    int                        size;
    audio_client_buf_t*        audio_buf_ptr    = NULL;

    if (dec->pcm_first_buffer || dec->sample_rate != pcm_info->sr || dec->num_channels != pcm_info->chnum || dec->bits_per_sample != pcm_info->cbps)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_aac: CONFIGURE audio render\n");

        if (dec->pcm_first_buffer)
        {
            wiced_time_get_time(&dec->audio_client->play_start_time);
        }

        dec->pcm_first_buffer = WICED_FALSE;

        /*
         * set pcm info from the pcm_info
         */

        if (NULL == pcm_info)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: missing PCM info\n");
            dec->audio_client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        /* make sure we only have a MAX of two channels in the buffer */
        if (pcm_info->chnum != 2)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: TWO channels required for playback (chnum=%d)\n", pcm_info->chnum);
            dec->audio_client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        dec->sample_rate     = pcm_info->sr;
        dec->num_channels    = pcm_info->chnum;
        dec->bits_per_sample = pcm_info->cbps;
        dec->block_align     = (pcm_info->cbps >> 3) * pcm_info->chnum;

        dec->audio_config.sample_rate     = dec->sample_rate;
        dec->audio_config.channels        = dec->num_channels;
        dec->audio_config.bits_per_sample = dec->bits_per_sample;
        dec->audio_config.frame_size      = dec->block_align;
        dec->audio_config.volume          = dec->audio_client->params.volume;

        if (audio_client_audio_config(dec->audio_client, &dec->audio_config) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: audio config failed\n");
            dec->audio_client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        if ((dec->audio_client->hls_playlist_active == WICED_FALSE) && (dec->audio_client->start_offset_ms > 0) && (dec->no_skip_output == WICED_FALSE))
        {
            dec->audio_frames_offset = (uint32_t)((double)dec->sample_rate * (double)dec->audio_client->start_offset_ms / 1000.0);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Starting frame offset is %lu (time %lu)\n", dec->audio_frames_offset, dec->audio_client->start_offset_ms);
        }
        else
        {
            dec->audio_frames_offset = 0;
        }
    }

    /*
     * OUTPUT copy in chunks (if needed)
     */

    inptr = buf;
    num_frames = (buf_len * 2) / dec->block_align;

    while ((num_frames > 0) && (dec->audio_client->decoder_done == WICED_FALSE))
    {
        result = audio_client_buffer_get(dec->audio_client, &audio_buf, AUDIO_CLIENT_AUDIO_BUF_TIMEOUT);
        if (result != WICED_SUCCESS)
        {
            /* we probably didn't get a buffer because the buffers are full */
            continue;
        }

        /* keep a reference to audio_buf */
        audio_buf_ptr = &audio_buf;

        output_framesize = (dec->bits_per_sample * dec->num_channels) / 8;
        if (output_framesize * num_frames <= audio_buf.buflen)
        {
            frames_to_copy = num_frames;
        }
        else
        {
            frames_to_copy = audio_buf.buflen / output_framesize;
        }

        outptr = (uint8_t*)audio_buf.buf;
        switch (dec->bits_per_sample)
        {
            case 8:
            case 16:
            case 32:
                size = frames_to_copy * output_framesize;
                memcpy(outptr, inptr, size);
                inptr         += size;
                break;

            case 24:
                /*
                 * Copy over the the audio.
                 */

                frames = frames_to_copy;
                while (frames > 0)
                {
                    *outptr++ = 0;
                    *outptr++ = *inptr++;
                    *outptr++ = *inptr++;
                    *outptr++ = *inptr++;

                    *outptr++ = 0;
                    *outptr++ = *inptr++;
                    *outptr++ = *inptr++;
                    *outptr++ = *inptr++;

                    frames--;
                }
                break;

            default:
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: decoder corruption\n");
                return WICED_ERROR;
        }

        dec->audio_frames_played += frames_to_copy;
        num_frames -= frames_to_copy;

        audio_buf.offset = 0;
        audio_buf.curlen = frames_to_copy * output_framesize;

        /*
         * Are we starting playback somewhere other than the beginning?
         */

        if (dec->audio_client->start_offset_ms > 0 && dec->audio_frames_offset)
        {
            if (dec->audio_frames_offset > frames_to_copy)
            {
                audio_buf.curlen = 0;
                dec->audio_frames_offset -= frames_to_copy;
            }
            else
            {
                audio_buf.offset += dec->audio_frames_offset * output_framesize;
                audio_buf.curlen -= dec->audio_frames_offset * output_framesize;
                dec->audio_frames_offset = 0;
            }
        }

        while (dec->audio_client->decoder_done == WICED_FALSE)
        {
            result = audio_client_buffer_release(dec->audio_client, &audio_buf);
            if (result == WICED_SUCCESS)
            {
                /* audio_buf has been released; drop the reference */
                audio_buf_ptr = NULL;
                break;
            }
            else if (result == WICED_BADARG)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: Error pushing audio buffer\n");
                return WICED_ERROR;
            }
        }

        output_bytes += frames_to_copy * output_framesize;
    }

    /* we still have a reference to audio_buf; we need to release it */
    if (audio_buf_ptr != NULL)
    {
        if (audio_client_buffer_release(dec->audio_client, audio_buf_ptr) == WICED_SUCCESS)
        {
            audio_buf_ptr = NULL;
        }
    }

    return WICED_SUCCESS;
}


static void stop_and_destroy_aac_decoder(audio_client_t* client, wiced_bool_t flush)
{
    aac_decoder_t* dec = (aac_decoder_t*) client->decoder_handle;

    if (dec->fdk_handle)
    {
        /* BEGIN - WORKAROUND: FDK library */
        /* make a call decoder_Fill to detect the OUT_OF_MEMORY error */
        /* which internally will _Close() the aac decoder already */

        uint32_t  tmpBytes      = 0xFFFF;
        uint32_t* tmpBytesPtr   = &tmpBytes;
        uint32_t  tmpBytesValid = 0;
        uint32_t  tmpByteSize   = sizeof( uint32_t );

        if (AAC_DEC_OK != aacDecoder_Fill(dec->fdk_handle, (unsigned char**)&tmpBytesPtr, (const UINT*)&tmpByteSize, (UINT*)&tmpBytesValid))
        {
            dec->fdk_handle = NULL;
        }
        /* END - WORKAROUND */
    }

    if (dec->fdk_handle)
    {
        aacDecoder_Close(dec->fdk_handle);
        dec->fdk_handle = NULL;
    }

    if (dec->data_decode != NULL)
    {
        free(dec->data_decode);
        dec->data_decode = NULL;
    }

    if (dec->data_coalesce != NULL)
    {
        free(dec->data_coalesce);
        dec->data_coalesce = NULL;
    }

    if (dec->m4a_audio_specific_info)
    {
        free(dec->m4a_audio_specific_info);
        dec->m4a_audio_specific_info = NULL;
    }

    if (dec->m4a_sample_size_table.table.data)
    {
        free(dec->m4a_sample_size_table.table.data);
        dec->m4a_sample_size_table.table.data = NULL;
    }

    if (dec->m4a_chunk_offset_table.table.data)
    {
        free(dec->m4a_chunk_offset_table.table.data);
        dec->m4a_chunk_offset_table.table.data = NULL;
    }

    if (dec->m4a_time_sample_table.table.data)
    {
        free(dec->m4a_time_sample_table.table.data);
        dec->m4a_time_sample_table.table.data = NULL;
    }

    if (dec->m4a_sample_chunk_table.table.data)
    {
        free(dec->m4a_sample_chunk_table.table.data);
        dec->m4a_sample_chunk_table.table.data = NULL;
    }
}


static wiced_result_t create_and_start_aac_decoder(audio_client_t* client, aac_decoder_t* dec, AUDIO_CLIENT_CODEC_T stream_type)
{
    AAC_DECODER_ERROR fdk_err;

    if (stream_type != AUDIO_CLIENT_CODEC_AAC_M4A && stream_type != AUDIO_CLIENT_CODEC_AAC_ADTS)
    {
        return WICED_ERROR;
    }

    dec->fdk_handle = aacDecoder_Open(stream_type == AUDIO_CLIENT_CODEC_AAC_M4A ? TT_MP4_RAW : TT_MP4_ADTS, 1);
    if (dec->fdk_handle == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to open AAC decoder\n");
        return WICED_OUT_OF_HEAP_SPACE;
    }

    /*
     * We need to allocate a buffer for the decoded audio.
     */

    if ((dec->data_decode = calloc(DECODE_BUF_SIZE, 1)) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate AAC decode buffer\n");
        aacDecoder_Close(dec->fdk_handle);
        dec->fdk_handle = NULL;
        return WICED_OUT_OF_HEAP_SPACE;
    }

    if (stream_type == AUDIO_CLIENT_CODEC_AAC_M4A)
    {
        /*
         * We need to allocate a coalesce buffer for M4A processing.
         */

        if ((dec->data_coalesce = calloc(COALESCE_BUF_SIZE, 1)) == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate AAC coalesce buffer\n");
            aacDecoder_Close(dec->fdk_handle);
            dec->fdk_handle = NULL;
            free(dec->data_decode);
            dec->data_decode = NULL;
            return WICED_OUT_OF_HEAP_SPACE;
        }

        dec->m4a_stream_seek  = WICED_TRUE;
        dec->m4a_chunk_lookup = WICED_TRUE;
    }

    /*
     * Configure the decoder the way we want it, interleaved, stereo output in WAV order.
     */

    if ((fdk_err = aacDecoder_GetFreeBytes(dec->fdk_handle, (UINT*)&dec->fdk_internal_buf_size)) != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to get decoder free bytes: %lx\n", fdk_err);
        goto _bail;
    }

    if ((fdk_err = aacDecoder_SetParam(dec->fdk_handle, AAC_PCM_OUTPUT_INTERLEAVED, 1 /* interleaved */)) != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set decoder interleaving: %lx\n", fdk_err);
        goto _bail;
    }

    if ((fdk_err = aacDecoder_SetParam(dec->fdk_handle, AAC_PCM_OUTPUT_CHANNEL_MAPPING, 1 /* WAV order*/)) != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set decoder channel mapping: %lx\n", fdk_err);
        goto _bail;
    }

    if ((fdk_err = aacDecoder_SetParam(dec->fdk_handle, AAC_PCM_MIN_OUTPUT_CHANNELS, 2)) != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set decoder min channels: %lx\n", fdk_err);
        goto _bail;
    }

    if ((fdk_err = aacDecoder_SetParam(dec->fdk_handle, AAC_PCM_MAX_OUTPUT_CHANNELS, 2)) != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set decoder max channels: %lx\n", fdk_err);
        goto _bail;
    }

    dec->pcm_first_buffer = WICED_TRUE;
    dec->audio_client     = client;

    return WICED_SUCCESS;

  _bail:
    stop_and_destroy_aac_decoder(client, WICED_FALSE);
    return WICED_ERROR;
}


void aac_release_buffer(audio_client_t* client, data_buf_t* dbuf)
{
    data_buf_t* next_dbuf;

    /*
     * This buffer is done. Advance to the next one.
     */

    RELEASE_DBUF(client, dbuf);

    CHECK_FOR_THRESHOLD_LOW_EVENT(client);

    /*
     * More data waiting to be processed?
     */

    next_dbuf = &client->data_bufs[client->data_buf_ridx];
    if (next_dbuf->inuse)
    {
        wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
    }
}


static wiced_result_t process_aac_data(audio_client_t* client)
{
    aac_decoder_t*      dec;
    data_buf_t*         dbuf;
    audio_client_buf_t  audio_buf;
    wiced_result_t      result;

    dec = (aac_decoder_t*)client->decoder_handle;

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (!dbuf->inuse)
    {
        return WICED_SUCCESS;
    }

    /*
     * command buffer
     */

    if (dbuf->buflen == 0)
    {
        client->decoder_done = WICED_TRUE;

        /*
         * Tell audio render that no more data is coming for this stream.
         */

        memset(&audio_buf, 0, sizeof(audio_client_buf_t));
        audio_buf.flags = AUDIO_CLIENT_BUF_FLAG_EOS;
        audio_client_buffer_release(client, &audio_buf);

        RELEASE_DBUF(client, dbuf);

        return WICED_SUCCESS;
    }

    if (client->audio_codec == AUDIO_CLIENT_CODEC_AAC_ADTS)
    {
        result = adts_process_data(client);
    }
    else
    {
        result = m4a_process_data(client);
    }

    if (result != WICED_SUCCESS)
    {
        if (!client->decoder_done)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: error from process_data (%d)\n", result);

            stop_and_destroy_aac_decoder(client, WICED_FALSE);

            /*
             * We might be able to continue with an ADTS stream...
             */

            if (client->audio_codec != AUDIO_CLIENT_CODEC_AAC_ADTS ||
                create_and_start_aac_decoder(client, dec, client->audio_codec) != WICED_SUCCESS)
            {
                /* Decoder can't be restarted; we have to bail out */
                client->decoder_done = WICED_TRUE;
                return WICED_ERROR;
            }
        }
    }

    return WICED_SUCCESS;
}


static wiced_result_t flush_decoder_data(audio_client_t* client)
{
    data_buf_t* dbuf;

    while (1)
    {
        dbuf = &client->data_bufs[client->data_buf_ridx];
        if (!dbuf->inuse)
        {
            break;
        }

        /*
         * Discard this buffer.
         */

        RELEASE_DBUF(client, dbuf);
    }

    /*
     * Tell the other thread that we have completed the flush operation.
     */

    wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_FLUSH_COMPLETE);

    return WICED_SUCCESS;
}


static void audio_client_aac_thread(uint32_t arg)
{
    audio_client_t* client = (audio_client_t*)arg;
    uint32_t        events;
    wiced_result_t  result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_aac: aac_thread start\n");

    while (!client->decoder_done)
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&client->decoder_events, DECODER_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if ((result != WICED_SUCCESS) || client->decoder_done)
        {
            break;
        }

        if (events & DECODER_EVENT_FLUSH)
        {
            flush_decoder_data(client);
        }

        if (events & DECODER_EVENT_AUDIO_DATA)
        {
            if (process_aac_data(client) != WICED_SUCCESS)
            {
                break;
            }
        }
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_aac: aac_thread exit\n");

    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE);

    WICED_END_OF_CURRENT_THREAD();
}


static wiced_result_t audio_client_aac_decoder_start(audio_client_t* client, AUDIO_CLIENT_CODEC_T stream_type)
{
    aac_decoder_t* dec;
    wiced_result_t result;

    if (client == NULL)
    {
        return WICED_BADARG;
    }

    /*
     * Allocate the internal decoder structure.
     */

    dec = (aac_decoder_t*)calloc(1, sizeof(aac_decoder_t));
    if (dec == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: Unable to allocate decoder structure\n");
        return WICED_ERROR;
    }

    if (create_and_start_aac_decoder(client, dec, stream_type) != WICED_SUCCESS)
    {
        free(dec);
        return WICED_ERROR;
    }

    /*
     * complete client configs
     */

    client->decoder_handle = dec;
    dec->tag = AAC_TAG_VALID;

    /* Start decoder thread */
    client->decoder_done = WICED_FALSE;

    result = wiced_rtos_create_thread_with_stack(&client->decoder_thread, AUDIO_CLIENT_DECODER_AAC_THREAD_PRIORITY, "AAC Decoder",
                                                 audio_client_aac_thread, dec->stack, AUDIO_CLIENT_DECODER_AAC_STACK_SIZE, client);

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: start: wiced_rtos_create_thread failed %d\n", result);
        stop_and_destroy_aac_decoder(client, WICED_FALSE);
        client->decoder_handle = NULL;
        dec->tag = AAC_TAG_INVALID;
        free(dec);
        return WICED_ERROR;
    }
    else
    {
        client->decoder_thread_ptr = &client->decoder_thread;
    }

    return result;
}


wiced_result_t audio_client_aac_m4a_decoder_start(audio_client_t* client)
{
    return audio_client_aac_decoder_start(client, AUDIO_CLIENT_CODEC_AAC_M4A);
}


wiced_result_t audio_client_aac_adts_decoder_start(audio_client_t* client)
{
    return audio_client_aac_decoder_start(client, AUDIO_CLIENT_CODEC_AAC_ADTS);
}


wiced_result_t audio_client_aac_decoder_stop(audio_client_t* client)
{
    if (client == NULL || client->decoder_handle == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac: stop: Bad ARG\n");
        return WICED_BADARG;
    }

    client->decoder_done = WICED_TRUE;

    stop_and_destroy_aac_decoder(client, WICED_FALSE);

    if (client->decoder_thread_ptr != NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_aac_decoder_stop\n");

        wiced_rtos_thread_force_awake(&client->decoder_thread);
        wiced_rtos_thread_join(&client->decoder_thread);
        wiced_rtos_delete_thread(&client->decoder_thread);
        client->decoder_thread_ptr = NULL;

        if (client->decoder_handle)
        {
            ((aac_decoder_t*)client->decoder_handle)->tag = AAC_TAG_INVALID;
            free(client->decoder_handle);
            client->decoder_handle = NULL;
        }
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_client_aac_decoder_ioctl(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg)
{
    aac_decoder_t* dec;
    audio_client_stream_info_t* info;
    uint32_t events;

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac_decoder_ioctl: Bad handle\n");
        return WICED_BADARG;
    }

    dec = (aac_decoder_t*)client->decoder_handle;
    if (dec == NULL || dec->tag != AAC_TAG_VALID)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_aac_decoder_ioctl: Bad decoder handle\n");
        return WICED_BADARG;
    }

    switch (ioctl)
    {
        case DECODER_IOCTL_INFO:
            info = (audio_client_stream_info_t*)arg;
            if (info != NULL)
            {
                if (0 == dec->audio_frames_total && 0 != dec->m4a_track_info.duration)
                {
                    dec->audio_frames_total = (uint32_t)dec->m4a_track_info.duration;
                }

                info->stream_total_samples  = 0;
                info->stream_current_sample = 0;
                if (dec->num_channels != 0)
                {
                    info->stream_total_samples  = dec->audio_frames_total;
                    info->stream_current_sample = dec->audio_frames_played;
                    if (client->audio_codec == AUDIO_CLIENT_CODEC_AAC_M4A && client->start_offset_ms && dec->no_skip_output)
                    {
                        /*
                         * We didn't start playing from the beginning of the file. Account for that here.
                         */

                        info->stream_current_sample += (uint64_t)dec->sample_rate * (uint64_t)client->start_offset_ms / (uint64_t)1000;
                    }
                }
                info->stream_sample_rate    = dec->sample_rate;
                info->stream_channels       = dec->num_channels;
                info->stream_bps            = dec->bits_per_sample;
            }
            break;

        case DECODER_IOCTL_SET_POSITION:
            if (client->audio_codec == AUDIO_CLIENT_CODEC_AAC_M4A)
            {
                dec->bitstream_idx            = (uint32_t)arg;
                dec->wait_for_stream_position = 0;
            }
            else
            {
                return WICED_UNSUPPORTED;
            }
            break;

        case DECODER_IOCTL_FLUSH:
            /*
             * Tell the decoder thread we want to flush.
             */

            wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_FLUSH);

            /*
             * And wait for the response.
             */

            events = 0;
            wiced_rtos_wait_for_event_flags(&client->decoder_events, DECODER_EVENT_FLUSH_COMPLETE, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
            break;

        case DECODER_IOCTL_GET_SEEK_POSITION:
        default:
            return WICED_UNSUPPORTED;
    }

    return WICED_SUCCESS;
}
