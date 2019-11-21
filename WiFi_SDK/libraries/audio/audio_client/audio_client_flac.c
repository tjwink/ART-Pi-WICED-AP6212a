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
 * File Audio Client FLAC Decode Routines
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_log.h"
#include "wwd_assert.h"

#include "audio_client_private.h"
#include "audio_client_flac.h"

/* FLAC library includes */
#include <FLAC/ordinals.h>
#include <stream_decoder.h>
#include <metadata.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define FLAC_TAG_VALID                  0xC0FFEE00
#define FLAC_TAG_INVALID                0xDEADBEEF

#define AUDIO_CLIENT_DECODER_FLAC_THREAD_PRIORITY   (WICED_DEFAULT_LIBRARY_PRIORITY)
#define AUDIO_CLIENT_DECODER_FLAC_STACK_SIZE        (8 * 1024)

#define MILLISECOND_PER_SECOND_FP                   (1000.0)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    FLAC_INPUT_BUFFER_FL = 0,
    FLAC_INPUT_BUFFER_FR,
    FLAC_INPUT_BUFFER_FC,
    FLAC_INPUT_BUFFER_LFE,
    FLAC_INPUT_BUFFER_BL,
    FLAC_INPUT_BUFFER_BR,

    FLAC_INPUT_BUFFER_MAX
} FLAC_INPUT_BUFFER_MAP_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t                tag;

    FLAC__StreamDecoder*    flac_decoder;
    FLAC__StreamMetadata*   flac_metadata;

    wiced_bool_t            flac_header_found;
    wiced_bool_t            wait_for_stream_position;

    /*
     *  Audio format information.
     */

    uint64_t                stream_total_samples;
    uint16_t                stream_num_channels;
    uint32_t                stream_sample_rate;
    uint16_t                stream_block_align;
    uint16_t                stream_bits_per_sample;

    uint16_t                render_num_channels;
    uint16_t                render_bits_per_sample;

    uint64_t                stream_current_sample;

    uint32_t                audio_frames_offset;

    uint8_t                 stack[AUDIO_CLIENT_DECODER_FLAC_STACK_SIZE];
} flac_decoder_t;


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

static wiced_result_t flac_push_data_to_audio_render(audio_client_t* client, const FLAC__Frame *frame, const FLAC__int32 * const buffer[]);

/******************************************************
 *       FLAC Callback Function Definitions
 ******************************************************/

/* FLAC__StreamDecoderReadCallback read_callback
 *
 *  decoder     - [in] returned from FLAC__stream_decoder_new();
 *  buffer      - [in] pointer to store new data read in by this routine
 *  bytes       - [in] pointer to size of buffer
 *                [out] fill with number of bytes read by this routine (0 for eof)
 *  client_data - [in] pointer provided when initializing the streamer
 *
 * return one of these:
 * FLAC__STREAM_DECODER_READ_STATUS_CONTINUE
 * FLAC__STREAM_DECODER_READ_STATUS_END_OF_STREAM
 * FLAC__STREAM_DECODER_READ_STATUS_ABORT
 *
 * const char * const FLAC__StreamDecoderReadStatusString[];
 */
static FLAC__StreamDecoderReadStatus flac_read_callback(const FLAC__StreamDecoder *decoder, FLAC__byte buffer[], size_t *bytes, void *client_data)
{
    audio_client_t*     client = (audio_client_t *)client_data;
    data_buf_t*         dbuf = NULL;
    size_t              bytes_to_copy;

    /* sanity check */
    wiced_assert("flac_read_callback() BAD ARGS!\n", ((decoder != NULL) && (client != NULL) && (bytes != NULL) && (buffer != NULL)) );

    while (!client->decoder_done)
    {
        dbuf = &client->data_bufs[client->data_buf_ridx];
        if (!dbuf->inuse)
        {
            wiced_rtos_delay_milliseconds(1);
            continue;
        }

        if (dbuf->buflen == 0)
        {
            RELEASE_DBUF(client, dbuf);
            *bytes = 0;
            return FLAC__STREAM_DECODER_READ_STATUS_END_OF_STREAM;
        }

        break;
    }

    if (client->decoder_done)
    {
        *bytes = 0;
        return FLAC__STREAM_DECODER_READ_STATUS_ABORT;
    }

    /*
     * Copy over the data.
     */

    bytes_to_copy = dbuf->buflen - dbuf->bufused;
    if (bytes_to_copy > *bytes)
    {
        bytes_to_copy = *bytes;
    }

    memcpy(buffer, &dbuf->buf[dbuf->bufused], bytes_to_copy);
    dbuf->bufused += bytes_to_copy;
    *bytes = bytes_to_copy;

    if (dbuf->bufused >= dbuf->buflen)
    {
        RELEASE_DBUF(client, dbuf);

        CHECK_FOR_THRESHOLD_LOW_EVENT(client);
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "flac_read_callback: copied:%ld\n", bytes_to_copy);

    return FLAC__STREAM_DECODER_READ_STATUS_CONTINUE;
}


/* FLAC__StreamDecoderWriteCallback write_callback
 *
 * decoder              - [in] returned from FLAC__stream_decoder_new();
 * frame                - [in] The description of the decoded frame.  See FLAC__Frame.
 * buffer               - [in] An array of pointers to decoded channels of data.
 *                             Each pointer will point to an array of signed
 *                             samples of length \a frame->header.blocksize.
 *                             Channels will be ordered according to the FLAC
 *                             specification; see the documentation for the frame header.
 *
 *  client_data         - [in] pointer provided when initializing the streamer
 *
 * return one of these:
 * FLAC__STREAM_DECODER_WRITE_STATUS_CONTINUE
 * FLAC__STREAM_DECODER_WRITE_STATUS_ABORT
 *
 * const char * const FLAC__StreamDecoderWriteStatusString[];
 *
 * FLAC__CHANNEL_ASSIGNMENT_INDEPENDENT = 0
 *      Independent. The left and right channels are coded independently.
 * FLAC__CHANNEL_ASSIGNMENT_LEFT_SIDE = 1
 *      Left-side. The left channel and side channel are coded.
 * FLAC__CHANNEL_ASSIGNMENT_RIGHT_SIDE = 2
 *      Right-side. The right channel and side channel are coded
 * FLAC__CHANNEL_ASSIGNMENT_MID_SIDE = 3
 *      Mid-side. The left and right channels are transformed into mid and side channels.
 *      The mid channel is the midpoint (average) of the left and right signals,
 *      and the side is the difference signal (left minus right).
 *
 * extern FLAC_API const char * const FLAC__ChannelAssignmentString[];
 *
 */
static FLAC__StreamDecoderWriteStatus flac_write_callback(const FLAC__StreamDecoder *decoder, const FLAC__Frame *frame, const FLAC__int32 * const buffer[], void *client_data)
{
    FLAC__StreamDecoderWriteStatus  retval = FLAC__STREAM_DECODER_WRITE_STATUS_CONTINUE;
    audio_client_t* client = (audio_client_t *)client_data;

    /* sanity check */
    wiced_assert("flac_test_tell_callback() BAD ARGS!\r\n", ((decoder != NULL) && (client != NULL)) );

    if (client->decoder_done == WICED_TRUE)
    {
        return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
    }

    if (flac_push_data_to_audio_render(client, frame, buffer) != WICED_SUCCESS)
    {
        retval = FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "flac_write_callback: ret %s\n", FLAC__StreamDecoderWriteStatusString[retval]);

    return retval;
}


/* FLAC__StreamDecoderMetadataCallback metadata_callback
 *
 * The metadata block that is passed in must not be
 *  modified, and it doesn't live beyond the callback, so you should make
 *  a copy of it with FLAC__metadata_object_clone() if you will need it
 *  elsewhere.  Since metadata blocks can potentially be large, by
 *  default the decoder only calls the metadata callback for the
 *  \c STREAMINFO block; you can instruct the decoder to pass or filter
 *  other blocks with FLAC__stream_decoder_set_metadata_*() calls.
 *
 * decoder              - [in] returned from FLAC__stream_decoder_new();
 * metadata             - [in] The decoded metadata block (ONLY VALID DURING THIS CALLBACK)
 * client_data          - [in] pointer provided when initializing the streamer
 *
 */

static void flac_metadata_callback(const FLAC__StreamDecoder *decoder, const FLAC__StreamMetadata *metadata, void *client_data)
{
    audio_client_t* client = (audio_client_t *)client_data;
    flac_decoder_t* dec;
    wiced_audio_config_t audio_config;

     dec = (flac_decoder_t*)client->decoder_handle;

    /* sanity check */
     wiced_assert("flac_metadata_callback: BAD ARGS!\n", ((decoder != NULL) && (client != NULL) && (dec != NULL) && (metadata != NULL)) );

    /* we get metadata info from the stream here - what to do with it ? */
    /* print some stats */
    switch(metadata->type)
    {
        case FLAC__METADATA_TYPE_STREAMINFO:    /* 0 */
            /* save for later */
            dec->stream_total_samples   = metadata->data.stream_info.total_samples;
            dec->stream_sample_rate     = metadata->data.stream_info.sample_rate;
            dec->stream_num_channels    = metadata->data.stream_info.channels;
            dec->stream_bits_per_sample = metadata->data.stream_info.bits_per_sample;

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    sample rate    : %d Hz\n", dec->stream_sample_rate);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    channels       : %d\n", dec->stream_num_channels);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    bits per sample: %d\n", dec->stream_bits_per_sample);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    total samples  : %lu\n", (uint32_t)dec->stream_total_samples);

            if (dec->stream_sample_rate !=  44100 && dec->stream_sample_rate !=  48000 && dec->stream_sample_rate != 96000 &&
                dec->stream_sample_rate != 88200 && dec->stream_sample_rate != 176400 && dec->stream_sample_rate != 192000)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "We do not support FLAC sample_rate: %ld at this time <---!\n", dec->stream_sample_rate);
                client->decoder_done = WICED_TRUE;
                break;
            }

            dec->render_num_channels    = 2;
            dec->render_bits_per_sample = 32;

            /*
             * Tell audio render about the audio format.
             * Use two channel output for audio render.
             */

            audio_config.bits_per_sample = dec->render_bits_per_sample;
            audio_config.channels        = dec->render_num_channels;
            audio_config.sample_rate     = dec->stream_sample_rate;
            audio_config.frame_size      = dec->render_bits_per_sample * dec->render_num_channels / 8;
            audio_config.volume          = client->params.volume;

            if (audio_client_audio_config(client, &audio_config) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_audio_render_configure: failed\n");
                client->decoder_done = WICED_TRUE;
            }

            if (client->start_offset_ms > 0)
            {
                dec->audio_frames_offset = (uint32_t)((float)dec->stream_sample_rate * (float)client->start_offset_ms / 1000.0);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Starting frame offset is %lu (time %lu)\n", dec->audio_frames_offset, client->start_offset_ms);

                /*
                 * We need to set the wait for stream position flag first. Since the HTTP thread
                 * runs at a higher priority, it's possible for it to make a new connection and
                 * set the stream position in response to a seek request before we get a chance
                 * to run again.
                 */

                dec->wait_for_stream_position = WICED_TRUE;
                if (!audio_client_check_start_offset_seek(client))
                {
                    /*
                     * No seek operation triggered. Reset the wait flag.
                     */

                    dec->wait_for_stream_position = WICED_FALSE;
                }
            }
            else
            {
                dec->audio_frames_offset = 0;
            }
            break;

        case FLAC__METADATA_TYPE_PADDING:        /* 1 */
        case FLAC__METADATA_TYPE_APPLICATION:    /* 2 */
        case FLAC__METADATA_TYPE_SEEKTABLE:      /* 3 */
        case FLAC__METADATA_TYPE_VORBIS_COMMENT: /* 4 */
        case FLAC__METADATA_TYPE_CUESHEET:       /* 5 */
        case FLAC__METADATA_TYPE_PICTURE:        /* 6 */
            break;
        case FLAC__METADATA_TYPE_UNDEFINED:      /* 7 */
        default:
            break;
    }

    /* this is allocated memory! need to free it when we are done with the flac decoder instance */
    if (dec->flac_metadata != NULL)
    {
        free(dec->flac_metadata);
    }

    dec->flac_metadata = FLAC__metadata_object_clone(metadata);
}


/* FLAC__StreamDecoderErrorCallback error_callback
 *
 * decoder              - [in] returned from FLAC__stream_decoder_new();
 * status               - [in] The error encountered by the decoder.
 * client_data          - [in] pointer provided when initializing the streamer
 *
 *status is one of:
 * FLAC__STREAM_DECODER_ERROR_STATUS_LOST_SYNC
 * FLAC__STREAM_DECODER_ERROR_STATUS_BAD_HEADER
 * FLAC__STREAM_DECODER_ERROR_STATUS_FRAME_CRC_MISMATCH
 * FLAC__STREAM_DECODER_ERROR_STATUS_UNPARSEABLE_STREAM
 *
 * const char * const FLAC__StreamDecoderErrorStatusString[];
 */

static void flac_error_callback(const FLAC__StreamDecoder *decoder, FLAC__StreamDecoderErrorStatus status, void *client_data)
{
    audio_client_t* client = (audio_client_t *)client_data;

    UNUSED_PARAMETER(client);
    UNUSED_PARAMETER(decoder);
    UNUSED_PARAMETER(status);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "flac_error_callback: %s\n", FLAC__StreamDecoderErrorStatusString[status]);
}


static wiced_result_t flac_push_data_to_audio_render(audio_client_t* client, const FLAC__Frame *frame, const FLAC__int32 * const buffer[])
{
    flac_decoder_t* dec = (flac_decoder_t*)client->decoder_handle;
    audio_client_buf_t audio_buf;
    const FLAC__int32*  in[FLAC_INPUT_BUFFER_MAX];
    wiced_bool_t        fill_FL_with_zero, fill_FR_with_zero;
    uint32_t  samples_given_to_us;
    uint16_t  count;
    uint32_t* out_buff;
    wiced_result_t result;

    if (dec->wait_for_stream_position)
    {
        return WICED_SUCCESS;
    }

    /* here is where we get the decoded LPCM data - write it to audio render */
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "flac_push_data_to_audio_render:\n");
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "         blocksize: %d\n", frame->header.blocksize);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "          channels: %d\n", frame->header.channels);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "      this channel:(%d) %s\n", frame->header.channel_assignment, FLAC__ChannelAssignmentString[frame->header.channel_assignment]);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "   bits per sample: %d\n", frame->header.bits_per_sample);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "              rate: %d\n", frame->header.sample_rate);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "         buffer[0]: %p\n", buffer[0]);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "         buffer[1]: %p\n", buffer[1]);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "         buffer[2]: %p\n", buffer[2]);

    if (frame->header.number_type == FLAC__FRAME_NUMBER_TYPE_SAMPLE_NUMBER)
    {
        /*
         * Make sure our notion of the current sample is up to date. If a seek operation was performed
         * than our position in the stream may have changed.
         */

        if (dec->stream_current_sample != frame->header.number.sample_number)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Current sample correction: %lu -> %lu\n",
                          (uint32_t)dec->stream_current_sample, (uint32_t)frame->header.number.sample_number);
        }
        dec->stream_current_sample = frame->header.number.sample_number;
    }

    samples_given_to_us = frame->header.blocksize;              /* updated in loop if we don't use all the first time */

    /*
     *  FLAC output buffers
     *  #       Buffer
     *  Chan    [0] [1] [2] [3] [4] [5]
     *  1       FL
     *  2       FL  FR
     *  3       FL  FR  FC
     *  4       FL  FR  BL  BR
     *  5       FL  FR  FC  BL  BR
     *  5.1     FL  FR  FC  LFE BL  BR
     *
     *
     * clear out our input buffer pointers, fill input based on what we want for output
     * cross-correlate the input buffers to the output we want in out render buffer
     * so that in[FLAC_INPUT_BUFFER_XX] always points to the appropriate buffer (may be NULL!)
     */

    memset(in, 0, sizeof(in));
    in[FLAC_INPUT_BUFFER_FL] = buffer[0];
    in[FLAC_INPUT_BUFFER_FR] = buffer[1];
    in[FLAC_INPUT_BUFFER_FC] = buffer[2];
    switch (frame->header.channels)
    {
        case 4:
            in[FLAC_INPUT_BUFFER_FC] = NULL;
            in[FLAC_INPUT_BUFFER_BL] = buffer[2];
            in[FLAC_INPUT_BUFFER_BR] = buffer[3];
            break;
        case 5:
            in[FLAC_INPUT_BUFFER_BL] = buffer[3];
            in[FLAC_INPUT_BUFFER_BR] = buffer[4];
        break;
        case 6: /* 5.1 */
            in[FLAC_INPUT_BUFFER_LFE] = buffer[3];
            in[FLAC_INPUT_BUFFER_BL]  = buffer[4];
            in[FLAC_INPUT_BUFFER_BR]  = buffer[5];
            break;
        default:
            break;
    }

    /* if the output buffer is for 2 channels, but FL or FR is not in the channel mask, we need to fill
     * the other part of the buffer with 0x00
     */

    fill_FL_with_zero = WICED_FALSE;
    fill_FR_with_zero = WICED_FALSE;

    while (samples_given_to_us > 0 && client->decoder_done == WICED_FALSE)
    {
        uint32_t chunk_size, samples_to_copy;
        uint32_t output_bytes_per_sample;

        result = audio_client_buffer_get(client, &audio_buf, AUDIO_CLIENT_AUDIO_BUF_TIMEOUT);
        if (result != WICED_SUCCESS)
        {
            /* we probably didn't get a buffer because the buffers are full */
            continue;
        }

        /*
         * All FLAC decoded samples will be converted to 32 bits in size.
         * Make sure the output buffer has enough room for all the data.
         */

        output_bytes_per_sample = dec->render_bits_per_sample / 8;
        chunk_size              = (samples_given_to_us * output_bytes_per_sample * dec->render_num_channels);
        if (chunk_size > audio_buf.buflen)
        {
            chunk_size = audio_buf.buflen;
        }

        samples_to_copy = chunk_size / (output_bytes_per_sample * dec->render_num_channels);

        /* copy samples over to where audio render wants them
         * The array "in" has 1 pointer per input, each being a 32bit buffer.
         * we want 16 or 32 bits per channel per sample interleaved into the output.
         * NOTE: For now we only pull out the FL and FR channels.
         */

        count    = samples_to_copy;
        out_buff = (uint32_t*)audio_buf.buf;
        switch (frame->header.bits_per_sample)
        {
            case 8:
                while (count > 0)
                {
                    if (fill_FL_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FL] == NULL)
                    {
                        *out_buff++ = 0x00;
                    }
                    else
                    {
                        *out_buff++ = ((uint8_t)(*in[FLAC_INPUT_BUFFER_FL]++)) << 24;
                    }
                    if (fill_FR_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FR] == NULL)
                    {
                        *out_buff++   = 0x00;
                    }
                    else
                    {
                        *out_buff++ = ((uint8_t)(*in[FLAC_INPUT_BUFFER_FR]++)) << 24;
                    }
                    count--;
                }
                break;

            case 16:
                while (count > 0)
                {
                    if (fill_FL_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FL] == NULL)
                    {
                        *out_buff++ = 0x00;
                    }
                    else
                    {
                        *out_buff++ = (uint16_t)(*in[FLAC_INPUT_BUFFER_FL]++) << 16;
                    }
                    if (fill_FR_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FR] == NULL)
                    {
                        *out_buff++   = 0x00;
                    }
                    else
                    {
                        *out_buff++ = (uint16_t)(*in[FLAC_INPUT_BUFFER_FR]++) << 16;
                    }
                    count--;
                }
                break;
            case 24:
                /* input of 24 needs to be shifted up 8 bits */
                while (count > 0)
                {
                    if (fill_FL_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FL] == NULL)
                    {
                        *out_buff++ = 0x00;
                    }
                    else
                    {
                        *out_buff++ = ((uint32_t)(*in[FLAC_INPUT_BUFFER_FL]++)) << 8;
                    }
                    if (fill_FR_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FR] == NULL)
                    {
                        *out_buff++   = 0x00;
                    }
                    else
                    {
                        *out_buff++ = ((uint32_t)(*in[FLAC_INPUT_BUFFER_FR]++)) << 8;
                    }
                    count--;
                }
                break;
            case 32:
                while (count > 0)
                {
                    if (fill_FL_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FL] == NULL)
                    {
                        *out_buff++ = 0x00;
                    }
                    else
                    {
                        *out_buff++ = (*in[FLAC_INPUT_BUFFER_FL]++);
                    }
                    if (fill_FR_with_zero == WICED_TRUE || in[FLAC_INPUT_BUFFER_FR] == NULL)
                    {
                        *out_buff++   = 0x00;
                    }
                    else
                    {
                        *out_buff++ = (*in[FLAC_INPUT_BUFFER_FR]++);
                    }
                    count--;
                }
                break;

            default:
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "flac_push_data_to_audio_render: unsupported sample size (%u) !\n", frame->header.bits_per_sample);
                audio_client_buffer_release(client, &audio_buf);
                return WICED_UNSUPPORTED;
        }

        /* fill the buffer info */

        audio_buf.offset = 0;
        audio_buf.curlen = samples_to_copy * (output_bytes_per_sample * dec->render_num_channels);

        /*
         * Are we starting playback somewhere other than the beginning?
         */

        if (client->start_offset_ms > 0 && dec->audio_frames_offset)
        {
            if (dec->audio_frames_offset > samples_to_copy)
            {
                audio_buf.curlen = 0;
                dec->audio_frames_offset -= samples_to_copy;
            }
            else
            {
                audio_buf.offset += dec->audio_frames_offset * (output_bytes_per_sample * dec->render_num_channels);
                audio_buf.curlen -= dec->audio_frames_offset * (output_bytes_per_sample * dec->render_num_channels);
                dec->audio_frames_offset = 0;
            }
        }

        result = audio_client_buffer_release(client, &audio_buf);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error releasing audio buffer\n");
            return WICED_ERROR;
        }

        samples_given_to_us -= samples_to_copy;

        dec->stream_current_sample += samples_to_copy;
    } /* while samples to send */

    return WICED_SUCCESS;
}


static wiced_result_t flac_decoder_start(audio_client_t* client)
{
    flac_decoder_t* dec = (flac_decoder_t*)client->decoder_handle;
    FLAC__StreamDecoderInitStatus flac_init_status;

    if (dec == NULL)
    {
        return WICED_ERROR;
    }

    if (dec->flac_decoder == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "flac_decoder_start: FLAC decoder not initialized!\n");
        return WICED_ERROR;
    }

    /* make sure the decoder is clean */
    if (FLAC__stream_decoder_flush(dec->flac_decoder) == false)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "FLAC decoder flush failed\n");
    }

    if (FLAC__stream_decoder_finish(dec->flac_decoder) == false)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "FLAC decoder finish failed\n");
    }

    flac_init_status = FLAC__stream_decoder_init_stream(dec->flac_decoder, flac_read_callback, NULL, NULL, NULL, NULL,
                                                        flac_write_callback, flac_metadata_callback, flac_error_callback, client);

    if (flac_init_status != FLAC__STREAM_DECODER_INIT_STATUS_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to init FLAC stream\n");
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


static wiced_result_t process_flac_data(audio_client_t* client)
{
    flac_decoder_t*     dec = (flac_decoder_t*)client->decoder_handle;
    audio_client_buf_t  audio_buf;
    data_buf_t*         dbuf;
    FLAC__bool          ret;

    if (dec == NULL || dec->tag != FLAC_TAG_VALID)
    {
        return WICED_ERROR;
    }

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (!dbuf->inuse)
    {
        return WICED_SUCCESS;
    }

    if (dbuf->buflen == 0)
    {
        /*
         * No more data will be sent to us.
         */

        if (dec->flac_decoder && dec->flac_header_found)
        {
            FLAC__stream_decoder_finish(dec->flac_decoder);
        }

        RELEASE_DBUF(client, dbuf);
        client->decoder_done = WICED_TRUE;

        /*
         * Tell audio render that no more data is coming for this stream.
         */

        memset(&audio_buf, 0, sizeof(audio_client_buf_t));
        audio_buf.flags = AUDIO_CLIENT_BUF_FLAG_EOS;
        audio_client_buffer_release(client, &audio_buf);
        return WICED_SUCCESS;
    }

    /*
     * Are we looking for the header?
     */

    if (dec->flac_header_found == WICED_FALSE)
    {
        if (dbuf->buf[0] == 'f' && dbuf->buf[1] == 'L'  && dbuf->buf[2] == 'a' && dbuf->buf[3] == 'C')
        {
            dec->flac_header_found = WICED_TRUE;

            /*
             * Start up the decoder after we see the header
             */

            if (flac_decoder_start(client) != WICED_SUCCESS)
            {
                goto _errout;
            }

            wiced_time_get_time(&client->play_start_time);
        }
    }

    if (dec->flac_header_found && !dec->wait_for_stream_position)
    {
        /*
         * Tell the FLAC decoder to run. It will pull data from the dbuf in the read callback.
         */

        ret = FLAC__stream_decoder_process_single(dec->flac_decoder);
        if (ret == false)
        {
            /*
             * A fatal error has been reported from the decoder.
             * Tell audio render that no more data is coming for this stream.
             */

            client->decoder_done = WICED_TRUE;

            memset(&audio_buf, 0, sizeof(audio_client_buf_t));
            audio_buf.flags = AUDIO_CLIENT_BUF_FLAG_EOS;
            audio_client_buffer_release(client, &audio_buf);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "FLAC__stream_decoder_process_single: ret = %d\n", ret);
            goto _errout;
        }
    }
    else
    {
        /*
         * This buffer is done. Advance to the next one.
         */

        RELEASE_DBUF(client, dbuf);

        CHECK_FOR_THRESHOLD_LOW_EVENT(client);
    }

    /*
     * More data waiting to be processed?
     */

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (dbuf->inuse)
    {
        wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
    }

    return WICED_SUCCESS;

_errout:
    RELEASE_DBUF(client, dbuf);

    return WICED_ERROR;
}


static wiced_result_t flush_flac_data(audio_client_t* client)
{
    flac_decoder_t* dec = (flac_decoder_t*)client->decoder_handle;
    data_buf_t* dbuf;

    /*
     * Flush the decoder.
     */

    if (dec && dec->flac_decoder)
    {
        if (FLAC__stream_decoder_flush(dec->flac_decoder) == false)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "FLAC decoder flush failed\n");
        }
    }

    /*
     * And clear out any buffers waiting to be processed.
     */

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


static void audio_client_flac_thread(uint32_t arg)
{
    audio_client_t*     client = (audio_client_t*)arg;
    uint32_t            events;
    wiced_result_t      result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client FLAC decoder start\n");

    while (!client->decoder_done)
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&client->decoder_events, DECODER_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS || client->decoder_done)
        {
            break;
        }

        if (events & DECODER_EVENT_FLUSH)
        {
            flush_flac_data(client);
        }

        if (events & DECODER_EVENT_AUDIO_DATA)
        {
            if (process_flac_data(client) != WICED_SUCCESS)
            {
                break;
            }
        }
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_player FLAC decoder exit\n");

    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE);

    WICED_END_OF_CURRENT_THREAD();
}


wiced_result_t audio_client_flac_decoder_start(audio_client_t* client)
{
    flac_decoder_t* dec;
    wiced_result_t result;

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_flac_decoder_start: Bad ARG\n");
        return WICED_BADARG;
    }

    /*
     * Allocate the internal decoder structure.
     */

    dec = (flac_decoder_t*)calloc(1, sizeof(flac_decoder_t));
    if (dec == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate decoder structure\n");
        return WICED_ERROR;
    }

    /* initialize a FLAC decoder instance */
    dec->flac_decoder = FLAC__stream_decoder_new();
    if (dec->flac_decoder == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create new FLAC decoder\n");
        free(dec);
        return WICED_ERROR;
    }

    client->decoder_handle = dec;
    dec->tag = FLAC_TAG_VALID;

    /* Start decoder thread */
    client->decoder_done = WICED_FALSE;
    result = wiced_rtos_create_thread_with_stack(&client->decoder_thread, AUDIO_CLIENT_DECODER_FLAC_THREAD_PRIORITY, "FLAC Decoder",
                                                 audio_client_flac_thread, dec->stack, AUDIO_CLIENT_DECODER_FLAC_STACK_SIZE, client);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_flac_decoder_start: wiced_rtos_create_thread failed %d\n", result);
        return WICED_ERROR;
    }
    else
    {
        client->decoder_thread_ptr = &client->decoder_thread;
    }

    return result;
}


wiced_result_t audio_client_flac_decoder_stop(audio_client_t* client)
{
    flac_decoder_t* dec;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_flac_decoder_stop\n");

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_flac_decoder_stop: Bad ARG\n");
        return WICED_BADARG;
    }

    if (client->decoder_thread_ptr != NULL)
    {
        client->decoder_done = WICED_TRUE;
        wiced_rtos_thread_force_awake(&client->decoder_thread);
        wiced_rtos_thread_join(&client->decoder_thread);
        wiced_rtos_delete_thread(&client->decoder_thread);
        client->decoder_thread_ptr = NULL;

        if (client->decoder_handle)
        {
            dec = (flac_decoder_t*)client->decoder_handle;
            if (dec->flac_decoder != NULL)
            {
                FLAC__stream_decoder_delete(dec->flac_decoder);
                dec->flac_decoder = NULL;
            }

            if (dec->flac_metadata != NULL)
            {
                free(dec->flac_metadata);
                dec->flac_metadata = NULL;
            }

            dec->tag = FLAC_TAG_INVALID;
            free(client->decoder_handle);
            client->decoder_handle = NULL;
        }
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_client_flac_decoder_ioctl(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg)
{
    flac_decoder_t* dec;
    audio_client_stream_info_t* info;
    double percentage;
    double seek_ms;
    uint32_t position;
    uint32_t sample;
    uint32_t events;

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_flac_decoder_ioctl: Bad handle\n");
        return WICED_BADARG;
    }

    dec = (flac_decoder_t*)client->decoder_handle;
    if (dec == NULL || dec->tag != FLAC_TAG_VALID)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_flac_decoder_ioctl: Bad decoder handle\n");
        return WICED_BADARG;
    }

    switch (ioctl)
    {
        case DECODER_IOCTL_INFO:
            info = (audio_client_stream_info_t*)arg;
            if (info != NULL)
            {
                info->stream_total_samples  = dec->stream_total_samples;
                info->stream_current_sample = dec->stream_current_sample;
                info->stream_sample_rate    = dec->stream_sample_rate;
                info->stream_channels       = dec->stream_num_channels;
                info->stream_bps            = dec->stream_bits_per_sample;
            }
            break;

        case DECODER_IOCTL_GET_SEEK_POSITION:
            seek_ms = *((uint32_t*)arg);

            /*
             * Convert the time to the corresponding sample in the audio stream.
             */

            sample = (uint32_t)((double)seek_ms * ((double)dec->stream_sample_rate / MILLISECOND_PER_SECOND_FP));

            /*
             * Figure out how far through the stream we are.
             */

            percentage = (double)sample / (double)dec->stream_total_samples;

            /*
             * And convert the percentage to a byte position in the stream.
             */

            position = ( uint32_t )( (double) client->http_params.http_total_content_length * percentage );

            if ( position > client->http_params.http_total_content_length )
            {
                position = client->http_params.http_total_content_length;
            }

            *((uint32_t*)arg) = position;
            break;

        case DECODER_IOCTL_SET_POSITION:
            /*
             * We'll pick up the new position automatically in the decoder output.
             */

            dec->audio_frames_offset      = 0;
            dec->wait_for_stream_position = WICED_FALSE;
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

        default:
            return WICED_UNSUPPORTED;
    }

    return WICED_SUCCESS;
}
