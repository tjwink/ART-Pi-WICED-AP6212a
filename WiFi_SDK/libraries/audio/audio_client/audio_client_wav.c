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

/** @file Audio Client WAV Decode Routines
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_log.h"
#include "wwd_assert.h"

#include "audio_client_private.h"
#include "audio_client_wav.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifndef le16toh
#define le16toh(x)      (x)
#endif

#ifndef le32toh
#define le32toh(x)      (x)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

#define WAV_TAG_VALID               0x61EDBA16
#define WAV_TAG_INVALID             0xDEADBEEF

#define WAV_FORMAT_PCM              (0x0001)    /* Microsoft Pulse Code Modulation (PCM) format */
#define WAV_FORMAT_PCM_FLOAT        (0x0003)    /* PCM format, values are floats                */
#define WAV_FORMAT_EXTENSIBLE       (0xFFFE)    /* Extended data format                         */
#define IBM_FORMAT_MULAW            (0x0101)    /* IBM mu-law format                            */
#define IBM_FORMAT_ALAW             (0x0102)    /* IBM a-law format                             */
#define IBM_FORMAT_ADPCM            (0x0103)    /* IBM AVC Adaptive                             */

#define WAV_FMT_CHUNKID_OFFSET      (12)
#define WAV_FMT_CHUNKSIZE_OFFSET    (16)
#define WAV_FMT_DATA_OFFSET         (20)

#define AUDIO_CLIENT_DECODER_WAV_THREAD_PRIORITY       (WICED_DEFAULT_LIBRARY_PRIORITY)
#define AUDIO_CLIENT_DECODER_WAV_STACK_SIZE            (8 * 1024)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t        tag;

    wiced_bool_t    header_parsed;
    wiced_bool_t    data_chunk_found;
    uint64_t        bytes_processed;
    uint32_t        chunk_remaining;            /* Bytes remaining in chunk being skipped   */

    uint32_t        data_chunk_size;
    uint32_t        data_chunk_offset;          /* Offset of start of data chunk in stream  */

    int             leftover_audio_bytes;
    uint8_t         leftover_audio_data[8];

    wiced_bool_t    wait_for_stream_position;

    /*
     *  Audio format information.
     */

    uint16_t        num_channels;
    uint32_t        sample_rate;
    uint32_t        byte_rate;
    uint16_t        block_align;
    uint16_t        bits_per_sample;

    uint16_t        render_bits_per_sample;

    /*
     * Some housekeeping variables.
     */

    uint32_t        audio_frames_total;
    uint32_t        audio_frames_played;
    uint32_t        audio_frames_offset;

    uint8_t         mono_stereo_offs;

    uint8_t         stack[AUDIO_CLIENT_DECODER_WAV_STACK_SIZE];
} wav_decoder_t;

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

static wiced_result_t find_data_chunk(wav_decoder_t* dec, data_buf_t* dbuf)
{
    uint8_t* ptr;
    int chunk_size;

    while (dbuf->bufused < dbuf->buflen)
    {
        if (dec->chunk_remaining > 0)
        {
            if (dbuf->buflen - dbuf->bufused > dec->chunk_remaining)
            {
                dbuf->bufused += dec->chunk_remaining;
                dec->chunk_remaining = 0;
            }
            else
            {
                dec->chunk_remaining -= (dbuf->buflen - dbuf->bufused);
                dbuf->bufused = dbuf->buflen;
            }
        }

        if (dbuf->bufused < dbuf->buflen)
        {
            if (dbuf->buflen - dbuf->bufused < 8)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Parsing chunk header across packet boundaries not implemented\n");
                return WICED_ERROR;
            }

            ptr = &dbuf->buf[dbuf->bufused];

            if (ptr[0] == 'd' && ptr[1] == 'a' && ptr[2] == 't' && ptr[3] == 'a')
            {
                dec->data_chunk_size  = le32toh(*((uint32_t*)&ptr[4]));
                dec->data_chunk_found = WICED_TRUE;
                dbuf->bufused += 8;

                dec->audio_frames_total = dec->data_chunk_size / dec->block_align;

                /*
                 * We need to keep track of where the audio data starts in case we
                 * need to seek later.
                 */

                dec->data_chunk_offset = dec->bytes_processed + dbuf->bufused;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Data chunk found at offset %lu\n", dec->data_chunk_offset);
                break;
            }
            else
            {
                chunk_size = le32toh(*((uint32_t*)&ptr[4])) + 8;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Skipping chunk: %4.4s, size %d\n", ptr, chunk_size - 8);

                if (dbuf->bufused + chunk_size < dbuf->buflen)
                {
                    dbuf->bufused += chunk_size;
                }
                else
                {
                    dec->chunk_remaining = chunk_size - (dbuf->buflen - dbuf->bufused);
                    dbuf->bufused = dbuf->buflen;
                }
            }
        }
    }

    return WICED_SUCCESS;
}


static wiced_result_t parse_wav_header(wav_decoder_t* dec, data_buf_t* dbuf)
{
    uint8_t* ptr;
    uint16_t wav_format;
    int fmt_chunk_size;
    int offset;

    ptr = dbuf->buf;

    if (dbuf->buflen < WAV_FMT_DATA_OFFSET + 20)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "First packet not large enough to contain WAV header\n");
        return WICED_ERROR;
    }

    /*
     * Verify the RIFF header bytes.
     */

    if (ptr[0] != 'R' || ptr[1] != 'I' || ptr[2] != 'F' || ptr[3] != 'F')
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No RIFF header\n");
        return WICED_ERROR;
    }

    /*
     * Verify the WAVE format.
     */

    if (ptr[8] != 'W' || ptr[9] != 'A' || ptr[10] != 'V' || ptr[11] != 'E')
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No WAVE format\n");
        return WICED_ERROR;
    }

    /*
     * Verify the format chunk and get the chunk size.
     */

    if (ptr[WAV_FMT_CHUNKID_OFFSET]     != 'f' || ptr[WAV_FMT_CHUNKID_OFFSET + 1] != 'm' ||
        ptr[WAV_FMT_CHUNKID_OFFSET + 2] != 't' || ptr[WAV_FMT_CHUNKID_OFFSET + 3] != ' ')
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No fmt chunk: %4.4s\n", &ptr[WAV_FMT_CHUNKID_OFFSET]);
        return WICED_ERROR;
    }

    fmt_chunk_size = le32toh(*((uint32_t*)&ptr[WAV_FMT_CHUNKSIZE_OFFSET]));

    /*
     * Extract the audio format information.
     */

    wav_format           = le16toh(*((uint16_t*)&ptr[WAV_FMT_DATA_OFFSET]));
    dec->num_channels    = le16toh(*((uint16_t*)&ptr[WAV_FMT_DATA_OFFSET + 2]));
    dec->sample_rate     = le32toh(*((uint32_t*)&ptr[WAV_FMT_DATA_OFFSET + 4]));
    dec->byte_rate       = le32toh(*((uint32_t*)&ptr[WAV_FMT_DATA_OFFSET + 8]));
    dec->block_align     = le16toh(*((uint16_t*)&ptr[WAV_FMT_DATA_OFFSET + 12]));
    dec->bits_per_sample = le16toh(*((uint16_t*)&ptr[WAV_FMT_DATA_OFFSET + 14]));

    if (wav_format != WAV_FORMAT_PCM && wav_format != WAV_FORMAT_EXTENSIBLE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unsupported wav file format: 0x%08x\n", wav_format);
        return WICED_ERROR;
    }

    /*
     * Verify that we can handle the format.
     */

    if ((dec->num_channels == 0) || (dec->num_channels > 2))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%d channels not supported at this time\n", dec->num_channels);
        return WICED_ERROR;
    }

    if (dec->sample_rate != 44100 && dec->sample_rate != 48000 && dec->sample_rate != 88200 &&
        dec->sample_rate != 96000 && dec->sample_rate != 176400  && dec->sample_rate != 192000 &&
        dec->sample_rate != 8000)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Sample rate of %d not supported at this time\n", dec->sample_rate);
        return WICED_ERROR;
    }

    if (dec->bits_per_sample != 8 && dec->bits_per_sample != 16 && dec->bits_per_sample != 24 && dec->bits_per_sample != 32)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bits per sample of %d not supported at this time\n", dec->bits_per_sample);
        return WICED_ERROR;
    }

    dec->render_bits_per_sample = dec->bits_per_sample == 24 ? 32 : dec->bits_per_sample;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Wav file format:\n");
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    audio_format: 0x%04x\n", wav_format);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    num channels: %u\n", dec->num_channels);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, " bits per sample: %u\n", dec->bits_per_sample);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "     sample rate: %lu\n", dec->sample_rate);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "     block align: %u\n", dec->block_align);

    /*
     * Advance past the fmt chunk.
     */

    offset = WAV_FMT_DATA_OFFSET + fmt_chunk_size;
    if (offset < dbuf->buflen)
    {
        dbuf->bufused = offset;
    }
    else
    {
        dbuf->bufused = dbuf->buflen;
        dec->chunk_remaining = WAV_FMT_DATA_OFFSET + fmt_chunk_size - dbuf->buflen;
    }

    return WICED_SUCCESS;
}

static void pcm_mono_to_stereo( uint8_t *dest, uint8_t *src, const uint8_t bit_per_sample_in, const uint8_t bit_per_sample_out,
                                const uint32_t num_of_bytes, uint8_t *offs )
{
    int32_t k = 0;
    int32_t i = 0;
    int32_t j = 0;

    if(NULL==offs)
        return;

    /*  bit_per_sample_out:
     *  to be used for bit-per-sample conversion like 16->24
     */

    uint8_t frame_size_in  = (bit_per_sample_in >> 3);
    uint8_t frame_size_out = (bit_per_sample_out >> 3);

    uint8_t frame_offs = (*offs)%frame_size_in;

    /* loop over data bytes */
    for( k = 0; k < num_of_bytes; k++ )
    {
        /* audio frame index */
        i = (k+frame_offs) / frame_size_in;
        /* audio frame byte index */
        j = (k+frame_offs) - i * frame_size_in;

        /* clear if we have bps conversion:                         */
        /* we only need to handle 24->32 so keep this memclr simple */
        /* and expecially do not clear what will be overwritten..   */
        /* only the padding bytes                                   */
        if( frame_size_out >  frame_size_in )
        {
            if(j==0)
            {
                *(dest + (i * frame_size_out *2) + 0              ) = 0;
                *(dest + (i * frame_size_out *2) + frame_size_out ) = 0;
            }
            /* shift one byte */
            j++;
        }

        /* scatter interleave */
        *(dest + (i * frame_size_out *2) + j + 0              ) = *(src +k);
        *(dest + (i * frame_size_out *2) + j + frame_size_out ) = *(src +k);
    }

    *offs = (num_of_bytes + frame_offs) % frame_size_in;
}


static wiced_result_t process_pcm_output(audio_client_t* client, wav_decoder_t* dec, data_buf_t* dbuf)
{
    audio_client_buf_t audio_buf;
    wiced_result_t result;
    uint8_t* inptr;
    uint8_t* outptr;
    int frames_to_copy = 0;
    int output_framesize = 0;
    int num_frames = 0;
    int num_frames_adj = 0;
    int num_frames_residual_bytes= 0;
    int frames = 0;
    int size = 0;
    int old;
    int new;
    uint8_t num_channels_adj = 0;
    uint8_t leftover_audio_bytes_adj = 0;
    audio_client_buf_t* audio_buf_ptr = NULL;

    inptr = &dbuf->buf[dbuf->bufused];

    num_channels_adj = (dec->num_channels >= 2) ? 0 : 1;

    num_frames = (dbuf->buflen - dbuf->bufused + dec->leftover_audio_bytes) / dec->block_align;

    if ( dec->leftover_audio_bytes % dec->block_align == 0 )
    {
        /* input stream is block aligned */
        num_frames_adj = 0;
        num_frames_residual_bytes = 0;
    }
    else
    {
        /* input stream is NOT block aligned, use one frame slot for padding to keep alignement*/
        num_frames_adj = 1;
        num_frames    -= num_frames_adj;
        num_frames_residual_bytes = dec->block_align - dec->leftover_audio_bytes;
    }

    output_framesize = (dec->render_bits_per_sample * (dec->num_channels << num_channels_adj) ) / 8;

    while ( (num_frames-num_frames_adj) > 0 && client->decoder_done == WICED_FALSE)
    {
        result = audio_client_buffer_get(client, &audio_buf, AUDIO_CLIENT_AUDIO_BUF_TIMEOUT);
        if (result != WICED_SUCCESS)
        {
            /* we probably didn't get a buffer because the buffers are full */
            continue;
        }
        /* keep a reference to audio_buf */
        audio_buf_ptr = &audio_buf;

        if ( (output_framesize * num_frames + dec->leftover_audio_bytes) <= audio_buf.buflen)
        {
            frames_to_copy = num_frames;
            frames_to_copy -= num_frames_adj;
        }
        else
        {
            num_frames_adj = (dec->leftover_audio_bytes % dec->block_align == 0 ) ? 0 : 1;
            frames_to_copy = audio_buf.buflen / output_framesize;
            frames_to_copy -= num_frames_adj;
        }

        /* reset */
        num_frames_adj = 0;

        outptr = audio_buf.buf;

        switch (dec->bits_per_sample)
        {
            case 8:
            case 16:
            case 32:
                if (dec->leftover_audio_bytes != 0)
                {
                    if(num_channels_adj == 0)
                    {
                        memcpy(outptr, dec->leftover_audio_data, dec->leftover_audio_bytes);
                        outptr += dec->leftover_audio_bytes;
                        dec->leftover_audio_bytes = 0;
                    }
                    else
                    {
                        pcm_mono_to_stereo( outptr, dec->leftover_audio_data,
                                            dec->bits_per_sample, dec->bits_per_sample,
                                            dec->leftover_audio_bytes, &dec->mono_stereo_offs );

                        outptr += (dec->leftover_audio_bytes / dec->block_align) *output_framesize;

                        inptr         += leftover_audio_bytes_adj ;
                        dbuf->bufused += leftover_audio_bytes_adj ;

                        num_frames_adj=1;
                    }
                }

                size = (frames_to_copy * output_framesize);

                if(num_channels_adj == 0)
                {
                    memcpy(outptr, inptr, size);
                }
                else
                {
                    pcm_mono_to_stereo( outptr, inptr,
                                        dec->bits_per_sample, dec->bits_per_sample,
                                        (size>> num_channels_adj) + num_frames_residual_bytes, &dec->mono_stereo_offs );
                }

                outptr        += size + ((dec->leftover_audio_bytes+num_frames_residual_bytes)/ dec->block_align) *output_framesize;
                inptr         += (size >> num_channels_adj) + num_frames_residual_bytes;
                dbuf->bufused += (size >> num_channels_adj) + num_frames_residual_bytes;
                break;

            case 24:
                frames = frames_to_copy;                    /* handle filling the frame after leftover bytes used */

                if (dec->leftover_audio_bytes)
                {
                    if(num_channels_adj == 0)
                    {
                        *outptr++ = 0;
                        for (old = 0; old < dec->leftover_audio_bytes; old++)
                        {
                            if (old == 3)
                            {
                                *outptr++ = 0;
                            }
                            *outptr++ = dec->leftover_audio_data[old];
                        }

                        for (new = dec->leftover_audio_bytes; new < 6; new++)
                        {
                            if (new == 3)
                            {
                                *outptr++ = 0;
                            }
                            *outptr++ = *inptr++;
                        }

                        dec->leftover_audio_bytes = 0;

                        /* The frames number is already adjusted for leftovers
                           during the initial computation */
                        //frames--;
                    }
                    else
                    {
                        pcm_mono_to_stereo( outptr, dec->leftover_audio_data,
                                            24, 32,
                                            dec->leftover_audio_bytes, &dec->mono_stereo_offs );

                        outptr += (dec->leftover_audio_bytes / dec->block_align) *output_framesize;

                        inptr         += leftover_audio_bytes_adj ;
                        dbuf->bufused += leftover_audio_bytes_adj ;

                        num_frames_adj=1;
                    }
                }

                /* We have handled the leftover bytes and finished a frame (both channels) with them.
                 * Now copy over the rest of the audio.
                 */

                if(num_channels_adj == 0)
                {
                    while (frames > 0)
                    {
                        /* ch0 */
                        *outptr++ = 0;
                        *outptr++ = *inptr++;
                        *outptr++ = *inptr++;
                        *outptr++ = *inptr++;

                        /* ch1 */
                        *outptr++ = 0;
                        *outptr++ = *inptr++;
                        *outptr++ = *inptr++;
                        *outptr++ = *inptr++;

                        frames--;
                    }
                }
                else
                {
                    pcm_mono_to_stereo( outptr, inptr,
                                        24, 32,
                                        (frames *dec->block_align) + num_frames_residual_bytes, &dec->mono_stereo_offs );

                    outptr        += (frames *output_framesize) + ((dec->leftover_audio_bytes+num_frames_residual_bytes)/ dec->block_align) *output_framesize;
                    inptr         += (frames *dec->block_align) + num_frames_residual_bytes;
                    dbuf->bufused += (frames *dec->block_align) + num_frames_residual_bytes;
                }

                break;

            default:
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Decoder corruption\n");
                return WICED_ERROR;
        }

        dec->audio_frames_played += frames_to_copy;
        num_frames -= frames_to_copy;
        dbuf->bufused = (int)inptr - (int)dbuf->buf ;

        audio_buf.offset = 0;
        audio_buf.curlen = (uint32_t)outptr - (uint32_t)audio_buf.buf;

        /*
         * Are we starting playback somewhere other than the beginning?
         */

        if (client->start_offset_ms > 0 && dec->audio_frames_offset)
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

        while (client->decoder_done == WICED_FALSE)
        {
            result = audio_client_buffer_release(client, &audio_buf);
            if (result == WICED_SUCCESS)
            {
                /* audio_buf has been released; drop the reference */
                audio_buf_ptr = NULL;
                break;
            }
            if (result == WICED_BADARG)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error pushing audio buffer\n");
                return WICED_ERROR;
            }
        }

        leftover_audio_bytes_adj = dec->leftover_audio_bytes ;
    }

    /* we still have a reference to audio_buf; we need to release it */
    if (audio_buf_ptr != NULL)
    {
        if (audio_client_buffer_release(client, audio_buf_ptr) == WICED_SUCCESS)
        {
            audio_buf_ptr = NULL;
        }
    }

    if (client->decoder_done == WICED_FALSE && dbuf->bufused < dbuf->buflen)
    {
        dec->leftover_audio_bytes = dbuf->buflen - dbuf->bufused;

        if (dec->leftover_audio_bytes <= sizeof(dec->leftover_audio_data))
        {
            for (old = 0; old < dec->leftover_audio_bytes; old++)
            {
                dec->leftover_audio_data[old] = *inptr++;
            }

            dbuf->bufused = dbuf->buflen;
        }
        else
        {
            /*
             * This shouldn't happen.
             */

            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Leftover audio bytes out of bounds (%d)\n", dec->leftover_audio_bytes);
        }
    }
    else
    {
        dec->leftover_audio_bytes = 0;
    }

    return WICED_SUCCESS;
}


static wiced_result_t process_wav_data(audio_client_t* client)
{
    wiced_audio_config_t audio_config;
    audio_client_buf_t audio_buf;
    wav_decoder_t* dec;
    data_buf_t*    dbuf;

    dec = (wav_decoder_t*)client->decoder_handle;

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

    if (dec->wait_for_stream_position)
    {
        goto _toss_buffer;
    }

    /*
     * Are we looking for the header?
     */

    if (dec->header_parsed == WICED_FALSE)
    {
        if (parse_wav_header(dec, dbuf) != WICED_SUCCESS)
        {
            client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        dec->header_parsed = WICED_TRUE;
        wiced_time_get_time(&client->play_start_time);

        /*
         * Tell audio render about the audio format.
         */

        audio_config.bits_per_sample = dec->render_bits_per_sample;

        if (dec->num_channels==1)
        {
            /* we handle mono->stereo duplication in the pcm processing */
            audio_config.channels = 2;
        }
        else
        {
            audio_config.channels    = dec->num_channels;
        }

        audio_config.sample_rate     = dec->sample_rate;
        audio_config.frame_size      = dec->render_bits_per_sample * audio_config.channels / 8;
        audio_config.volume          = client->params.volume;

        if (audio_client_audio_config(client, &audio_config) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio config event failed\n");
            client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        if (client->start_offset_ms > 0)
        {
            dec->audio_frames_offset = (uint32_t)((float)dec->sample_rate * (float)client->start_offset_ms / 1000.0);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Starting frame offset is %lu (time %lu)\n", dec->audio_frames_offset, client->start_offset_ms);
        }
        else
        {
            dec->audio_frames_offset = 0;
        }
    }

    /*
     * Have we found the data chunk yet?
     */

    if (dec->data_chunk_found == WICED_FALSE)
    {
        if (find_data_chunk(dec, dbuf) != WICED_SUCCESS)
        {
            client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }

        if (dec->data_chunk_found && client->start_offset_ms > 0)
        {
            /*
             * We need to set the wait for stream position flag first. Since the HTTP thread
             * runs at a higher priority, it's possible for it to make a new connection and
             * set the stream position in response to a seek request before we get a chance
             * to run again.
             */

            dec->wait_for_stream_position = WICED_TRUE;
            if (audio_client_check_start_offset_seek(client))
            {
                /*
                 * A seek operation was started. Don't output the current buffer.
                 */

                goto _toss_buffer;
            }
            dec->wait_for_stream_position = WICED_FALSE;
        }
    }

    if (dec->data_chunk_found && dbuf->bufused < dbuf->buflen)
    {
        if (process_pcm_output(client, dec, dbuf) != WICED_SUCCESS)
        {
            client->decoder_done = WICED_TRUE;
            return WICED_ERROR;
        }
    }

    dec->bytes_processed += dbuf->buflen;

  _toss_buffer:
    /*
     * This buffer is done. Advance to the next one.
     */

    RELEASE_DBUF(client, dbuf);

    CHECK_FOR_THRESHOLD_LOW_EVENT(client);

    /*
     * More data waiting to be processed?
     */

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (dbuf->inuse)
    {
        wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
    }

    return WICED_SUCCESS;
}


static wiced_result_t flush_wav_data(audio_client_t* client)
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


static void audio_client_wav_thread(uint32_t arg)
{
    audio_client_t*     client = (audio_client_t*)arg;
    uint32_t            events;
    wiced_result_t      result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client wav decoder start\n");

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
            flush_wav_data(client);
        }

        if (events & DECODER_EVENT_AUDIO_DATA)
        {
            if (process_wav_data(client) != WICED_SUCCESS)
            {
                break;
            }
        }
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client wav decoder exit\n");

    wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE);

    WICED_END_OF_CURRENT_THREAD();
}


wiced_result_t audio_client_wav_decoder_start(audio_client_t* client)
{
    wav_decoder_t* dec;
    wiced_result_t result;

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_wav_decoder_start: Bad ARG\n");
        return WICED_BADARG;
    }

    /*
     * Allocate the internal decoder structure.
     */

    dec = (wav_decoder_t*)calloc(1, sizeof(wav_decoder_t));
    if (dec == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate decoder structure\n");
        return WICED_ERROR;
    }

    client->decoder_handle = dec;
    dec->tag = WAV_TAG_VALID;

    dec->leftover_audio_bytes = 0;
    dec->mono_stereo_offs = 0;

    /* Start decoder thread */
    client->decoder_done = WICED_FALSE;
    result = wiced_rtos_create_thread_with_stack(&client->decoder_thread, AUDIO_CLIENT_DECODER_WAV_THREAD_PRIORITY, "Wav Decoder",
                                                 audio_client_wav_thread, dec->stack, AUDIO_CLIENT_DECODER_WAV_STACK_SIZE, client);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_wav_decoder_start: wiced_rtos_create_thread failed %d\n", result);
        return WICED_ERROR;
    }
    else
    {
        client->decoder_thread_ptr = &client->decoder_thread;
    }

    return result;
}


wiced_result_t audio_client_wav_decoder_stop(audio_client_t* client)
{
    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_wav_decoder_stop: Bad ARG\n");
        return WICED_BADARG;
    }

    if (client->decoder_thread_ptr != NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_wav_decoder_stop\n");

        client->decoder_done = WICED_TRUE;
        wiced_rtos_thread_force_awake(&client->decoder_thread);
        wiced_rtos_thread_join(&client->decoder_thread);
        wiced_rtos_delete_thread(&client->decoder_thread);
        client->decoder_thread_ptr = NULL;

        if (client->decoder_handle)
        {
            ((wav_decoder_t*)client->decoder_handle)->tag = WAV_TAG_INVALID;
            free(client->decoder_handle);
            client->decoder_handle = NULL;
        }
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_client_wav_decoder_ioctl(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg)
{
    wav_decoder_t* dec;
    audio_client_stream_info_t* info;
    double seek_ms;
    uint32_t position;
    uint32_t sample;
    uint32_t events;

    if (client == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_wav_decoder_ioctl: Bad handle\n");
        return WICED_BADARG;
    }

    dec = (wav_decoder_t*)client->decoder_handle;
    if (dec == NULL || dec->tag != WAV_TAG_VALID)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_wav_decoder_ioctl: Bad decoder handle\n");
        return WICED_BADARG;
    }

    switch (ioctl)
    {
        case DECODER_IOCTL_INFO:
            info = (audio_client_stream_info_t*)arg;
            if (info != NULL)
            {
                info->stream_total_samples  = dec->audio_frames_total;
                info->stream_current_sample = dec->audio_frames_played;
                info->stream_sample_rate    = dec->sample_rate;
                info->stream_channels       = dec->num_channels;
                info->stream_bps            = dec->bits_per_sample;
            }
            break;

        case DECODER_IOCTL_GET_SEEK_POSITION:
            seek_ms = *((uint32_t*)arg);

            /*
             * Convert the time to the corresponding sample in the audio stream.
             */

            sample = (uint32_t)((double)seek_ms * ((double)dec->sample_rate / 1000.0));

            /*
             * Now convert the sample to a byte position.
             */

            position = sample * ((dec->num_channels * dec->bits_per_sample) >> 0x03);

            /*
             * And add in the offset for the start of the audio data in the wav file.
             */

            position += dec->data_chunk_offset;

            if (position > client->http_params.http_total_content_length)
            {
                position = client->http_params.http_total_content_length;
            }

            *((uint32_t*)arg) = position;
            break;

        case DECODER_IOCTL_SET_POSITION:
            position = (uint32_t)arg;

            /*
             * We are given the byte offset within the stream. We need to adjust for where the audio
             * data started after the RIFF header.
             */

            if (position >= dec->data_chunk_offset)
            {
                position -= dec->data_chunk_offset;
            }
            else
            {
                position = 0;
            }

            /*
             * And convert to samples.
             */

            dec->audio_frames_played      = position / ((dec->render_bits_per_sample * dec->num_channels) / 8);
            dec->audio_frames_offset      = 0;
            dec->leftover_audio_bytes     = 0;
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
