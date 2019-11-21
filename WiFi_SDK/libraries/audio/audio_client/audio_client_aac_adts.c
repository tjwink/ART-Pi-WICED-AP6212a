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
 * Audio Client AAC ADTS decoding support
 *
 */

#include "wiced_result.h"
#include "wiced_platform.h"
#include "wiced_log.h"

#include "audio_client_private.h"
#include "audio_client_aac_private.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define ADTS_SEARCH_COMPLETE_FLAGS  (SEARCH_ADTS_SYNCWORD_FOUND)
#define ADTS_SEARCH_COMPLETE(dec)   ((dec->search_status & ADTS_SEARCH_COMPLETE_FLAGS) == ADTS_SEARCH_COMPLETE_FLAGS)

/******************************************************
 *                    Constants
 ******************************************************/

#define ADTS_HDR_ID_MPEG2           ( 1 )
#define ADTS_HDR_ID_MPEG4           ( 0 )

/*
 * Search flags
 */

#define SEARCH_ADTS_SYNCWORD_FOUND  ((uint32_t)(1 <<  0))

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

wiced_result_t adts_get_header(audio_client_t* client, aac_decoder_t* dec)
{
    data_buf_t* dbuf;
    data_buf_t* dptr;
    uint32_t begin_idx;
    uint32_t buf_ridx;
    uint32_t i;

    /*
     * Search for the sync word is complicated by the fact that it may span
     * multiple input buffers. And we don't want to advance past the sync word
     * if we find it.
     */

    buf_ridx  = client->data_buf_ridx;
    begin_idx = 0;

    dec->adts_header_idx = 0;
    while (dec->adts_header_idx < ADTS_HEADER_LEN)
    {
        dbuf = &client->data_bufs[buf_ridx];
        if (!dbuf->inuse)
        {
            return WICED_SUCCESS;
        }

        for (i = dbuf->bufused; i < dbuf->buflen && dec->adts_header_idx < ADTS_HEADER_LEN; i++)
        {
            /*
             * Check for the beginning of the sync word
             */

            if (dec->adts_header_idx == 0 && dbuf->buf[i] != 0xFF)
            {
                continue;
            }
            else if (dec->adts_header_idx == 1 && (dbuf->buf[i] & 0xF0) != 0xF0)
            {
                dec->adts_header_idx = 0;
                if (i == 0)
                {
                    /*
                     * Special case where the last character of the previous buffer
                     * is 0xFF. We need to release the previous buffer.
                     */

                    dptr = &client->data_bufs[client->data_buf_ridx];
                    if (client->data_buf_ridx == buf_ridx || !dptr->inuse)
                    {
                        /*
                         * Something is wrong!!!!!!!
                         */

                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Corrupted buffers\n");
                        return WICED_ERROR;
                    }
                    aac_release_buffer(client, dptr);
                }
                continue;
            }

            if (dec->adts_header_idx == 0)
            {
                begin_idx = i;
            }

            dec->adts_header[dec->adts_header_idx] = dbuf->buf[i];
            dec->adts_header_idx++;
        }

        if (dec->adts_header_idx == 0)
        {
            /*
             * Got to the end of the buffer without finding the sync word.
             * Release the buffer.
             */

            aac_release_buffer(client, dbuf);
            buf_ridx = client->data_buf_ridx;
        }
        else if (dec->adts_header_idx < ADTS_HEADER_LEN)
        {
            /*
             * We finished the buffer and have a partial header.
             * Mark where the header started in this buffer.
             */

            if (buf_ridx == client->data_buf_ridx)
            {
                dbuf->bufused = begin_idx;
            }
            buf_ridx++;
        }
    }

    return WICED_SUCCESS;
}


wiced_result_t adts_find_frame(audio_client_t* client, aac_decoder_t* dec)
{
    adts_hdr_t adts_hdr;

    /*
     * See if the ADTS header is lurking in our buffers.
     */

    if (adts_get_header(client, dec) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /*
     * Have we found a complete header?
     */

    if (dec->adts_header_idx < ADTS_HEADER_LEN)
    {
        return WICED_SUCCESS;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "ADTS Sync word bytes: %02x%02x%02x%02x %02x%02x%02x\n",
            dec->adts_header[0], dec->adts_header[1], dec->adts_header[2], dec->adts_header[3],
            dec->adts_header[4], dec->adts_header[5], dec->adts_header[6]);
    /*
     * We have a header
     */

    adts_hdr.fixed.id                                 = (dec->adts_header[1] >> 3) & 0x01;
    adts_hdr.fixed.layer                              = (dec->adts_header[1] >> 1) & 0x03;
    adts_hdr.fixed.protection_absent                  = (dec->adts_header[1] >> 0) & 0x01;
    adts_hdr.fixed.profile                            = (dec->adts_header[2] >> 6);

    adts_hdr.fixed.sampling_frequency_idx             = (dec->adts_header[2] >> 2) & 0x0F;
    adts_hdr.fixed.private_bit                        = (dec->adts_header[2] >> 1) & 0x01;
    adts_hdr.fixed.channel_configuration              = ((dec->adts_header[2] & 0x01) << 2) | (dec->adts_header[3] >> 6);
    adts_hdr.fixed.original_copy                      = (dec->adts_header[3] >> 5) & 0x01;
    adts_hdr.fixed.home                               = (dec->adts_header[3] >> 4) & 0x01;

    adts_hdr.variable.copyright_identification_bit    = (dec->adts_header[3] >> 3) & 0x01;
    adts_hdr.variable.copyright_identification_start  = (dec->adts_header[3] >> 2) & 0x01;
    adts_hdr.variable.aac_frame_length                = (dec->adts_header[3] & 0x03) << 11 | dec->adts_header[4] << 3 | dec->adts_header[5] >> 5;
    adts_hdr.variable.adts_buffer_fullness            = (dec->adts_header[5] & 0x1F) <<  6 | dec->adts_header[6] >> 2;
    adts_hdr.variable.num_raw_data_blocks_in_frame    = (dec->adts_header[6]) & 0x03;

    /*
     * Sanity checks on the header
     */

    if (adts_hdr.fixed.layer != 0)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: wrong layer in the header\n");
        return WICED_ERROR;
    }

    if (adts_hdr.fixed.sampling_frequency_idx >= ADTS_HDR_SAMPLING_RATE_MAX)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: invalid sampling frequency (%d)\n", adts_hdr.fixed.sampling_frequency_idx);
        return WICED_ERROR;
    }

    if (adts_hdr.fixed.profile == ADTS_HDR_AAC_LTP && adts_hdr.fixed.id == ADTS_HDR_ID_MPEG2)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: invalid ID/PROFILE match (MPEG-2 with LTP)\n");
        return WICED_ERROR;
    }

    dec->search_status |= SEARCH_ADTS_SYNCWORD_FOUND;

    return WICED_SUCCESS;
}


wiced_result_t adts_process_data(audio_client_t* client)
{
    aac_decoder_t*      dec;
    data_buf_t*         dbuf;
    aac_pcm_info_t      pcm_info;
    CStreamInfo*        info;
    uint32_t            bytes_avail;
    uint32_t            free_bytes;
    uint32_t            fdk_valid;
    uint8_t*            data_ptr;
    AAC_DECODER_ERROR   fdk_err;

    dec = (aac_decoder_t*)client->decoder_handle;

    /*
     * Have we found the header yet?
     */

    if (!ADTS_SEARCH_COMPLETE(dec))
    {
        if ((adts_find_frame(client, dec)) != WICED_SUCCESS)
        {
            return WICED_ERROR;
        }

        if (!ADTS_SEARCH_COMPLETE(dec))
        {
            return WICED_SUCCESS;
        }
    }

    /*
     * Push this buffer to the decoder.
     */

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (!dbuf->inuse)
    {
        return WICED_SUCCESS;
    }

    bytes_avail = dbuf->buflen - dbuf->bufused;
    fdk_valid   = bytes_avail;
    data_ptr    = &dbuf->buf[dbuf->bufused];
    fdk_err     = aacDecoder_Fill(dec->fdk_handle, &data_ptr, (const UINT*)&bytes_avail, (UINT*)&fdk_valid);
    if (fdk_err != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ATDS: Fill error %lx\n", fdk_err);
        return WICED_ERROR;
    }

    if (fdk_valid == 0)
    {
        aac_release_buffer(client, dbuf);
    }
    else
    {
        dbuf->bufused = dbuf->buflen - fdk_valid;
    }

    /*
     * Now decode the data.
     */

    fdk_err = aacDecoder_GetFreeBytes(dec->fdk_handle, (UINT*)&free_bytes);
    if (fdk_err != AAC_DEC_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: Unable to get decoder free bytes: %lx\n", fdk_err);
        return WICED_ERROR;
    }

    while (!dec->audio_client->decoder_done && free_bytes < dec->fdk_internal_buf_size)
    {
        fdk_err = aacDecoder_DecodeFrame(dec->fdk_handle, (INT_PCM*)dec->data_decode, DECODE_BUF_SIZE, 0);
        if (fdk_err != AAC_DEC_OK)
        {
            if (fdk_err == AAC_DEC_NOT_ENOUGH_BITS || fdk_err == AAC_DEC_TRANSPORT_SYNC_ERROR)
            {
                /*
                 * Not fatal.
                 */

                return WICED_SUCCESS;
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: Decode failed: %lx\n", fdk_err);
            return WICED_ERROR;
        }

        /*
         * Get the current stream info
         */

        info = aacDecoder_GetStreamInfo(dec->fdk_handle);
        if (!info || info->sampleRate <= 0)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: No stream info\n");
            return WICED_ERROR;
        }

        /* always rebuild the pcm info because with AAC+v2 streams */
        /* we might have SBR info later in the stream and so */
        /* chnum and sr might change while playing the stream */
        /* note: this is possible especially with webradio streams */

        /* our FDK only sends s16LE output */
        pcm_info.bps      = 16;
        pcm_info.cbps     = 16;
        pcm_info.chnum    = info->numChannels;
        pcm_info.sr       = info->sampleRate;

        aac_pcm_output_push(dec, &pcm_info, dec->data_decode, info->frameSize * info->numChannels);

        fdk_err = aacDecoder_GetFreeBytes(dec->fdk_handle, (UINT*)&free_bytes);
        if (fdk_err != AAC_DEC_OK)
        {
            if (!dec->audio_client->decoder_done)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "ADTS: Unable to get decoder free bytes: %lx\n", fdk_err);
            }
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}
