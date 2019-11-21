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
 * Audio Client AAC M4A decoding support
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

#define M4A_SEARCH_COMPLETE_FLAGS       (SEARCH_M4A_STSIZE_COMPLETE | SEARCH_M4A_COFFSET_COMPLETE | SEARCH_M4A_STTS_COMPLETE | SEARCH_M4A_STSC_COMPLETE)
#define M4A_SEARCH_COMPLETE(dec)        ((0 != dec->m4a_audio_specific_info_size) && ((dec->search_status & M4A_SEARCH_COMPLETE_FLAGS) == M4A_SEARCH_COMPLETE_FLAGS))

#define M4A_BOX_HEADER_SIZE(size)       (size == 1 ? 16 : 8)

#define M4A_GET_BOX_SIZE(ptr)           (((ptr)[0] << 24) | ((ptr)[1] << 16) | ((ptr)[2] << 8) | ((ptr)[3]))
#define M4A_GET_BOX_TYPE(ptr)           (((ptr)[3] << 24) | ((ptr)[2] << 16) | ((ptr)[1] << 8) | ((ptr)[0]))

#define M4A_BOX_TYPE(c1,c2,c3,c4)       ((c4 << 24) | (c3 << 16) | (c2 << 8) | (c1))

#define SWAP64(x)                       (((uint64_t)htonl((uint32_t)x)) << 32 | htonl((uint32_t)(x >> 32)))

/******************************************************
 *                    Constants
 ******************************************************/

#define SEARCH_M4A_FTYP_FOUND                       ((uint32_t)(1 <<  0))
#define SEARCH_M4A_FTYP_AUDIO_FOUND                 ((uint32_t)(1 <<  1))
#define SEARCH_M4A_DISCARD_FOUND                    ((uint32_t)(1 <<  2))
#define SEARCH_M4A_DISCARD_COMPLETE                 ((uint32_t)(1 <<  3))
#define SEARCH_M4A_FTYP_ISO_FOUND                   ((uint32_t)(1 <<  4))
#define SEARCH_M4A_MOOV_FOUND                       ((uint32_t)(1 <<  5))
#define SEARCH_M4A_TRAK_FOUND                       ((uint32_t)(1 <<  6))
#define SEARCH_M4A_TRAK_AUDIO_FOUND                 ((uint32_t)(1 <<  7))
#define SEARCH_M4A_TRAK_AAC_FOUND                   ((uint32_t)(1 <<  8))
#define SEARCH_M4A_MDIA_FOUND                       ((uint32_t)(1 <<  9))
#define SEARCH_M4A_MINF_FOUND                       ((uint32_t)(1 << 10))
#define SEARCH_M4A_HDLR_SOUN_FOUND                  ((uint32_t)(1 << 11))
#define SEARCH_M4A_STSD_FOUND                       ((uint32_t)(1 << 12))
#define SEARCH_M4A_DECSPECINFO_FOUND                ((uint32_t)(1 << 13))
#define SEARCH_M4A_STSC_FOUND                       ((uint32_t)(1 << 14))
#define SEARCH_M4A_STSC_COMPLETE                    ((uint32_t)(1 << 15))
#define SEARCH_M4A_STSZ_FOUND                       ((uint32_t)(1 << 16))
#define SEARCH_M4A_STZ2_FOUND                       ((uint32_t)(1 << 17))
#define SEARCH_M4A_STSIZE_COMPLETE                  ((uint32_t)(1 << 18))
#define SEARCH_M4A_STCO_FOUND                       ((uint32_t)(1 << 19))
#define SEARCH_M4A_CO64_FOUND                       ((uint32_t)(1 << 20))
#define SEARCH_M4A_COFFSET_COMPLETE                 ((uint32_t)(1 << 21))
#define SEARCH_M4A_MDAT_FOUND                       ((uint32_t)(1 << 22))
#define SEARCH_M4A_MDAT_COMPLETE                    ((uint32_t)(1 << 23))
#define SEARCH_M4A_STTS_FOUND                       ((uint32_t)(1 << 24))
#define SEARCH_M4A_STTS_COMPLETE                    ((uint32_t)(1 << 25))

#define M4A_BOX_FTYP                                M4A_BOX_TYPE('f','t','y','p')
#define M4A_BOX_MOOV                                M4A_BOX_TYPE('m','o','o','v')
#define M4A_BOX_TRAK                                M4A_BOX_TYPE('t','r','a','k')
#define M4A_BOX_MDIA                                M4A_BOX_TYPE('m','d','i','a')
#define M4A_BOX_MDHD                                M4A_BOX_TYPE('m','d','h','d')
#define M4A_BOX_HDLR                                M4A_BOX_TYPE('h','d','l','r')
#define M4A_BOX_MINF                                M4A_BOX_TYPE('m','i','n','f')
#define M4A_BOX_STBL                                M4A_BOX_TYPE('s','t','b','l')
#define M4A_BOX_STSD                                M4A_BOX_TYPE('s','t','s','d')
#define M4A_BOX_MP4A                                M4A_BOX_TYPE('m','p','4','a')
#define M4A_BOX_ESDS                                M4A_BOX_TYPE('e','s','d','s')
#define M4A_BOX_STTS                                M4A_BOX_TYPE('s','t','t','s')
#define M4A_BOX_STSC                                M4A_BOX_TYPE('s','t','s','c')
#define M4A_BOX_STSZ                                M4A_BOX_TYPE('s','t','s','z')
#define M4A_BOX_STZ2                                M4A_BOX_TYPE('s','t','z','2')
#define M4A_BOX_STCO                                M4A_BOX_TYPE('s','t','c','o')
#define M4A_BOX_CO64                                M4A_BOX_TYPE('c','o','6','4')
#define M4A_BOX_MDAT                                M4A_BOX_TYPE('m','d','a','t')

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    M4A_RESULT_SUCCESS          = 0,
    M4A_RESULT_ERROR            = -1,
    M4A_RESULT_NEED_MORE_BITS   = -2,
} M4A_RESULT_T;

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

#if 0
static void m4a_dump_tables(aac_decoder_t* dec)
{
    uint32_t i;

    if (1)
    {
        if (dec->m4a_sample_size_table.table_variant == 0)
        {
            if (dec->m4a_sample_size_table.sample_size != 0)
            {
                printf("Sample Size (stsz) Table: All samples are size %lu\n", dec->m4a_sample_size_table.sample_size);
            }
            else
            {
                uint32_t* tv = (uint32_t*) dec->m4a_sample_size_table.table.data;

                printf("Sample Size (stsz) Table: %lu samples in track\n", dec->m4a_sample_size_table.sample_count);
                for (i = 0; i < dec->m4a_sample_size_table.sample_count; i++)
                {
                    printf("Sample number %lu, bytes %lu\n", i, ntohl(tv[i]));
                }
            }
        }
        else
        {
            printf("STSZ Table: Variant #1\n");
        }
        printf("\n");
    }

    if (1)
    {
        uint32_t* stsc_entry_ptr = (uint32_t*)dec->m4a_sample_chunk_table.table.data;

        printf("STSC Table: %lu entries\n", dec->m4a_sample_chunk_table.entry_count);
        for (i = 0; i < dec->m4a_sample_chunk_table.entry_count; i++)
        {
            printf("First chunk: %03lu, samples per chunk: %03lu, desc idx: %03lu\n", ntohl(stsc_entry_ptr[0]), ntohl(stsc_entry_ptr[1]), ntohl(stsc_entry_ptr[2]));
            stsc_entry_ptr += 3;
        }
        printf("\n");
    }

    if (1)
    {
        uint32_t* tv32 = (uint32_t*)dec->m4a_chunk_offset_table.table.data;
        uint64_t* tv64 = (uint64_t*)dec->m4a_chunk_offset_table.table.data;

        printf("STCO Table: %lu entries\n", dec->m4a_chunk_offset_table.entry_count);
        for (i = 0; i < dec->m4a_chunk_offset_table.entry_count; i++ )
        {
            if (dec->m4a_chunk_offset_table.table_variant == 0)
            {
                printf("Chunk %03lu, offset: %lu\n", i, ntohl(tv32[i]));
            }
            else
            {
                printf("Chunk %03lu, offset: %lu\n", i, (uint32_t)(SWAP64(tv64[i])));
            }
        }
        printf("\n");
    }

    if (1)
    {
        uint32_t* stts_entry_ptr = (uint32_t*)dec->m4a_time_sample_table.table.data;
        uint32_t total_delta = 0;

        printf("STTS Table: %lu entries\n", dec->m4a_time_sample_table.entry_count);
        for (i = 0; i < dec->m4a_time_sample_table.entry_count; i++)
        {
            printf("Entry %03lu:  Sample count: %03lu, sample delta: %04lu\n", i, ntohl(stts_entry_ptr[0]), ntohl(stts_entry_ptr[1]));
            total_delta += ntohl(stts_entry_ptr[0]) * ntohl(stts_entry_ptr[1]);
            stts_entry_ptr += 2;
        }
        printf("Total delta is %lu (%lu)\n", total_delta, (uint32_t)dec->m4a_track_info.duration);
        printf("\n");
    }
}
#endif


static int m4a_initiate_start_offset(aac_decoder_t* dec)
{
    uint64_t    target_offset;
    uint64_t    total_delta;
    uint32_t*   ptr32;
    uint32_t    sample_number;
    uint32_t    sample_delta;
    uint32_t    num_samples;
    uint32_t    base_sample;
    uint32_t    samples_per_chunk;
    uint32_t    chunk_sample_table_idx;
    uint32_t    chunk_number;
    uint32_t    chunk;
    uint32_t    count;
    uint32_t    byte_offset;
    uint32_t    sample_offset;
    uint32_t    sample_idx;
    uint32_t    size;
    uint32_t    i;
    uint8_t     fsize;
    uint8_t     not_found;

    /*
     * We need to figure out where in the stream to start.
     * Start by converting the millisecond time offset to the proper timescale.
     */

    target_offset = (uint32_t)((double)dec->audio_client->start_offset_ms * ((double)dec->m4a_track_info.timescale / 1000.0));

    /*
     * We need to find sample that contains this time.
     */

    sample_number = 0;
    total_delta   = 0;
    ptr32         = (uint32_t*)dec->m4a_time_sample_table.table.data;

    for (i = 0, not_found = 1; i < dec->m4a_time_sample_table.entry_count && not_found; i++)
    {
        count        = ntohl(ptr32[0]);
        sample_delta = ntohl(ptr32[1]);
        for (num_samples = 0; num_samples < count; num_samples++)
        {
            if (target_offset < total_delta + sample_delta)
            {
                /*
                 * The target offset is within this sample.
                 * Break out of the search loop.
                 */

                not_found = 0;
                break;
            }
            total_delta += sample_delta;
            sample_number++;
        }
        ptr32 += 2;
    }

    if (not_found)
    {
        /*
         * Something went wrong and we couldn't find what a sample with the requested time offset.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "m4a_initiate_start_offset: sample not found (%lu, %lu)\n", (uint32_t)target_offset, (uint32_t)total_delta);
        return -1;
    }

    /*
     * Now we need to find the chunk that contains this sample.
     */

    chunk_number = 0;
    base_sample  = 0;
    ptr32        = (uint32_t*)dec->m4a_sample_chunk_table.table.data;

    for (i = 0, not_found = 1; i < dec->m4a_sample_chunk_table.entry_count && not_found; i++)
    {
        /*
         * Get the number of samples per chunk for this block of chunks and
         * how many chunks are in the block.
         */

        samples_per_chunk = ntohl(ptr32[1]);
        if (i + 1 < dec->m4a_sample_chunk_table.entry_count)
        {
            count = ntohl(ptr32[0 + 3]) - ntohl(ptr32[0]);
        }
        else
        {
            count = dec->m4a_chunk_offset_table.entry_count - ntohl(ptr32[0]);
        }
        for (chunk = 0; chunk < count; chunk++)
        {
            if (sample_number < base_sample + samples_per_chunk)
            {
                /*
                 * The target offset is within this sample.
                 * Break out of the search loop.
                 */

                not_found = 0;
                chunk_sample_table_idx = i;
                break;
            }
            base_sample += samples_per_chunk;
            chunk_number++;
        }
        ptr32 += 3;
    }

    if (not_found)
    {
        /*
         * Something went wrong and we couldn't find what a chunk with the sample we want.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "m4a_initiate_start_offset: chunk not found (%lu, %lu)\n", sample_number, base_sample);
        return -1;
    }

    /*
     * At this point we have the sample for the requested time in the track: sample_number
     * And the chunk that contains the sample: chunk_number
     * Along with the first sample in the chunk: base_sample
     *
     * Now we need to get the starting address for the chunk and add in the sample
     * offset within the chunk.
     */

    if (dec->m4a_chunk_offset_table.table_variant == 0)
    {
        byte_offset = (uint32_t)ntohl(((uint32_t*)dec->m4a_chunk_offset_table.table.data)[chunk_number]);
    }
    else
    {
        byte_offset = (uint32_t)SWAP64(((uint64_t*)dec->m4a_chunk_offset_table.table.data)[chunk_number]);
    }

    sample_offset = 0;
    sample_idx    = 0;
    if (dec->m4a_sample_size_table.table_variant == 0)
    {
        /*
         * We need to calculate where this sample starts within the chunk.
         */

        if (dec->m4a_sample_size_table.sample_size != 0)
        {
            sample_offset = dec->m4a_sample_size_table.sample_size * (sample_number - base_sample);
        }
        else
        {
            ptr32 = (uint32_t*)dec->m4a_sample_size_table.table.data;
            for (i = base_sample; i < sample_number; i++)
            {
                sample_offset += ntohl(ptr32[i]);
            }
        }
    }
    else
    {
        fsize      = dec->m4a_sample_size_table.field_size;             /* Sample field size in bits (4, 8, or 16)  */
        sample_idx = (fsize * base_sample) / 8;                         /* Table index in bytes                     */
        for (i = base_sample; i < sample_number; i++)
        {
            size = *(dec->m4a_sample_size_table.table.data + sample_idx);
            if (fsize < 8)
            {
                sample_offset += (size >> ((i & 0x01) << 3) & 0xFF);
                if (i & 0x01)
                {
                    sample_idx++;
                }
            }
            else
            {
                if (fsize > 8)
                {
                    sample_offset += (size << 8) | *(dec->m4a_sample_size_table.table.data + sample_idx + 1);
                    sample_idx += 2;
                }
                else
                {
                    sample_offset += size;
                    sample_idx++;
                }
            }
        }
    }
    byte_offset += sample_offset;

    /*
     * Tell the upper layer that we want to seek to the proper file location.
     * We need to set the flag to wait for the pending stream position callback
     * before initiating the seek request. Since the reader thread runs at a
     * higher priority, it is possible to get the callback setting the new
     * stream position before this routine returns.
     */

    dec->wait_for_stream_position = 1;
    if (aac_start_offset_callback(dec, dec->audio_client->start_offset_ms, byte_offset))
    {
        dec->wait_for_stream_position = 0;
        return -1;
    }

    /*
     * The seek request was successful. Now we need to set up the table indices to match where
     * we'll begin playback.
     */

    if (dec->m4a_sample_size_table.table_variant == 0)
    {
        if (dec->m4a_sample_size_table.sample_size == 0)
        {
            dec->m4a_sample_size_table.sample_count_idx = sample_number;
            dec->m4a_sample_size_table.table.data_idx   = sizeof(uint32_t) * sample_number;
        }
    }
    else
    {
        dec->m4a_sample_size_table.sample_count_idx = sample_number;
        dec->m4a_sample_size_table.table.data_idx   = sample_idx;
    }

    dec->m4a_sample_chunk_table.entry_count_idx        = chunk_sample_table_idx;
    dec->m4a_sample_chunk_table.table.data_idx         = chunk_sample_table_idx * (sizeof(uint32_t) * 3);
    dec->m4a_sample_chunk_table.sample_in_chunk_offset = sample_offset;
    dec->m4a_sample_chunk_table.sample_in_chunk_idx    = (sample_number - base_sample);

    dec->m4a_chunk_offset_table.entry_count_idx = chunk_number;
    if (dec->m4a_chunk_offset_table.table_variant == 0)
    {
        dec->m4a_chunk_offset_table.table.data_idx = sizeof(uint32_t) * chunk_number;
    }
    else
    {
        dec->m4a_chunk_offset_table.table.data_idx = sizeof(uint64_t) * chunk_number;
    }

    return 0;
}


static M4A_RESULT_T m4a_get_box_type_and_size(aac_decoder_t* dec, uint32_t idx, uint32_t* box_type, uint32_t* box_size)
{
    *box_type = 0;
    *box_size = 0;

    if (idx + 8 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    *box_size = M4A_GET_BOX_SIZE(&dec->data_coalesce[idx]);
    *box_type = M4A_GET_BOX_TYPE(&dec->data_coalesce[idx + 4]);

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_continue_table_data_copy(aac_decoder_t* dec, uint32_t* data_idx, m4a_table_data_t* table, uint32_t completion_flag)
{
    uint32_t bytes_to_copy;
    uint32_t idx = *data_idx;

    /*
     * Continue copying the table data.
     */

    bytes_to_copy = table->data_size - table->data_idx;
    if (idx + bytes_to_copy > dec->data_widx)
    {
        bytes_to_copy = dec->data_widx - idx;
    }
    memcpy(&table->data[table->data_idx], &dec->data_coalesce[idx], bytes_to_copy);
    table->data_idx += bytes_to_copy;

    *data_idx = idx + bytes_to_copy;
    if (table->data_idx < table->data_size)
    {
        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * All done.
     */

    table->data_idx = 0;
    dec->search_status |= completion_flag;

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_allocate_copy_table_data(aac_decoder_t* dec, uint32_t* data_idx, m4a_table_data_t* table, uint32_t completion_flag)
{
    uint32_t bytes_to_copy;
    uint32_t idx = *data_idx;

    if (table->data != NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "table data already exists (0x%08lx)\n", completion_flag);
        free(table->data);
        table->data = NULL;
    }

    table->data = calloc(1, table->data_size);
    if (table->data == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate table data (0x%08lx)\n", completion_flag);
        return M4A_RESULT_ERROR;
    }

    table->data_idx  = 0;

    bytes_to_copy = table->data_size;
    if (idx + bytes_to_copy > dec->data_widx)
    {
        bytes_to_copy = dec->data_widx - idx;
    }
    memcpy(table->data, &dec->data_coalesce[idx], bytes_to_copy);
    table->data_idx += bytes_to_copy;

    *data_idx = idx + bytes_to_copy;
    if (table->data_idx < table->data_size)
    {
        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * All done.
     */

    table->data_idx = 0;
    dec->search_status |= completion_flag;

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_parse_box_stco(aac_decoder_t* dec, uint32_t box_size, uint32_t* data_idx)
{
    uint32_t box_type;
    uint32_t begin_idx;
    uint32_t idx;

    if ((dec->search_status & (SEARCH_M4A_STCO_FOUND | SEARCH_M4A_CO64_FOUND)) && !(dec->search_status & SEARCH_M4A_COFFSET_COMPLETE))
    {
        /*
         * We are in the midst of an incremental copy.
         */

        return m4a_continue_table_data_copy(dec, data_idx, &dec->m4a_chunk_offset_table.table, SEARCH_M4A_COFFSET_COMPLETE);
    }

    idx = *data_idx;
    if (idx + 20 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * Chunk Offset Box, 14496-12 section 8.7.5.
     * Might be stsz or stz2.
     */

    box_type = M4A_GET_BOX_TYPE(&dec->data_coalesce[idx + 4]);

    begin_idx = idx;
    idx += M4A_BOX_HEADER_SIZE(box_size) + 4;

    dec->m4a_chunk_offset_table.entry_count = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;

    if (box_type == M4A_BOX_STCO)
    {
        dec->m4a_chunk_offset_table.table_variant = 0;

        dec->search_status |= SEARCH_M4A_STCO_FOUND;
    }
    else    /* M4A_BOX_CO64 */
    {
        dec->m4a_chunk_offset_table.table_variant = 1;

        dec->search_status |= SEARCH_M4A_CO64_FOUND;
    }

    dec->m4a_chunk_offset_table.table.data_size = box_size - (idx - begin_idx);
    dec->m4a_chunk_offset_table.entry_count_idx = 0;
    dec->m4a_chunk_offset_table.table.data_idx  = 0;

    *data_idx = idx;
    return m4a_allocate_copy_table_data(dec, data_idx, &dec->m4a_chunk_offset_table.table, SEARCH_M4A_COFFSET_COMPLETE);
}


static M4A_RESULT_T m4a_parse_box_stsz(aac_decoder_t* dec, uint32_t box_size, uint32_t *data_idx)
{
    uint32_t box_type;
    uint32_t begin_idx;
    uint32_t sample_size;
    uint32_t sample_count;
    uint32_t idx;

    if ((dec->search_status & (SEARCH_M4A_STSZ_FOUND | SEARCH_M4A_STZ2_FOUND)) && !(dec->search_status & SEARCH_M4A_STSIZE_COMPLETE))
    {
        /*
         * We are in the midst of an incremental copy.
         */

        return m4a_continue_table_data_copy(dec, data_idx, &dec->m4a_sample_size_table.table, SEARCH_M4A_STSIZE_COMPLETE);
    }

    idx = *data_idx;
    if (idx + 20 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * Sample Size Box, 14496-12 section 8.7.3.
     * Might be stsz or stz2.
     */

    box_type = M4A_GET_BOX_TYPE(&dec->data_coalesce[idx + 4]);

    begin_idx = idx;
    idx += M4A_BOX_HEADER_SIZE(box_size) + 4;

    sample_size  = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;
    sample_count = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;

    if (box_type == M4A_BOX_STSZ)
    {
        dec->m4a_sample_size_table.table_variant = 0;
        dec->m4a_sample_size_table.sample_size   = sample_size;

        dec->search_status |= SEARCH_M4A_STSZ_FOUND;
    }
    else    /* M4A_BOX_STZ2 */
    {
        dec->m4a_sample_size_table.table_variant = 1;
        dec->m4a_sample_size_table.field_size    = sample_size & 0xFF;

        dec->search_status |= SEARCH_M4A_STZ2_FOUND;
    }

    dec->m4a_sample_size_table.sample_count     = sample_count;
    dec->m4a_sample_size_table.sample_count_idx = 0;
    dec->m4a_sample_size_table.table.data_size  = box_size - (idx - begin_idx);
    dec->m4a_sample_size_table.table.data_idx   = 0;

    *data_idx = idx;
    return m4a_allocate_copy_table_data(dec, data_idx, &dec->m4a_sample_size_table.table, SEARCH_M4A_STSIZE_COMPLETE);
}


static M4A_RESULT_T m4a_parse_box_stsc(aac_decoder_t* dec, uint32_t box_size, uint32_t* data_idx)
{
    uint32_t begin_idx;
    uint32_t idx;

    if ((dec->search_status & SEARCH_M4A_STSC_FOUND) && !(dec->search_status & SEARCH_M4A_STSC_COMPLETE))
    {
        /*
         * We are in the midst of an incremental copy.
         */

        return m4a_continue_table_data_copy(dec, data_idx, &dec->m4a_sample_chunk_table.table, SEARCH_M4A_STSC_COMPLETE);
    }

    idx = *data_idx;
    if (idx + 20 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * Sample to Chunk Box, 14496-12 section 8.7.4.
     */

    begin_idx = idx;
    idx += M4A_BOX_HEADER_SIZE(box_size) + 4;

    dec->m4a_sample_chunk_table.entry_count = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;

    dec->search_status |= SEARCH_M4A_STSC_FOUND;

    dec->m4a_sample_chunk_table.table.data_size = box_size - (idx - begin_idx);
    dec->m4a_sample_chunk_table.table.data_idx  = 0;

    *data_idx = idx;
    return m4a_allocate_copy_table_data(dec, data_idx, &dec->m4a_sample_chunk_table.table, SEARCH_M4A_STSC_COMPLETE);
}


static M4A_RESULT_T m4a_parse_box_stts(aac_decoder_t* dec, uint32_t box_size, uint32_t* data_idx)
{
    uint32_t begin_idx;
    uint32_t idx;

    if ((dec->search_status & SEARCH_M4A_STTS_FOUND) && !(dec->search_status & SEARCH_M4A_STTS_COMPLETE))
    {
        /*
         * We are in the midst of an incremental copy.
         */

        return m4a_continue_table_data_copy(dec, data_idx, &dec->m4a_time_sample_table.table, SEARCH_M4A_STTS_COMPLETE);
    }

    idx = *data_idx;
    if (idx + 20 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * Time to Sample Box, 14496-12 section 8.6.1.2.
     */

    begin_idx = idx;
    idx += M4A_BOX_HEADER_SIZE(box_size) + 4;

    dec->m4a_time_sample_table.entry_count = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;

    dec->m4a_time_sample_table.table.data_size = box_size - (idx - begin_idx);
    dec->m4a_time_sample_table.table.data_idx  = 0;

    dec->search_status |= SEARCH_M4A_STTS_FOUND;

    *data_idx = idx;
    return m4a_allocate_copy_table_data(dec, data_idx, &dec->m4a_time_sample_table.table, SEARCH_M4A_STTS_COMPLETE);
}


static M4A_RESULT_T m4a_parse_box_esds(aac_decoder_t* dec, uint32_t box_size, uint32_t idx)
{
    uint32_t size_of_instance;
    uint8_t stream_dependence_flag;
    uint8_t url_flag;
    uint8_t ocr_stream_flag;
    uint8_t len;

    if (idx + box_size >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    idx += M4A_BOX_HEADER_SIZE(box_size) + 4;

    /*
     * Parse the ESDescriptor, 14496-1 section 7.2.6.5.
     * Start by computing the variable sizeOfInstance, 14496-1 section 8.3.3.
     */

    size_of_instance = 0;
    do
    {
        idx++;
        size_of_instance = (size_of_instance << 7) | (dec->data_coalesce[idx] & 0x7F);
    } while ((dec->data_coalesce[idx] & 0x80) != 0);
    idx++;

    /*
     * Skip the ES_ID and get the optional flags.
     */

    idx += 2;
    stream_dependence_flag = dec->data_coalesce[idx] & 0x01;
    url_flag               = dec->data_coalesce[idx] & 0x02;
    ocr_stream_flag        = dec->data_coalesce[idx] & 0x04;
    idx++;

    if (stream_dependence_flag)
    {
        idx += 2;
    }

    if (url_flag)
    {
        len = dec->data_coalesce[idx];
        idx += 1 + len;
    }

    if (ocr_stream_flag)
    {
        idx += 2;
    }

    /*
     * Now parse the DecoderConfigDescriptor, 14996-1 section 7.2.6.6.
     */

    size_of_instance = 0;
    do
    {
        idx++;
        size_of_instance = (size_of_instance << 7) | (dec->data_coalesce[idx] & 0x7F);
    } while ((dec->data_coalesce[idx] & 0x80) != 0);
    idx++;

    /*
     * Skip all the fields to the DecoderSpecificInfo.
     */

    idx += 13;

    size_of_instance = 0;
    do
    {
        idx++;
        size_of_instance = (size_of_instance << 7) | (dec->data_coalesce[idx] & 0x7F);
    } while ((dec->data_coalesce[idx] & 0x80) != 0);
    idx++;

    if (dec->m4a_audio_specific_info != NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "m4a_audio_specific_info already exists\n");
        free(dec->m4a_audio_specific_info);
        dec->m4a_audio_specific_info = NULL;
    }

    dec->m4a_audio_specific_info_size = size_of_instance;
    dec->m4a_audio_specific_info = calloc(1, dec->m4a_audio_specific_info_size);
    if (dec->m4a_audio_specific_info == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate m4a_audio_specific_info\n");
        return M4A_RESULT_ERROR;
    }

    memcpy(dec->m4a_audio_specific_info, &dec->data_coalesce[idx], dec->m4a_audio_specific_info_size);

    dec->search_status |= SEARCH_M4A_DECSPECINFO_FOUND;

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_parse_box_mp4a(aac_decoder_t* dec, uint32_t box_size, uint32_t idx)
{
    uint32_t version;
    uint16_t channel_count;
    uint16_t sample_size;
    uint16_t pre_defined;
    uint16_t reserved;
    uint32_t sample_rate;
    uint32_t type;
    uint32_t size;

    (void)pre_defined;
    (void)reserved;

    /*
     * Audio Sample Entry, 14496-12 section 8.5.2.
     */

    if (idx + box_size >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * Skip sample entry fields.
     */

    idx += M4A_BOX_HEADER_SIZE(box_size) + 8;

    /*
     * Grab the version and skip reserved[2]
     */

    version = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 8;

    channel_count = ntohs(*((uint16_t*)(&dec->data_coalesce[idx])));
    idx += 2;

    sample_size = ntohs(*((uint16_t*)(&dec->data_coalesce[idx])));
    idx += 2;

    pre_defined = ntohs(*((uint16_t*)(&dec->data_coalesce[idx])));
    idx += 2;

    reserved = ntohs(*((uint16_t*)(&dec->data_coalesce[idx])));
    idx += 2;

    sample_rate = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    idx += 4;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "mp4a: V%d, channels %u, bps %u, rate %lu\n",
                  version, channel_count, sample_size, sample_rate >> 16);

    if (version != 0)
    {
        idx += 16;
    }

    /*
     * Make sure that we are at the esds box.
     */

    m4a_get_box_type_and_size(dec, idx, &type, &size);
    if (type != M4A_BOX_ESDS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "mp4a: No esds box\n");
        return M4A_RESULT_ERROR;
    }

    return m4a_parse_box_esds(dec, size, idx);
}


static M4A_RESULT_T m4a_parse_box_hdlr(aac_decoder_t* dec, uint32_t box_size, uint32_t idx)
{
    /*
     * Handler Reference Box, 14496-12 section 8.4.3.
     */

    if (idx + 20 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    idx += M4A_BOX_HEADER_SIZE(box_size) + 8;

    /*
     * We need to make sure that this is an audio track.
     */

    if (dec->data_coalesce[idx] != 's' || dec->data_coalesce[idx + 1] != 'o' || dec->data_coalesce[idx + 2] != 'u' || dec->data_coalesce[idx + 3] != 'n')
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Not an audio track\n");
        return M4A_RESULT_ERROR;
    }

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_parse_box_mdhd(aac_decoder_t* dec, uint32_t box_size, uint32_t idx)
{
    uint8_t version;

    /*
     * Media Header Box, 14496-12 section 8.4.2.
     */

    if (idx + 44 >= dec->data_widx)
    {
        /*
         * Not enough data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }

    idx += M4A_BOX_HEADER_SIZE(box_size);

    version = dec->data_coalesce[idx];
    idx += 4;

    if (version == 1)
    {
        dec->m4a_track_info.creation_time     = SWAP64(*((uint64_t*)(&dec->data_coalesce[idx])));
        idx += 8;

        dec->m4a_track_info.modification_time = SWAP64(*((uint64_t*)(&dec->data_coalesce[idx])));
        idx += 8;

        dec->m4a_track_info.timescale         = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
        idx += 4;

        dec->m4a_track_info.duration          = SWAP64(*((uint64_t*)(&dec->data_coalesce[idx])));
    }
    else
    {
        dec->m4a_track_info.creation_time     = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
        idx += 4;

        dec->m4a_track_info.modification_time = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
        idx += 4;

        dec->m4a_track_info.timescale         = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
        idx += 4;

        dec->m4a_track_info.duration          = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
    }

    /*
     * We don't care about the language code.
     */

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_parse_box_ftyp(aac_decoder_t* dec)
{
    uint8_t*    ptr;
    uint32_t    box_size;
    uint32_t    offset;

    ptr = &dec->data_coalesce[dec->data_ridx];
    if (ptr[4] != 'f' || ptr[5] != 't' || ptr[6] != 'y' || ptr[7] != 'p')
    {
        return M4A_RESULT_ERROR;
    }

    box_size = M4A_GET_BOX_SIZE(ptr);
    if (dec->data_widx - dec->data_ridx < box_size)
    {
        /*
         * We need more data.
         */

        return M4A_RESULT_NEED_MORE_BITS;
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Found box %.4s, size %lu\n", &ptr[4], box_size);
    dec->search_status |= SEARCH_M4A_FTYP_FOUND;

    dec->data_skip_bytes = box_size;

    /*
     * Check the major brand for audio.
     */

    offset = 8;
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Major brand %.4s\n", &ptr[offset]);
    if (strncasecmp((char*)&ptr[offset], "m4a ", 4) != 0 && strncasecmp((char*)&ptr[offset], "mp42", 4) != 0)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Non-audio major brand %.4s\n", &ptr[offset]);
        return M4A_RESULT_ERROR;
    }
    dec->search_status |= SEARCH_M4A_FTYP_AUDIO_FOUND;

    /*
     * Skip the minor version and check the compatible brands.
     */

    for (offset += 8; offset < box_size; offset += 4)
    {
        if (strncasecmp((char*)&ptr[offset], "isom", 4) == 0)
        {
            dec->search_status |= SEARCH_M4A_FTYP_ISO_FOUND;
        }
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Compatible brand %.4s\n", &ptr[offset]);
    }

    return M4A_RESULT_SUCCESS;
}


static M4A_RESULT_T m4a_find_audio_info(audio_client_t* client, aac_decoder_t* dec)
{
    uint32_t            box_type;
    uint32_t            size;
    uint32_t            count;
    uint32_t            idx;
    M4A_RESULT_T        result;

    if (dec->data_widx - dec->data_ridx < 16)
    {
        return M4A_RESULT_NEED_MORE_BITS;
    }

    /*
     * ftyp box must be first.
     */

    if (!(dec->search_status & SEARCH_M4A_FTYP_FOUND))
    {
        result = m4a_parse_box_ftyp(dec);
        if (result != M4A_RESULT_SUCCESS)
        {
            if (result != M4A_RESULT_NEED_MORE_BITS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "M4A file does not start with proper ftyp!\n");
            }
            return result;
        }
    }

    result = M4A_RESULT_SUCCESS;
    idx = dec->data_ridx;
    while (idx < dec->data_widx - 16 && result == M4A_RESULT_SUCCESS && !M4A_SEARCH_COMPLETE(dec))
    {
        if (dec->data_skip_bytes)
        {
            dec->collecting_box_type = 0;

            size = dec->data_skip_bytes;
            if (idx + size >= dec->data_widx)
            {
                size = dec->data_widx - idx;
            }
            idx += size;
            dec->data_skip_bytes -= size;
            continue;
        }

        if (dec->collecting_box_type == 0)
        {
            m4a_get_box_type_and_size(dec, idx, &box_type, &size);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Found box %.4s, size %lu\n", (char*)&box_type, size);
        }
        else
        {
            box_type = dec->collecting_box_type;
            size     = dec->collecting_box_size;
        }

        switch (box_type)
        {
            case M4A_BOX_MOOV:
                dec->search_status |= SEARCH_M4A_MOOV_FOUND;
                idx += M4A_BOX_HEADER_SIZE(size);
                break;

            case M4A_BOX_TRAK:
                dec->search_status |= SEARCH_M4A_TRAK_FOUND;
                idx += M4A_BOX_HEADER_SIZE(size);
                break;

            case M4A_BOX_MDIA:
                dec->search_status |= SEARCH_M4A_MDIA_FOUND;
                idx += M4A_BOX_HEADER_SIZE(size);
                break;

            case M4A_BOX_MDHD:
                result = m4a_parse_box_mdhd(dec, size, idx);
                if (result == M4A_RESULT_SUCCESS)
                {
                    dec->data_skip_bytes = size;
                }
                break;

            case M4A_BOX_HDLR:
                result = m4a_parse_box_hdlr(dec, size, idx);
                if (result == M4A_RESULT_SUCCESS)
                {
                    dec->data_skip_bytes = size;
                }
                break;

            case M4A_BOX_MINF:
                dec->search_status |= SEARCH_M4A_MINF_FOUND;
                idx += M4A_BOX_HEADER_SIZE(size);
                break;

            case M4A_BOX_STBL:
                idx += M4A_BOX_HEADER_SIZE(size);
                break;

            case M4A_BOX_STSD:
                dec->search_status |= SEARCH_M4A_STSD_FOUND;
                idx += M4A_BOX_HEADER_SIZE(size) + 4;
                count = ntohl(*((uint32_t*)(&dec->data_coalesce[idx])));
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "STSD contains %lu entries\n", count);
                idx += 4;
                break;

            case M4A_BOX_MP4A:
                result = m4a_parse_box_mp4a(dec, size, idx);
                if (result == M4A_RESULT_SUCCESS)
                {
                    dec->data_skip_bytes = size;
                }
                break;

            case M4A_BOX_STTS:
                result = m4a_parse_box_stts(dec, size, &idx);
                break;

            case M4A_BOX_STSC:
                result = m4a_parse_box_stsc(dec, size, &idx);
                break;

            case M4A_BOX_STSZ:
            case M4A_BOX_STZ2:
                result = m4a_parse_box_stsz(dec, size, &idx);
                break;

            case M4A_BOX_STCO:
            case M4A_BOX_CO64:
                result = m4a_parse_box_stco(dec, size, &idx);
                break;

            case M4A_BOX_MDAT:
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "M4A file: MDAT during header processing!\n");
                return WICED_ERROR;

            default:
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Skip box %.4s\n", (char*)&box_type);
                dec->data_skip_bytes = size;
                break;
        }

        if (result == M4A_RESULT_NEED_MORE_BITS)
        {
            dec->collecting_box_type = box_type;
            dec->collecting_box_size = size;
        }
        else
        {
            dec->collecting_box_type = 0;
            dec->collecting_box_size = 0;
        }
    }

    dec->data_ridx = idx;
    if (dec->data_ridx == dec->data_widx)
    {
        dec->data_widx = 0;
        dec->data_ridx = 0;
    }
    else if (dec->data_ridx < dec->data_widx)
    {
        memmove(dec->data_coalesce, &dec->data_coalesce[dec->data_ridx], dec->data_widx - dec->data_ridx);
        dec->data_bitstream_idx += dec->data_ridx;
        dec->data_widx -= dec->data_ridx;
        dec->data_ridx = 0;
    }

    return result;
}

static M4A_RESULT_T m4a_get_sample_limits(aac_decoder_t* dec)
{
    uint64_t offset;
    uint64_t size;
    uint8_t field_size;
    uint8_t inc;
    uint32_t* stsc_entry_ptr;
    uint32_t stsc_sample_per_chunk;

    if (dec->m4a_sample_size_table.sample_count_idx >= dec->m4a_sample_size_table.sample_count)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Sample count index exceeded\n");
        return WICED_ERROR;
    }

    if (dec->m4a_chunk_offset_table.entry_count_idx >= dec->m4a_chunk_offset_table.entry_count)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Chunk count index exceeded\n");
        return WICED_ERROR;
    }

    /*
     * Decode size table.
     */

    if (dec->m4a_sample_size_table.table_variant == 0)
    {
        if (dec->m4a_sample_size_table.sample_size != 0)
        {
            size = dec->m4a_sample_size_table.sample_size;
        }
        else
        {
            size = htonl(*(uint32_t*)(dec->m4a_sample_size_table.table.data + dec->m4a_sample_size_table.table.data_idx));
            dec->m4a_sample_size_table.table.data_idx += sizeof(uint32_t);
            dec->m4a_sample_size_table.sample_count_idx++;
        }
    }
    else
    {
        field_size = dec->m4a_sample_size_table.field_size;

        size = dec->m4a_sample_size_table.table.data[dec->m4a_sample_size_table.table.data_idx];

        if (field_size < 8)
        {
            size = (size >> ((dec->m4a_sample_size_table.sample_count_idx & 0x01) << 3)) & 0xFF;
        }

        if (field_size > 8)
        {
            size = (size << 8) | (dec->m4a_sample_size_table.table.data[dec->m4a_sample_size_table.table.data_idx + 1]);
        }

        dec->m4a_sample_size_table.sample_count_idx++;

        inc = (field_size > 8) ? 2 : 1;
        inc >>= ((field_size < 8) && (dec->m4a_sample_size_table.sample_count_idx & 0x01)) ? 1 : 0;

        dec->m4a_sample_size_table.table.data_idx += inc;
    }

    /*
     * Decode sample to chunk table.
     */

    stsc_entry_ptr = (uint32_t*)(dec->m4a_sample_chunk_table.table.data + 3 * sizeof(uint32_t) * dec->m4a_sample_chunk_table.entry_count_idx);
    stsc_sample_per_chunk = ntohl(*(stsc_entry_ptr + 1));

    dec->m4a_sample_chunk_table.sample_in_chunk_idx++;
    dec->m4a_sample_chunk_offset = dec->m4a_sample_chunk_table.sample_in_chunk_offset;
    dec->m4a_sample_chunk_table.sample_in_chunk_offset += size;

    /*
     * Do we have to select the next chunk?
     */

    if (dec->m4a_sample_chunk_table.sample_in_chunk_idx > stsc_sample_per_chunk)
    {
        dec->m4a_sample_chunk_table.sample_in_chunk_idx = 1;
        dec->m4a_sample_chunk_offset = 0;
        dec->m4a_sample_chunk_table.sample_in_chunk_offset = size;

        /*
         * Increment chunk offset table based on table variant.
         */

        dec->m4a_chunk_offset_table.entry_count_idx++;
        if (dec->m4a_chunk_offset_table.table_variant == 0)
        {
            dec->m4a_chunk_offset_table.table.data_idx += sizeof(uint32_t);
        }
        else
        {
            dec->m4a_chunk_offset_table.table.data_idx += sizeof(uint64_t);
        }

        /*
         * Do we need to update the sample to chunk entry?
         */

        if (dec->m4a_sample_chunk_table.entry_count > dec->m4a_sample_chunk_table.entry_count_idx)
        {
            uint32_t* tmp_stsc_entry_ptr;
            uint32_t tmp_stsc_first_chunk;

            tmp_stsc_entry_ptr   = (uint32_t*)(dec->m4a_sample_chunk_table.table.data + 3 * sizeof(uint32_t) * (dec->m4a_sample_chunk_table.entry_count_idx + 1));
            tmp_stsc_first_chunk = ntohl(*tmp_stsc_entry_ptr);

            /*
             * Do we need to switch
             */
            if (dec->m4a_sample_chunk_table.entry_count_idx + 1 == tmp_stsc_first_chunk)
            {
                dec->m4a_sample_chunk_table.entry_count_idx++;
                dec->m4a_sample_chunk_table.table.data_idx += sizeof(uint32_t) * 3;
            }
        }
    }

    /*
     * Decode offset table.
     */

    if (dec->m4a_chunk_offset_table.table_variant == 0)
    {
        offset = ntohl(*(uint32_t*)(dec->m4a_chunk_offset_table.table.data + dec->m4a_chunk_offset_table.table.data_idx));
    }
    else
    {
        offset = SWAP64(*(uint64_t*)(dec->m4a_chunk_offset_table.table.data + dec->m4a_chunk_offset_table.table.data_idx));
    }

    dec->m4a_chunk_bitstream_offset = offset;
    dec->m4a_sample_size            = size;

    return WICED_SUCCESS;
}


static M4A_RESULT_T m4a_copy_data_to_coalesce_buffer(audio_client_t* client, aac_decoder_t* dec)
{
    data_buf_t* dbuf;

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (!dbuf->inuse)
    {
        return M4A_RESULT_SUCCESS;
    }

    if (dec->wait_for_stream_position)
    {
        aac_release_buffer(client, dbuf);
        return M4A_RESULT_NEED_MORE_BITS;
    }

    if ((dbuf->buflen - dbuf->bufused) + dec->data_widx >= COALESCE_BUF_SIZE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "AAC coalesce buffer overflow (%lu,%lu)\n", dec->data_ridx, dec->data_widx);
        return M4A_RESULT_ERROR;
    }

    /*
     * Copy the data into our coalesce buffer and keep track of the bitstream index.
     */

    memcpy(&dec->data_coalesce[dec->data_widx], &dbuf->buf[dbuf->bufused], dbuf->buflen - dbuf->bufused);
    if (dec->data_widx == 0)
    {
        dec->data_bitstream_idx = dec->bitstream_idx;
    }
    dec->bitstream_idx += dbuf->buflen - dbuf->bufused;
    dec->data_widx     += dbuf->buflen - dbuf->bufused;

    aac_release_buffer(client, dbuf);

    return M4A_RESULT_SUCCESS;
}


wiced_result_t m4a_process_data(audio_client_t* client)
{
    aac_decoder_t*      dec;
    M4A_RESULT_T        m4a_result;
    AAC_DECODER_ERROR   fdk_err;
    uint32_t            local_offset;
    uint32_t            push_size;
    uint32_t            fdk_valid;
    uint32_t            bytes_pushed;
    aac_pcm_info_t      pcm_info;
    CStreamInfo*        info;
    uint8_t*            data_ptr;

    dec = (aac_decoder_t*)client->decoder_handle;

    if ((m4a_result = m4a_copy_data_to_coalesce_buffer(client, dec)) != M4A_RESULT_SUCCESS)
    {
        return m4a_result == M4A_RESULT_NEED_MORE_BITS ? WICED_SUCCESS : WICED_ERROR;
    }

    /*
     * Have we finished parsing the header atoms?
     */

    if (!M4A_SEARCH_COMPLETE(dec))
    {
        m4a_result = m4a_find_audio_info(client, dec);
        if (m4a_result != M4A_RESULT_SUCCESS)
        {
            if (m4a_result != M4A_RESULT_NEED_MORE_BITS)
            {
                return WICED_ERROR;
            }
        }

        if (!M4A_SEARCH_COMPLETE(dec))
        {
            return WICED_SUCCESS;
        }

        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "All header boxes parsed!\n");

        /*
         * We've finished parsing all the header boxes. Are we starting from an offset?
         */

        if (client->start_offset_ms > 0)
        {
            if (m4a_initiate_start_offset(dec) == 0)
            {
                /*
                 * Reset the input buffers.
                 */

                dec->data_ridx = 0;
                dec->data_widx = 0;
                return WICED_SUCCESS;
            }
        }
    }

    /*
     * Has the decoder been configured?
     */

    if (!dec->m4a_decoder_configured)
    {
        if ((fdk_err = aacDecoder_ConfigRaw(dec->fdk_handle, &dec->m4a_audio_specific_info, (const UINT*)&dec->m4a_audio_specific_info_size)) != AAC_DEC_OK)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from ConfigRaw\n");
            return WICED_ERROR;
        }
        dec->m4a_decoder_configured = WICED_TRUE;
    }

    /*
     * We need to loop on the data buffer to make sure that we don't overflow.
     * Each HTTP buffer could contain multiple AAC frames.
     */

    while (dec->data_widx > dec->data_ridx && (dec->m4a_chunk_lookup || dec->m4a_sample_size < dec->data_widx - dec->data_ridx))
    {
        /*
         * Do we need to figure out the correct location in the stream?
         */

        if (dec->m4a_chunk_lookup)
        {
            dec->m4a_chunk_lookup = WICED_FALSE;

            m4a_get_sample_limits(dec);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "New chunk offset %lu, sample in chunk offset %lu, sample size %lu\n",
                          (uint32_t)dec->m4a_chunk_bitstream_offset, (uint32_t)dec->m4a_sample_chunk_offset, dec->m4a_sample_size);
        }

        /*
         * Does the input buffer contain data that we're looking for?
         */

        if (dec->data_bitstream_idx + (dec->data_widx - dec->data_ridx) < dec->m4a_chunk_bitstream_offset + dec->m4a_sample_chunk_offset)
        {
            /*
             * Release the buffered input since it is not needed.
             */

            dec->data_ridx = 0;
            dec->data_widx = 0;
            return WICED_SUCCESS;
        }

        /*
         * Adjust the read index to the proper location if necessary.
         */

        /*wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "Data index %lu, looking for index %lu (%lu,%lu)\n",
                      (uint32_t)dec->data_bitstream_idx, (uint32_t)(dec->m4a_chunk_bitstream_offset + dec->m4a_sample_chunk_offset), dec->data_ridx, dec->data_widx); */

        if (dec->data_bitstream_idx < dec->m4a_chunk_bitstream_offset + dec->m4a_sample_chunk_offset)
        {
            local_offset = dec->m4a_chunk_bitstream_offset + dec->m4a_sample_chunk_offset - dec->data_bitstream_idx;

            /*wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "Shift read index %lu bytes from %lu to %lu\n", local_offset, dec->data_ridx, dec->data_ridx + local_offset);*/

            if (dec->data_ridx + local_offset > dec->data_widx)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error adjusting read index\n");
                return WICED_ERROR;
            }

            dec->data_ridx += local_offset;
            dec->data_bitstream_idx += local_offset;

            if (dec->data_ridx == dec->data_widx)
            {
                dec->data_ridx = 0;
                dec->data_widx = 0;
                return WICED_SUCCESS;
            }
        }

        /*
         * Figure out how many bytes we're pushing to the decoder.
         * Note: The FDK decoder doesn't seem to like M4A frames passed to it in multiple chunks
         * so we'll wait until we have an entire frame ready to go.
         */

        push_size = dec->m4a_sample_size;
        if (push_size > dec->data_widx - dec->data_ridx)
        {
            break;
        }

        /*wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "Pushing %lu bytes from location %lu (%lu,%lu)\n", push_size, (uint32_t)dec->data_bitstream_idx, dec->data_ridx, dec->data_widx);*/

        /*
         * Fill the decoder input buffer.
         */

        data_ptr  = &dec->data_coalesce[dec->data_ridx];
        fdk_valid = push_size;
        fdk_err   = aacDecoder_Fill(dec->fdk_handle, (UCHAR**)&data_ptr, (const UINT*)&push_size, (UINT*)&fdk_valid);
        if (fdk_err != AAC_DEC_OK)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error pushing to fdk (0x%04x)\n", fdk_err);
            return WICED_ERROR;
        }

        bytes_pushed = push_size - fdk_valid;
        dec->m4a_sample_bytes_pushed += bytes_pushed;
        dec->data_ridx               += bytes_pushed;
        dec->data_bitstream_idx      += bytes_pushed;

        /*
         * Have we pushed the whole frame?
         */

        if (dec->m4a_sample_bytes_pushed == dec->m4a_sample_size)
        {
            /*
             * Lookup the next chunk/sample offsets the next time through.
             */

            dec->m4a_sample_bytes_pushed = 0;
            dec->m4a_chunk_lookup        = WICED_TRUE;
            push_size                    = 0;

            /*
             * Now decode the frame.
             */

            fdk_err = aacDecoder_DecodeFrame(dec->fdk_handle, (INT_PCM*)dec->data_decode, DECODE_BUF_SIZE, 0);
            if (fdk_err != AAC_DEC_OK)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error from aac_DecodeFrame (0x%04x)\n", fdk_err);
                return WICED_ERROR;
            }

            /*
             * Get the current stream info
             */

            info = aacDecoder_GetStreamInfo(dec->fdk_handle);
            if (!info || info->sampleRate <= 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "M4A: No stream info\n");
                return WICED_ERROR;
            }

            /* our FDK only sends s16LE output */
            pcm_info.bps      = 16;
            pcm_info.cbps     = 16;
            pcm_info.chnum    = info->numChannels;
            pcm_info.sr       = info->sampleRate;

            aac_pcm_output_push(dec, &pcm_info, dec->data_decode, info->frameSize * info->numChannels);
        }
    }

    if (dec->data_ridx == dec->data_widx)
    {
        dec->data_ridx = 0;
        dec->data_widx = 0;
    }
    else if (dec->data_ridx > 0)
    {
        /*
         * Shift the data down so we don't overflow the coalesce buffer.
         */

        memmove(dec->data_coalesce, &dec->data_coalesce[dec->data_ridx], dec->data_widx - dec->data_ridx);
        dec->data_widx -= dec->data_ridx;
        dec->data_ridx = 0;
    }

    return WICED_SUCCESS;
}
