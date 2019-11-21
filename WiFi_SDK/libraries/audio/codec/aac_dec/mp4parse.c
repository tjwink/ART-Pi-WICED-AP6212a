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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined ( LINUX )
    #include <netinet/in.h>
#endif

#include "globals.h"
#include "mp4parse_core.h"
#include "tsp.h"
#include "tpm.h"
#include "sysclk.h"

#include "logger.h"

#include "aacdec_types.h"
#include "aacdec_core.h"


// #define GCC_UNUSED __attribute__((__unused__))

/*
 * internal timeout for start/stop failures
 */
#define MP4PARSE_TIMEOUT_MS ( (uint32_t) 200 )

#if defined ( LINUX )
    #define SWAP64( x ) be64toh( ( x ) )
#endif

#if defined ( WICED )
    #define SWAP64( x ) ( ( (uint64_t) htonl( (uint32_t) x ) ) << 32 | htonl( (uint32_t) ( x >> 32 ) ) )
#endif

/*
 * search & indexing flags
 */
#define SEARCH_EMPTY_FLAG                ( (uint32_t) ( 0 ) )
#define SEARCH_NONE_FOUND                ( (uint32_t) ( 0 ) )
#define SEARCH_FTYP_FOUND             ( (uint32_t) ( 1 << 0 ) )
#define SEARCH_FTYP_AUDIO_FOUND       ( (uint32_t) ( 1 << 1 ) )
#define SEARCH_DISCARD_FOUND          ( (uint32_t) ( 1 << 2 ) )
#define SEARCH_DISCARD_COMPLETE       ( (uint32_t) ( 1 << 3 ) )
#define SEARCH_FTYP_ISO_FOUND         ( (uint32_t) ( 1 << 4 ) )
#define SEARCH_MOOV_FOUND             ( (uint32_t) ( 1 << 5 ) )
#define SEARCH_TRAK_FOUND             ( (uint32_t) ( 1 << 6 ) )
#define SEARCH_TRAK_AUDIO_FOUND       ( (uint32_t) ( 1 << 7 ) )
#define SEARCH_TRAK_AAC_FOUND         ( (uint32_t) ( 1 << 8 ) )
#define SEARCH_MDIA_FOUND             ( (uint32_t) ( 1 << 9 ) )
#define SEARCH_MINF_FOUND            ( (uint32_t) ( 1 << 10 ) )
#define SEARCH_HDLR_SOUN_FOUND       ( (uint32_t) ( 1 << 11 ) )
#define SEARCH_STSD_FOUND            ( (uint32_t) ( 1 << 12 ) )
#define SEARCH_DECSPECINFO_FOUND     ( (uint32_t) ( 1 << 13 ) )
#define SEARCH_STSC_FOUND            ( (uint32_t) ( 1 << 14 ) )
#define SEARCH_STSC_COMPLETE         ( (uint32_t) ( 1 << 15 ) )
#define SEARCH_STSZ_FOUND            ( (uint32_t) ( 1 << 16 ) )
#define SEARCH_STZ2_FOUND            ( (uint32_t) ( 1 << 17 ) )
#define SEARCH_STSIZE_COMPLETE       ( (uint32_t) ( 1 << 18 ) )
#define SEARCH_STCO_FOUND            ( (uint32_t) ( 1 << 19 ) )
#define SEARCH_CO64_FOUND            ( (uint32_t) ( 1 << 20 ) )
#define SEARCH_COFFSET_COMPLETE      ( (uint32_t) ( 1 << 21 ) )
#define SEARCH_MDAT_FOUND            ( (uint32_t) ( 1 << 22 ) )
#define SEARCH_MDAT_COMPLETE         ( (uint32_t) ( 1 << 23 ) )
#define SEARCH_STTS_FOUND            ( (uint32_t) ( 1 << 24 ) )
#define SEARCH_STTS_COMPLETE         ( (uint32_t) ( 1 << 25 ) )
/**/
#define SEARCH_RESERVED_31           ( (uint32_t) ( 1 << 31 ) )

#define SEARCH_COMPLETE_FLAGS        ( SEARCH_STSIZE_COMPLETE | SEARCH_COFFSET_COMPLETE | SEARCH_STTS_COMPLETE | SEARCH_STSC_COMPLETE)
#define SEARCH_COMPLETE(mp4p)        ( ( 0 != mp4p->mp4_audio_specific_info_size ) && ( ( mp4p->search_status & SEARCH_COMPLETE_FLAGS ) == SEARCH_COMPLETE_FLAGS ) )

/* ****************************************************************************** */
/* ****************************************************************************** */
/* PROTOTYPES                                                                     */
/* ****************************************************************************** */
/* ****************************************************************************** */
static void init_mp4parse( int8_t* err, mp4parse_t* mp4parse );
static void destroy_mp4parse( int8_t* err, mp4parse_t** mp4parse );

inline int8_t        match_mp4parse( int8_t* err, mp4parse_t* mp4parse, mp4parse_ctype_t checktype );
static inline int8_t check_ready_mp4parse( int8_t* err, mp4parse_t* mp4parse );

tsp_loop_return mp4parse_dsp_loop( tsp_loop_param data );


/* ****************************************************************************** */
/* ****************************************************************************** */
/* DSP-PROTOTYPES                                                                 */
/* ****************************************************************************** */
/* ****************************************************************************** */
static inline uint32_t fdkchmap_to_chmap( int8_t* err, CStreamInfo* info );

static uint8_t mp4_parse_box( int8_t* err, mp4parse_t* mp4parse,
                              uint32_t* idx, char* box_id, uint32_t set_flag, uint8_t skip_flag );

static uint8_t mp4_parse_box_discard( int8_t* err, mp4parse_t* mp4parse,
                                      uint32_t* idx, char* box_id, uint32_t set_flag );

static uint32_t mp4_parse_box_stts( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx );

static inline uint8_t  mp4_parse_box_AudioSampleEntry( int8_t* err, mp4parse_t* mp4parse, uint32_t idx );
static inline uint32_t mp4_parse_box_SampleSizeTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx );
static inline uint32_t mp4_parse_box_ChunkOffsetTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx );
static inline uint32_t mp4_parse_box_SampleToChunkTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx );
static inline uint8_t  mp4_parse_box_esds( int8_t* err, mp4parse_t* mp4parse, uint32_t idx );
static inline uint8_t  mp4_parse_box_mdhd( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx );

static inline void mp4_parse_obj_ESDescriptor( int8_t* err, mp4parse_t* mp4parse, uint32_t idx );
static inline void mp4_parse_obj_DecoderConfigDescriptor( int8_t* err, mp4parse_t* mp4parse, uint32_t idx );
static inline void mp4_parse_obj_DecoderSpecificInfo( int8_t* err, mp4parse_t* mp4parse, uint32_t idx );

static inline void mp4_find_audioSpecificInfo( int8_t*     err,
                                               mp4parse_t* mp4parse,
                                               uint8_t**   audio_specific_info,
                                               uint32_t*   audio_specific_info_size );

static int8_t mp4_get_sample_limits( int8_t* err, mp4parse_t* mp4parse,
                                     uint64_t* chunk_bitstream_offset,
                                     uint64_t* sample_chunk_offset,
                                     uint32_t* sample_size );

/* ****************************************************************************** */
/* ****************************************************************************** */
/* DSP                                                                            */
/* ****************************************************************************** */
/* ****************************************************************************** */

#if 0
static void mp4_dump_tables(mp4parse_t* mp4p)
{
    uint32_t i;

    if (1)
    {
        if (mp4p->mp4_sample_size_table.table_variant == 0)
        {
            if (mp4p->mp4_sample_size_table.sample_size != 0)
            {
                printf("Sample Size (stsz) Table: All samples are size %lu\n", mp4p->mp4_sample_size_table.sample_size);
            }
            else
            {
                uint32_t* tv = (uint32_t*) mp4p->mp4_sample_size_table.byte_data;

                printf("Sample Size (stsz) Table: %lu samples in track\n", mp4p->mp4_sample_size_table.sample_count);
                for (i = 0; i < mp4p->mp4_sample_size_table.sample_count; i++)
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
        uint32_t* stsc_entry_ptr = (uint32_t*)mp4p->mp4_sample_chunk_table.byte_data;

        printf("STSC Table: %lu entries\n", mp4p->mp4_sample_chunk_table.entry_count);
        for (i = 0; i < mp4p->mp4_sample_chunk_table.entry_count; i++)
        {
            printf("First chunk: %03lu, samples per chunk: %03lu, desc idx: %03lu\n", ntohl(stsc_entry_ptr[0]), ntohl(stsc_entry_ptr[1]), ntohl(stsc_entry_ptr[2]));
            stsc_entry_ptr += 3;
        }
        printf("\n");
    }

    if (1)
    {
        uint32_t* tv32 = (uint32_t*)mp4p->mp4_chunk_offs_table.byte_data;
        uint64_t* tv64 = (uint64_t*)mp4p->mp4_chunk_offs_table.byte_data;

        printf("STCO Table: %lu entries\n", mp4p->mp4_chunk_offs_table.entry_count);
        for (i = 0; i < mp4p->mp4_chunk_offs_table.entry_count; i++ )
        {
            if (mp4p->mp4_chunk_offs_table.table_variant == 0)
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
        uint32_t* stts_entry_ptr = (uint32_t*)mp4p->mp4_time_sample_table.byte_data;
        uint32_t total_delta = 0;

        printf("STTS Table: %lu entries\n", mp4p->mp4_time_sample_table.entry_count);
        for (i = 0; i < mp4p->mp4_time_sample_table.entry_count; i++)
        {
            printf("Entry %03lu:  Sample count: %03lu, sample delta: %04lu\n", i, ntohl(stts_entry_ptr[0]), ntohl(stts_entry_ptr[1]));
            total_delta += ntohl(stts_entry_ptr[0]) * ntohl(stts_entry_ptr[1]);
            stts_entry_ptr += 2;
        }
        printf("Total delta is %lu (%lu)\n", total_delta, (uint32_t)mp4p->mp4_audio_track_info.duration);
        printf("\n");
    }
}
#endif


static int mp4_initiate_start_offset(mp4parse_t* mp4p)
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

    if (mp4p->decoder == NULL || ((aacdec_t*)mp4p->decoder)->start_offset_callback == NULL)
    {
        return -1;
    }

    /*
     * We need to figure out where in the stream to start.
     * Start by converting the millisecond time offset to the proper timescale.
     */

    target_offset = (uint32_t)((double)mp4p->start_offset_ms * ((double)mp4p->mp4_audio_track_info.timescale / 1000.0));

    /*
     * We need to find sample that contains this time.
     */

    sample_number = 0;
    total_delta   = 0;
    ptr32         = (uint32_t*)mp4p->mp4_time_sample_table.byte_data;

    for (i = 0, not_found = 1; i < mp4p->mp4_time_sample_table.entry_count && not_found; i++)
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

        LOG_ERR(0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4_initiate_start_offset: sample not found (%" PRIu32 ", %" PRIu32 ")\n",
                mp4p->label, (uint32_t)target_offset, (uint32_t)total_delta);
        return -1;
    }

    /*
     * Now we need to find the chunk that contains this sample.
     */

    chunk_number = 0;
    base_sample  = 0;
    ptr32        = (uint32_t*)mp4p->mp4_sample_chunk_table.byte_data;

    for (i = 0, not_found = 1; i < mp4p->mp4_sample_chunk_table.entry_count && not_found; i++)
    {
        /*
         * Get the number of samples per chunk for this block of chunks and
         * how many chunks are in the block.
         */

        samples_per_chunk = ntohl(ptr32[1]);
        if (i + 1 < mp4p->mp4_sample_chunk_table.entry_count)
        {
            count = ntohl(ptr32[0 + 3]) - ntohl(ptr32[0]);
        }
        else
        {
            count = mp4p->mp4_chunk_offs_table.entry_count - ntohl(ptr32[0]);
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

        LOG_ERR(0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4_initiate_start_offset: chunk not found (%" PRIu32 ", %" PRIu32 ")\n",
                mp4p->label, sample_number, base_sample);
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

    if (mp4p->mp4_chunk_offs_table.table_variant == 0)
    {
        byte_offset = (uint32_t)ntohl(((uint32_t*)mp4p->mp4_chunk_offs_table.byte_data)[chunk_number]);
    }
    else
    {
        byte_offset = (uint32_t)SWAP64(((uint64_t*)mp4p->mp4_chunk_offs_table.byte_data)[chunk_number]);
    }

    sample_offset = 0;
    sample_idx    = 0;
    if (mp4p->mp4_sample_size_table.table_variant == 0)
    {
        /*
         * We need to calculate where this sample starts within the chunk.
         */

        if (mp4p->mp4_sample_size_table.sample_size != 0)
        {
            sample_offset = mp4p->mp4_sample_size_table.sample_size * (sample_number - base_sample);
        }
        else
        {
            ptr32 = (uint32_t*)mp4p->mp4_sample_size_table.byte_data;
            for (i = base_sample; i < sample_number; i++)
            {
                sample_offset += ntohl(ptr32[i]);
            }
        }
    }
    else
    {
        fsize      = mp4p->mp4_sample_size_table.field_size;            /* Sample field size in bits (4, 8, or 16)  */
        sample_idx = (fsize * base_sample) / 8;                         /* Table index in bytes                     */
        for (i = base_sample; i < sample_number; i++)
        {
            size = *(mp4p->mp4_sample_size_table.byte_data + sample_idx);
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
                    sample_offset += (size << 8) | *(mp4p->mp4_sample_size_table.byte_data + sample_idx + 1);
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

    mp4p->wait_for_stream_position = 1;
    if (((aacdec_t*)mp4p->decoder)->start_offset_callback(mp4p->decoder, ((aacdec_t*)mp4p->decoder)->player, mp4p->start_offset_ms, byte_offset))
    {
        mp4p->wait_for_stream_position = 0;
        return -1;
    }

    /*
     * The seek request was successful. Now we need to set up the table indices to match where
     * we'll begin playback.
     */

    if (mp4p->mp4_sample_size_table.table_variant == 0)
    {
        if (mp4p->mp4_sample_size_table.sample_size == 0)
        {
            mp4p->mp4_sample_size_table.sample_count_idx = sample_number;
            mp4p->mp4_sample_size_table.byte_data_idx    = sizeof(uint32_t) * sample_number;
        }
    }
    else
    {
        mp4p->mp4_sample_size_table.sample_count_idx = sample_number;
        mp4p->mp4_sample_size_table.byte_data_idx    = sample_idx;
    }

    mp4p->mp4_sample_chunk_table.entry_count_idx      = chunk_sample_table_idx;
    mp4p->mp4_sample_chunk_table.byte_data_idx        = chunk_sample_table_idx * (sizeof(uint32_t) * 3);
    mp4p->mp4_sample_chunk_table.sample_in_chunk_offs = sample_offset;
    mp4p->mp4_sample_chunk_table.sample_in_chunk_idx  = (sample_number - base_sample);

    mp4p->mp4_chunk_offs_table.entry_count_idx = chunk_number;
    if (mp4p->mp4_chunk_offs_table.table_variant == 0)
    {
        mp4p->mp4_chunk_offs_table.byte_data_idx = sizeof(uint32_t) * chunk_number;
    }
    else
    {
        mp4p->mp4_chunk_offs_table.byte_data_idx = sizeof(uint64_t) * chunk_number;
    }

    return 0;
}


static uint8_t mp4_parse_box( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx, char* box_id, uint32_t set_flag, uint8_t skip_flag )
{
    uint8_t     box_found_f = FALSE;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    uint32_t    u32_tmp = 0;

    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( ( NULL == box_id ) ||
         ( NULL == idx ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );


    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return 0;
    }

    if ( ( 0 == strncmp( id4, box_id, 4 ) ) ||
         ( 0 == strncmp( id4, "uuid", 4 ) ) )
    {
        if ( mp4p->search_status & set_flag )
        {
            LOG_ERR( mp4p->log_lvl, LOG_MEDIUM,
                     "[MP4PARSE][%s] [ERROR] mp4_parse_box, invalid stream (%c%c%c%c flag already set)\n",
                     mp4p->label,
                     box_id[ 0 ], box_id[ 1 ], box_id[ 2 ], box_id[ 3 ] );

            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        box_found_f = TRUE;

        /* mark this finding */
        if ( set_flag )
        {
            mp4p->search_status |= set_flag;
        }

        box_idx = i - 4;

        box_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + box_idx ) ) );

        LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4_parse_box, FOUND %c%c%c%c: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                 mp4p->label,
                 box_id[ 0 ], box_id[ 1 ], box_id[ 2 ], box_id[ 3 ],
                 box_size, box_idx );

        /* move past the box_id */
        mp4p->search_idx = i + 4;

        /* box extension */
        if ( box_size == 1 )
        {
            uint64_t GCC_UNUSED largesize = 0;

            largesize = ( ( (uint64_t) u32_tmp ) << 32 ) | ( ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );
            i += 8;
        }
        else if ( box_size == 0 )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box, tbd: "
                                             "found a box to the end of file!\n", mp4p->label );
        }

        if ( 0 == strncmp( box_id, "uuid", 4 ) )
        {
            char GCC_UNUSED usertype[ 16 ];
            memcpy( usertype, mp4p->data_raw + i, 16 );
            i += 16;
        }

        // skip it
        if ( ( skip_flag ) && ( mp4p->data_raw_w_idx > ( box_idx + box_size ) ) )
        {
            i = box_idx + box_size;

            mp4p->search_idx = box_idx + box_size;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box, SKIP %c%c%c%c\n",
                     mp4p->label,
                     box_id[ 0 ], box_id[ 1 ], box_id[ 2 ], box_id[ 3 ] );
        }
    }


    return box_found_f;
}


static uint8_t mp4_parse_box_discard( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx, char* box_id, uint32_t set_flag )
{
    int8_t      myerr = 0;

    uint8_t     box_found_f = 0;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == idx )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );


    if ( ( mp4p->search_status & SEARCH_DISCARD_FOUND ) &&
         ( !( mp4p->search_status & SEARCH_DISCARD_COMPLETE ) ) )
    {
        if ( ( mp4p->data_raw_w_idx - i ) >= ( mp4p->mp4_discard.byte_data_size - mp4p->mp4_discard.byte_data_idx ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, INCREMENTAL copy %" PRIu32 " (sstatus %" PRIu32 ")\n",
                     mp4p->label,
                     ( mp4p->mp4_discard.byte_data_size - mp4p->mp4_discard.byte_data_idx ),
                     ( mp4p->search_status & SEARCH_DISCARD_COMPLETE ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + ( mp4p->mp4_discard.byte_data_size - mp4p->mp4_discard.byte_data_idx );

            /* increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, s.idx %" PRIu32 " + %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx, ( mp4p->mp4_discard.byte_data_size - mp4p->mp4_discard.byte_data_idx ) );

            mp4p->search_idx += ( mp4p->mp4_discard.byte_data_size - mp4p->mp4_discard.byte_data_idx );

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, s.idx %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx );


            /* reset internals */
            mp4p->mp4_discard.byte_data_idx = 0;

            mp4p->mp4_discard.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_DISCARD_COMPLETE;

            /* clear flags since we can have zero or more of this box */
            mp4p->search_status &= ~( SEARCH_DISCARD_FOUND | SEARCH_DISCARD_COMPLETE );

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, COMPLETE! \n",
                     mp4p->label );
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, INCREMENTAL copy %" PRIu32 "\n",
                     mp4p->label, ( mp4p->data_raw_w_idx - i ) );

            mp4p->mp4_discard.byte_data_idx += ( mp4p->data_raw_w_idx - i );

            mp4p->mp4_discard.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, PARTIAL copy! %" PRIu32 " [%" PRIu32 " of %" PRIu32 "]\n",
                     mp4p->label,
                     mp4p->data_raw_w_idx - i,
                     mp4p->mp4_discard.byte_data_idx,
                     mp4p->mp4_discard.byte_data_size );

            /* */
            mp4p->data_bitstream_skip_f = TRUE;

            myerr = MP4PARSE_ERR_NEED_MORE_BITS;
        }
    }
    else if ( ( 0 == strncmp( id4, box_id, 4 ) ) ||
              ( 0 == strncmp( id4, "uuid", 4 ) ) )
    {
        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < ( *idx + 8 ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        if ( mp4p->search_status & set_flag )
        {
            LOG_ERR( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] [ERROR] mp4_parse_box_discard, "
                                             "invalid stream (%c%c%c%c flag already set)\n",
                     mp4p->label,
                     box_id[ 0 ], box_id[ 1 ], box_id[ 2 ], box_id[ 3 ] );

            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        box_found_f = TRUE;

        /* mark this finding */
        if ( set_flag )
        {
            mp4p->search_status |= set_flag;
        }

        mp4p->search_status |= SEARCH_DISCARD_FOUND;

        box_idx = i - 4;

        box_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + box_idx ) ) );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "\n[MP4PARSE][%s] mp4_parse_box_discard, FOUND %c%c%c%c: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                 mp4p->label,
                 box_id[ 0 ], box_id[ 1 ], box_id[ 2 ], box_id[ 3 ],
                 box_size, box_idx );

        box_idx = i - 4;

        /* init the box variable */
        mp4p->mp4_discard.byte_data_size       = box_size - ( i - box_idx );
        mp4p->mp4_discard.byte_data_idx        = 0;
        mp4p->mp4_discard.byte_data_complete_f = FALSE;


        if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_discard.byte_data_size ) )
        {
            mp4p->mp4_discard.byte_data_idx        = 0;
            mp4p->mp4_discard.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_DISCARD_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, COMPLETE!\n", mp4p->label );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_discard.byte_data_size + 8;

            /* increment raw_data index */
            mp4p->search_idx += mp4p->mp4_discard.byte_data_size + 8;

            /* clear flags since we can have zero or more of this box */
            mp4p->search_status &= ~( SEARCH_DISCARD_FOUND | SEARCH_DISCARD_COMPLETE );
        }
        else
        {
            mp4p->mp4_discard.byte_data_idx        = mp4p->data_raw_w_idx - i;
            mp4p->mp4_discard.byte_data_complete_f = FALSE;


            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_discard, PARTIAL_CPY!"
                                             "%" PRIu32 " n.%" PRIu32 " (raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label,
                     i, mp4p->data_raw_w_idx - i,
                     mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            // *idx = (*idx) + data_raw_w_idx-i +16;
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            mp4p->search_idx = mp4p->data_raw_w_idx;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return box_found_f;
}


static inline uint8_t mp4_parse_box_AudioSampleEntry( int8_t* err, mp4parse_t* mp4parse, uint32_t idx )
{
    int8_t              myerr = 0;

    uint32_t            i = idx;

    int32_t             box_size = 0;

    uint32_t            u32_tmp = 0;

    uint32_t            k = 0;

    uint8_t             esds_found_f = 0;


    mp4parse_t*         mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return FALSE;
    }
#endif

    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx <= ( idx + 26 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return 0;
    }

    u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
    box_size = u32_tmp;

    i += 4;
    u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    if ( 0 != strncmp( (char*) ( mp4p->data_raw + i ), "mp4a", 4 ) )
    {
        myerr = MP4PARSE_BITSTREAM_FAILURE;

        if ( NULL != err )
        {
            *err = myerr;
        }

        return FALSE;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] AudioSampleEntry: %c %c %c %c, size %" PRIu32 "\n",
             mp4p->label,
             (char) ( u32_tmp >> 24 ),
             (char) ( u32_tmp >> 16 ),
             (char) ( u32_tmp >> 8 ),
             (char) ( u32_tmp >> 0 ),
             box_size );

    /*
     * box decode and search
     */
    i += 4; // next
    i += 6; // skip se_reserved = 0

    uint16_t GCC_UNUSED se_data_reference_index = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );


    /* uint32_t reserved[0] */
    i += 2;
    u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    /* uint32_t reserved[2], skip */
    i += 8;

    /* check for AudioSampleEntry V0 or V1 */
    if ( 0 == u32_tmp )
    {
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] AudioSampleEntry: V0\n", mp4p->label );

        uint16_t GCC_UNUSED channelcount = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED samplesize = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED pre_defined = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED reserved = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        // uint32_t GCC_UNUSED samplerate = ntohl( *( (uint16_t*) ( mp4p->data_raw + i ) ) );
        uint32_t GCC_UNUSED samplerate = 0; // {default_samplerate_of_media} << 16

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] AudioSampleEntry: ch.cnt=%" PRIu16 ", bps=%" PRIu16 ", sfreq=%" PRIu32 "\n",
                 mp4p->label,
                 channelcount,
                 samplesize,
                 ( samplerate >> 16 )
               );

        if ( 0 != reserved )
        {
            myerr = MP4PARSE_BITSTREAM_FAILURE;

            if ( NULL != err )
            {
                *err = myerr;
            }

            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] AudioSampleEntry: bitstream error\n", mp4p->label );

            return FALSE;
        }


        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx <= ( idx + 4 + box_size ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* now search for 'esds' from Apple/QT atom list */
        i += 4;
        k = i;
        while ( ( k < ( box_size + idx ) ) && ( esds_found_f == 0 ) )
        {
            if ( 0 == strncmp( (char*) ( mp4p->data_raw + k ), "esds", 4 ) )
            {
                esds_found_f = 1;

                mp4_parse_box_esds( &myerr, mp4p, k - 4 );
            }

            k++;
        }
    }
    else
    {
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] AudioSampleEntry: V1\n", mp4p->label );

        uint16_t GCC_UNUSED channelcount = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED samplesize = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED pre_defined = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint16_t GCC_UNUSED reserved = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

        i += 2;
        uint32_t GCC_UNUSED samplerate = 1 << 16; // need to add SamplingRateBox() parsing

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] AudioSampleEntry: ch.cnt=%" PRIu16 ", bps=%" PRIu16 ", sfreq=%" PRIu32 "\n",
                 mp4p->label,
                 channelcount,
                 samplesize,
                 ( samplerate >> 16 )
               );

        if ( 0 != reserved )
        {
            myerr = MP4PARSE_BITSTREAM_FAILURE;

            if ( NULL != err )
            {
                *err = myerr;
            }

            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] AudioSampleEntry: bitstream error\n", mp4p->label );

            return FALSE;
        }


        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx <= ( idx + 4 + box_size ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* now search for 'esds' from Apple/QT atom list */
        i += 4;
        k = i;
        while ( ( k < ( box_size + idx ) ) && ( esds_found_f == 0 ) )
        {
            if ( 0 == strncmp( (char*) ( mp4p->data_raw + k ), "esds", 4 ) )
            {
                esds_found_f = 1;

                mp4_parse_box_esds( &myerr, mp4p, k - 4 );
            }

            k++;
        }
    }

    return TRUE;
}


static inline uint8_t mp4_parse_box_esds( int8_t* err, mp4parse_t* mp4parse, uint32_t idx )
{
    int8_t              myerr = 0;

    uint32_t            i = idx;

    int32_t GCC_UNUSED  box_size = 0;

    uint32_t            u32_tmp = 0;


    mp4parse_t*         mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif
    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( idx + 16 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return 0;
    }

    /*
     *  esds: elementary stream descriptor expand FullBox
     */

    u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
    box_size = u32_tmp;

    i += 4;
    u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
    if ( 0 != strncmp( (char*) ( mp4p->data_raw + i ), "esds", 4 ) )
    {
        myerr = MP4PARSE_BITSTREAM_FAILURE;

        if ( NULL != err )
        {
            *err = myerr;
        }

        return 0;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_esds: %c %c %c %c, size %" PRIu32 "\n",
             mp4p->label,
             (char) ( u32_tmp >> 24 ),
             (char) ( u32_tmp >> 16 ),
             (char) ( u32_tmp >> 8 ),
             (char) ( u32_tmp >> 0 ), box_size );

    i += 4;
    uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_esds:  -- esds: version %" PRIu32 "\n",
             mp4p->label, version_flags >> 24 );

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_esds:  -- esds: flags   %" PRIu32 "\n",
             mp4p->label,
             ( version_flags << 8 ) >> 8 );

    /* deep into ES_Descriptor from 14496-1, 7.2.6.5 */
    i += 4;
    mp4_parse_obj_ESDescriptor( &myerr, mp4p, i );


    if ( NULL != err )
    {
        *err = myerr;
    }

    return 1;
}


static inline uint8_t mp4_parse_box_mdhd( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx )
{
    int8_t      myerr = NO_ERR;

    uint8_t     box_found_f = FALSE;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    uint32_t    u32_tmp = 0;

    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( ( NULL == mp4parse ) || ( NULL == idx ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );

    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( *idx + 44 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return 0;
    }


    if ( ( 0 == strncmp( id4, "mdhd", 4 ) ) ||
         ( 0 == strncmp( id4, "uuid", 4 ) ) )
    {
        box_found_f = TRUE;

        box_idx = i - 4;

        box_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + box_idx ) ) );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box_mdhd, FOUND mdhd: size=%" PRIu32 "\n",
                 mp4p->label,
                 box_size );

        i += 4;

        /* move past the box_id */
        mp4p->search_idx = i;

        /* box extension */
        if ( box_size == 1 )
        {
            uint64_t GCC_UNUSED largesize = 0;

            largesize = ( ( (uint64_t) u32_tmp ) << 32 ) | ( ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );
            i += 8;
        }
        else if ( box_size == 0 )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_box, tbd: "
                                             "found a box to the end of file!\n", mp4p->label );
        }

        if ( 0 == strncmp( id4, "uuid", 4 ) )
        {
            char GCC_UNUSED usertype[ 16 ];
            memcpy( usertype, mp4p->data_raw + i, 16 );
            i += 16;
        }


        /* 14496-12 section 8.4.2.1 */
        if ( NO_ERR == myerr )
        {
            /*
             *  mdhd: media header box expand FullBox
             */
            uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

            LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4_parse_box_mdhd: version %" PRIu32 "\n",
                     mp4p->label, version_flags >> 24 );

            LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4_parse_box_mdhd: flags   %" PRIu32 "\n",
                     mp4p->label,
                     ( version_flags << 8 ) >> 8 );

            if ( ( version_flags >> 24 ) == 1 )
            {
                /* set track info values */
                i += 4;
                mp4p->mp4_audio_track_info.creation_time     = SWAP64( *( (uint64_t*) ( mp4p->data_raw + i ) ) );

                i += 8;
                mp4p->mp4_audio_track_info.modification_time = SWAP64( *( (uint64_t*) ( mp4p->data_raw + i ) ) );

                i += 8;
                mp4p->mp4_audio_track_info.timescale         = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

                i += 4;
                mp4p->mp4_audio_track_info.duration          = SWAP64( *( (uint64_t*) ( mp4p->data_raw + i ) ) );

                /* increment search index */
                mp4p->search_idx += ( i - 4 );
            }
            else if ( ( version_flags >> 24 ) == 0 )
            {
                /* set track info values */
                i += 4;
                mp4p->mp4_audio_track_info.creation_time     = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

                i += 4;
                mp4p->mp4_audio_track_info.modification_time = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

                i += 4;
                mp4p->mp4_audio_track_info.timescale         = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

                i += 4;
                mp4p->mp4_audio_track_info.duration          = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

                /* increment search index */
                mp4p->search_idx += ( i - 4 );
            }
            else
            {
                myerr = -1;
            }

            if ( NO_ERR == myerr )
            {
                LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4_parse_box_mdhd: timescale=%" PRIu32 " duration=%" PRIu64 "\n",
                         mp4p->label,
                         mp4p->mp4_audio_track_info.timescale, mp4p->mp4_audio_track_info.duration );
            }

            // skip it
            if ( mp4p->data_raw_w_idx > ( box_idx + box_size ) )
            {
                i = box_idx + box_size;

                mp4p->search_idx = box_idx + box_size;
            }
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
        return 0;
    }

    return box_found_f;
}


static inline void mp4_parse_obj_ESDescriptor( int8_t* err, mp4parse_t* mp4parse, uint32_t idx )
{
    int8_t              myerr = 0;

    uint32_t            i = idx;

    uint32_t            sizeOfInstance = 0;

    uint8_t             u8_tmp = 0;


    mp4parse_t*         mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return;
    }
#endif

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj, ESD:TAG %" PRIu32 "\n",
             mp4p->label,
             *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( idx + 18 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return;
    }


    i += 1;

    /* compute variable sizeOfInstance as in 14996-1 8.3.3 */
    u8_tmp = *( mp4p->data_raw + i );
    sizeOfInstance = ( u8_tmp & 0x7F );
    i += 1;
    while ( 0 != ( u8_tmp & 0x80 ) )
    {
        u8_tmp = *( mp4p->data_raw + i );
        sizeOfInstance = ( sizeOfInstance << 7 ) | ( u8_tmp & 0x7F );
        i += 1;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj, ESD:TAG_SIZE %" PRIu32 "\n",
             mp4p->label,
             sizeOfInstance );

    /* ES_ID */
    uint16_t GCC_UNUSED ES_ID = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );

    i += 2;
    u8_tmp = *( mp4p->data_raw + i );

    uint8_t             streamDependenceFlag = u8_tmp & 0x01;
    uint8_t             URL_Flag = ( u8_tmp >> 1 ) & 0x01;
    uint8_t             OCRstreamFlag = ( u8_tmp >> 2 ) & 0x01;
    uint8_t GCC_UNUSED  streamPriority = ( u8_tmp >> 3 );

    i += 1;
    /* optionals */
    if ( streamDependenceFlag )
    {
        uint16_t GCC_UNUSED dependsOn_ES_ID = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );
        i += 2;
    }

    if ( URL_Flag )
    {
        uint8_t GCC_UNUSED URLlength = *( mp4p->data_raw + i );
        i += 1;
        /* skip urlstring encoding! */
        i += URLlength;

        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < i )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return;
        }
    }

    if ( OCRstreamFlag )
    {
        uint16_t GCC_UNUSED OCR_ES_ID = ntohs( *( (uint16_t*) ( mp4p->data_raw + i ) ) );
        i += 2;
    }

    /* DecoderConfigDescriptor */
    mp4_parse_obj_DecoderConfigDescriptor( &myerr, mp4p, i );

    if ( NULL != err )
    {
        *err = myerr;
    }
}


static inline void mp4_parse_obj_DecoderConfigDescriptor( int8_t* err, mp4parse_t* mp4parse, uint32_t idx )
{
    int8_t              myerr = 0;

    uint32_t            i = idx;

    uint32_t            sizeOfInstance = 0;

    uint8_t             u8_tmp = 0;


    mp4parse_t*         mp4p = mp4parse;


#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return;
    }
#endif


    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DCD:TAG %" PRIu32 "\n",
             mp4p->label, *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( idx + 8 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return;
    }

    i += 1;

    /* compute variable sizeOfInstance as in 14996-1 8.3.3 */
    u8_tmp = *( mp4p->data_raw + i );
    sizeOfInstance = ( u8_tmp & 0x7F );
    i += 1;
    while ( 0 != ( u8_tmp & 0x80 ) )
    {
        u8_tmp = *( mp4p->data_raw + i );
        sizeOfInstance = ( sizeOfInstance << 7 ) | ( u8_tmp & 0x7F );
        i += 1;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DCD:TAG_SIZE %" PRIu32 "\n",
             mp4p->label, sizeOfInstance );


    uint8_t GCC_UNUSED  objectTypeIndication = *( mp4p->data_raw + i );

    i += 1;
    u8_tmp = *( mp4p->data_raw + i );

    /* check reserved==1 */
    if ( 0 == ( u8_tmp & 0x01 ) )
    {
        myerr = MP4PARSE_BITSTREAM_FAILURE;

        if ( NULL != err )
        {
            *err = myerr;
        }

        return;
    }

    uint8_t GCC_UNUSED  streamType = u8_tmp >> 2;
    uint8_t GCC_UNUSED  upStream = ( u8_tmp >> 1 ) & 0x01;


    /* compute bufferSizeDB(24) */
    i += 1;
    u8_tmp = *( mp4p->data_raw + i );
    uint32_t            bufferSizeDB = u8_tmp;

    i += 1;
    u8_tmp = *( mp4p->data_raw + i );
    bufferSizeDB = bufferSizeDB << 8 | u8_tmp;

    i += 1;
    u8_tmp = *( mp4p->data_raw + i );
    bufferSizeDB = bufferSizeDB << 8 | u8_tmp;

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DCD:bufferSizeDB %" PRIu32 "\n", mp4p->label, bufferSizeDB );


    /* compute bitrates */
    i += 1;
    uint32_t GCC_UNUSED maxBitrate = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DCD:maxBitrate %" PRIu32 "\n", mp4p->label, maxBitrate );

    i += 4;
    uint32_t GCC_UNUSED avgBitrate = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DCD:avgBitrate %" PRIu32 "\n", mp4p->label, avgBitrate );

    i += 4;
    mp4_parse_obj_DecoderSpecificInfo( &myerr, mp4p, i );


    if ( NULL != err )
    {
        *err = myerr;
    }
}


static inline void mp4_parse_obj_DecoderSpecificInfo( int8_t* err, mp4parse_t* mp4parse, uint32_t idx )
{
    int8_t      myerr = 0;

    uint32_t    i = idx;

    uint32_t    sizeOfInstance = 0;

    uint8_t     u8_tmp = 0;


    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return;
    }
#endif

    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( idx + 2 ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj DSC:TAG %" PRIu32 "\n", mp4p->label, *( (uint32_t*) ( mp4p->data_raw + i ) ) );

    i += 1;

    /* compute variable sizeOfInstance as in 14996-1 8.3.3 */
    u8_tmp = *( mp4p->data_raw + i );
    sizeOfInstance = ( u8_tmp & 0x7F );
    i += 1;
    while ( 0 != ( u8_tmp & 0x80 ) )
    {
        u8_tmp = *( mp4p->data_raw + i );
        sizeOfInstance = ( sizeOfInstance << 7 ) | ( u8_tmp & 0x7F );
        i += 1;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj  DSC:TAG_SIZE %" PRIu32 "\n", mp4p->label, sizeOfInstance );


    /* SANITY CHECK minimum num of bytes needed */
    if ( mp4p->data_raw_w_idx < ( idx + sizeOfInstance ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_NEED_MORE_BITS;
        }

        return;
    }

    if ( NULL == mp4p->mp4_audio_specific_info )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] allocation audio specific info %ld\n",
                 mp4p->label, sizeOfInstance );

        mp4p->mp4_audio_specific_info = (uint8_t*) calloc( sizeof( uint8_t ), sizeOfInstance );

        if ( NULL != mp4p->mp4_audio_specific_info )
        {
            memcpy( mp4p->mp4_audio_specific_info, ( mp4p->data_raw + i ), sizeOfInstance );
            mp4p->mp4_audio_specific_info_size = sizeOfInstance;
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] cannot allocate audio_specific_info. \n", mp4p->label );

            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4_parse_obj  DSC:INFO ", mp4p->label );
    uint8_t     k;
    for ( k = 0; ( k < mp4p->mp4_audio_specific_info_size ); k++ )
    {
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "0x%X ", *( mp4p->data_raw + i + k ) );
    }
    LOG_MSG( mp4p->log_lvl, LOG_LOW, "\n" );

    mp4p->search_status |= SEARCH_DECSPECINFO_FOUND;

    if ( NULL != err )
    {
        *err = myerr;
    }
}


static inline uint32_t mp4_parse_box_SampleSizeTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx )
{
    int8_t      myerr = 0;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    uint32_t    u32_tmp = 0;

    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == idx )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );

    if ( ( mp4p->search_status & ( SEARCH_STSZ_FOUND | SEARCH_STZ2_FOUND ) ) &&
         ( !( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) ) )
    {
        /* incremental memcpy to finish the table */
        if ( ( mp4p->data_raw_w_idx - i ) >=
             ( mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: INCREMENTAL copy %" PRIu32 " (sstatus %" PRIu32 ")\n",
                     mp4p->label,
                     ( mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx ),
                     ( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) );

            memcpy( mp4p->mp4_sample_size_table.byte_data + mp4p->mp4_sample_size_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + ( mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx );

            /* increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table s.idx %" PRIu32 " + %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx,
                     ( mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx ) );

            // mp4p->search_idx += (mp4p->mp4_sample_size_table.byte_data_size - mp4p->mp4_sample_size_table.byte_data_idx);
            mp4p->search_idx = *idx;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table s.idx %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx );


            /* reset internals */
            mp4p->mp4_sample_size_table.byte_data_idx = 0;

            mp4p->mp4_sample_size_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STSIZE_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: COMPLETE! \n", mp4p->label );
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: INCREMENTAL copy %" PRIu32 "\n",
                     mp4p->label,
                     ( mp4p->data_raw_w_idx - i ) );

            memcpy( mp4p->mp4_sample_size_table.byte_data + mp4p->mp4_sample_size_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->data_raw_w_idx - i ) );

            mp4p->mp4_sample_size_table.byte_data_idx += ( mp4p->data_raw_w_idx - i );

            mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: PARTIAL_CPY! %" PRIu32 " [%" PRIu32 " of %" PRIu32 "]\n",
                     mp4p->label,
                     mp4p->data_raw_w_idx - i,
                     mp4p->mp4_sample_size_table.byte_data_idx,
                     mp4p->mp4_sample_size_table.byte_data_size );

            /* */
            mp4p->data_bitstream_skip_f = TRUE;

            /* increment local data index */
            // *idx = (*idx) + (data_raw_w_idx-i);

            ///* increment raw_data index */
            // search_idx += (data_raw_w_idx-i);

            myerr = MP4PARSE_ERR_NEED_MORE_BITS;
        }
    }
    else if ( ( id4[ 0 ] == 's' ) &&
              ( id4[ 1 ] == 't' ) &&
              ( id4[ 2 ] == 's' ) &&
              ( id4[ 3 ] == 'z' ) )
    {
        if ( ( ( mp4p->search_status & SEARCH_STSZ_FOUND ) && ( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) ) ||
             ( ( mp4p->search_status & SEARCH_STZ2_FOUND ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] ERROR: sample_size_table: "
                                    "invalid stream (stsz flag already set)\n", mp4p->label );
            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* mark this box as found */
        mp4p->search_status |= SEARCH_STSZ_FOUND;

        /*
         * sample size table extends fullbox
         */
        u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
        box_size = u32_tmp;
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: FOUND stsz size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                 mp4p->label,
                 u32_tmp, i - 4 );

        box_idx = i - 4;

        i += 4;
        uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: version %" PRIu32 "\n",
                 mp4p->label, version_flags >> 24 );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: flags   %" PRIu32 "\n",
                 mp4p->label, ( version_flags << 8 ) >> 8 );

        i += 4;
        uint32_t            sample_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        i += 4;
        uint32_t            sample_count = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        /* sizetable raw data */
        i += 4;

        if ( NULL == mp4p->mp4_sample_size_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: allocation memory table %" PRIu32 " (4*%" PRIu32 ")\n",
                     mp4p->label,
                     box_size - ( i - box_idx ), sample_count );

            mp4p->mp4_sample_size_table.byte_data = (uint8_t*) calloc( sizeof( uint8_t ), box_size - ( i - box_idx ) );

            mp4p->mp4_sample_size_table.table_variant    = 0;
            mp4p->mp4_sample_size_table.sample_size      = sample_size;
            mp4p->mp4_sample_size_table.sample_count     = sample_count;
            mp4p->mp4_sample_size_table.sample_count_idx = 0;
            mp4p->mp4_sample_size_table.byte_data_size   = box_size - ( i - box_idx );
            mp4p->mp4_sample_size_table.byte_data_idx    = 0;

            mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;
        }

        if ( NULL == mp4p->mp4_sample_size_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: error on allocation memory table\n", mp4p->label );

            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_sample_size_table.byte_data_size ) )
        {
            memcpy( mp4p->mp4_sample_size_table.byte_data, mp4p->data_raw + i, mp4p->mp4_sample_size_table.byte_data_size );

            mp4p->mp4_sample_size_table.byte_data_idx = 0;
            mp4p->mp4_sample_size_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STSIZE_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: COMPLETE! %" PRIu32 "\n",
                     mp4p->label, version_flags >> 24 );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_sample_size_table.byte_data_size + 16;

            /* increment raw_data index */
            // mp4p->search_idx += mp4p->mp4_sample_size_table.byte_data_size +16;
            mp4p->search_idx = *idx;
        }
        else
        {
            memcpy( mp4p->mp4_sample_size_table.byte_data, mp4p->data_raw + i, mp4p->data_raw_w_idx - i );

            mp4p->mp4_sample_size_table.byte_data_idx        = mp4p->data_raw_w_idx - i;
            mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: PARTIAL_CPY! "
                                             "%" PRIu32 " n.%" PRIu32 " (raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label,
                     i, mp4p->data_raw_w_idx - i, mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            // *idx = (*idx) + data_raw_w_idx-i +16;
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );

            mp4p->search_idx = mp4p->data_raw_w_idx; // += data_raw_w_idx-i+16;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );
        }
    }
    else if ( ( id4[ 0 ] == 's' ) &&
              ( id4[ 1 ] == 't' ) &&
              ( id4[ 2 ] == 'z' ) &&
              ( id4[ 3 ] == '2' ) )
    {
        if ( ( ( mp4p->search_status & SEARCH_STZ2_FOUND ) && ( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) ) ||
             ( ( mp4p->search_status & SEARCH_STSZ_FOUND ) ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: invalid stream (stz2 flag already set)\n",
                     mp4p->label );
            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* mark this box as found */
        mp4p->search_status |= SEARCH_STZ2_FOUND;

        /*
         * sample size table extends fullbox
         */
        u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
        box_size = u32_tmp;
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: FOUND stz2: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                 mp4p->label,
                 u32_tmp, i - 4 );

        uint32_t GCC_UNUSED box_idx = i - 4;

        i += 4;
        uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:   -- stz2: version %" PRIu32 "\n",
                 mp4p->label, version_flags >> 24 );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:   -- stz2: flags   %" PRIu32 "\n",
                 mp4p->label, ( version_flags << 8 ) >> 8 );

        i += 4;
        uint32_t            field_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
        field_size = field_size & 0xFF;

        i += 4;
        uint32_t            sample_count = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        /* sizetable raw data */
        i += 4;

        if ( NULL == mp4p->mp4_sample_size_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:  -- stz2: allocation memory table %" PRIu32 " (4*%" PRIu32 ")\n",
                     mp4p->label,
                     box_size - ( i - box_idx ), sample_count );

            mp4p->mp4_sample_size_table.byte_data = (uint8_t*) calloc( sizeof( uint8_t ), box_size - ( i - box_idx ) );

            mp4p->mp4_sample_size_table.table_variant    = 1;
            mp4p->mp4_sample_size_table.field_size       = field_size;
            mp4p->mp4_sample_size_table.sample_count     = sample_count;
            mp4p->mp4_sample_size_table.sample_count_idx = 0;
            mp4p->mp4_sample_size_table.byte_data_size   = box_size - ( i - box_idx );
            mp4p->mp4_sample_size_table.byte_data_idx    = 0;

            mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;
        }

        if ( NULL == mp4p->mp4_sample_size_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:  -- stz2: "
                                             "error on allocation memory table\n", mp4p->label );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_sample_size_table.byte_data_size ) )
        {
            memcpy( mp4p->mp4_sample_size_table.byte_data, mp4p->data_raw + i, mp4p->mp4_sample_size_table.byte_data_size );

            mp4p->mp4_sample_size_table.byte_data_idx = 0;
            mp4p->mp4_sample_size_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STSIZE_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:  -- stz2: COMPLETE! %" PRIu32 "\n",
                     mp4p->label, version_flags >> 24 );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_sample_size_table.byte_data_size + 16;

            /* increment raw_data index */
            mp4p->search_idx += mp4p->mp4_sample_size_table.byte_data_size + 16;
        }
        else
        {
            memcpy( mp4p->mp4_sample_size_table.byte_data, mp4p->data_raw + i, mp4p->data_raw_w_idx - i );

            mp4p->mp4_sample_size_table.byte_data_idx = mp4p->data_raw_w_idx - i;
            mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table:  -- stz2: "
                                             "PARTIAL_CPY! %" PRIu32 " n.%" PRIu32 " (raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label,
                     i, mp4p->data_raw_w_idx - i, mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            // *idx = (*idx) + data_raw_w_idx-i +16;
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );

            mp4p->search_idx = mp4p->data_raw_w_idx; // += data_raw_w_idx-i+16;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_size_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );
        }
    }


    if ( NULL != err )
    {
        *err = myerr;
    }

    return i;
}


static inline uint32_t mp4_parse_box_ChunkOffsetTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx )
{
    int8_t      myerr = 0;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    uint32_t    u32_tmp = 0;

    uint8_t     tmp_variant = 0;

    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == idx )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif


    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );

    if ( ( mp4p->search_status & ( SEARCH_STCO_FOUND | SEARCH_CO64_FOUND ) ) &&
         ( !( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) ) )
    {
        if ( ( mp4p->data_raw_w_idx - i ) >= ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: INCREMENTAL copy %" PRIu32 " (sstatus %" PRIu32 ")\n",
                     mp4p->label,
                     ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx ),
                     ( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) );

            memcpy( mp4p->mp4_chunk_offs_table.byte_data + mp4p->mp4_chunk_offs_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx );

            /* increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: s.idx %" PRIu32 " + %" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx, ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx ) );

            mp4p->search_idx += ( mp4p->mp4_chunk_offs_table.byte_data_size - mp4p->mp4_chunk_offs_table.byte_data_idx );

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: s.idx %" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );


            /* reset internals */
            mp4p->mp4_chunk_offs_table.byte_data_idx = 0;

            mp4p->mp4_chunk_offs_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_COFFSET_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  ---COMPLETE!\n", mp4p->label );
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: INCREMENTAL copy %" PRIu32 "\n",
                     mp4p->label,
                     ( mp4p->data_raw_w_idx - i ) );

            memcpy( mp4p->mp4_chunk_offs_table.byte_data + mp4p->mp4_chunk_offs_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->data_raw_w_idx - i ) );

            mp4p->mp4_chunk_offs_table.byte_data_idx += ( mp4p->data_raw_w_idx - i );

            mp4p->mp4_chunk_offs_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  ---PARTIAL_CPY! %" PRIu32 " [%" PRIu32 " of %" PRIu32 "]\n",
                     mp4p->label,
                     mp4p->data_raw_w_idx - i,
                     mp4p->mp4_chunk_offs_table.byte_data_idx,
                     mp4p->mp4_chunk_offs_table.byte_data_size );

            /* */
            mp4p->data_bitstream_skip_f = TRUE;

            /* increment local data index */
            // *idx = (*idx) + (data_raw_w_idx-i);

            ///* increment raw_data index */
            // search_idx += (data_raw_w_idx-i);

            myerr = MP4PARSE_ERR_NEED_MORE_BITS;
        }
    }
    else
    {
        if ( ( id4[ 0 ] == 's' ) &&
             ( id4[ 1 ] == 't' ) &&
             ( id4[ 2 ] == 'c' ) &&
             ( id4[ 3 ] == 'o' ) )
        {
            if ( ( ( mp4p->search_status & SEARCH_STCO_FOUND ) && ( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) ) ||
                 ( ( mp4p->search_status & SEARCH_CO64_FOUND ) ) )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] chunk_offset_table:  invalid stream (stco flag already set)\n", mp4p->label );
                *err = MP4PARSE_ERR_UNKNOWN;
                return 0;
            }

            /* SANITY CHECK minimum num of bytes needed */
            if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
            {
                if ( NULL != err )
                {
                    *err = MP4PARSE_ERR_NEED_MORE_BITS;
                }

                return 0;
            }

            /* mark this box as found */
            mp4p->search_status |= SEARCH_STCO_FOUND;

            tmp_variant = 0;

            /*
             * sample size table extends fullbox
             */
            u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
            box_size = u32_tmp;
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: FOUND stco: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                     mp4p->label,
                     u32_tmp, i - 4 );
        }
        else if ( ( id4[ 0 ] == 'c' ) &&
                  ( id4[ 1 ] == 'o' ) &&
                  ( id4[ 2 ] == '6' ) &&
                  ( id4[ 3 ] == '4' ) )
        {
            if ( ( ( mp4p->search_status & SEARCH_CO64_FOUND ) && ( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) ) ||
                 ( ( mp4p->search_status & SEARCH_STCO_FOUND ) ) )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] chunk_offset_table: invalid stream (co64 flag already set)\n", mp4p->label );
                *err = MP4PARSE_ERR_UNKNOWN;
                return 0;
            }

            /* SANITY CHECK minimum num of bytes needed */
            if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
            {
                if ( NULL != err )
                {
                    *err = MP4PARSE_ERR_NEED_MORE_BITS;
                }

                return 0;
            }

            /* mark this box as found */
            mp4p->search_status |= SEARCH_CO64_FOUND;

            tmp_variant = 1;

            /*
             * sample size table extends fullbox
             */
            u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
            box_size = u32_tmp;
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: FOUND co64: size=%" PRIu32 " (s.idx %" PRIu32 ")\n", mp4p->label, u32_tmp, i - 4 );
        }

        box_idx = i - 4;

        i += 4;
        uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  -- stcoffset: version %" PRIu32 "\n",
                 mp4p->label,
                 version_flags >> 24 );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:   -- stcoffset: flags   %" PRIu32 "\n",
                 mp4p->label,
                 ( version_flags << 8 ) >> 8 );

        i += 4;
        uint32_t            entry_count = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        /* sizetable raw data */
        i += 4;

        if ( NULL == mp4p->mp4_chunk_offs_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  -- stcoffset: allocation memory "
                                             "table %" PRIu32 " (v=%" PRIu8 " entry=%" PRIu32 ")\n",
                     mp4p->label,
                     box_size - ( i - box_idx ), tmp_variant, entry_count );

            mp4p->mp4_chunk_offs_table.byte_data = (uint8_t*) calloc( sizeof( uint8_t ), box_size - ( i - box_idx ) );

            mp4p->mp4_chunk_offs_table.table_variant   = tmp_variant;
            mp4p->mp4_chunk_offs_table.entry_count     = entry_count;
            mp4p->mp4_chunk_offs_table.entry_count_idx = 0;
            mp4p->mp4_chunk_offs_table.byte_data_size  = box_size - ( i - box_idx );
            mp4p->mp4_chunk_offs_table.byte_data_idx   = 0;

            mp4p->mp4_chunk_offs_table.byte_data_complete_f = FALSE;
        }

        if ( NULL == mp4p->mp4_chunk_offs_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  -- stoffset: error on allocation memory table\n",
                     mp4p->label );

            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_chunk_offs_table.byte_data_size ) )
        {
            memcpy( mp4p->mp4_chunk_offs_table.byte_data, mp4p->data_raw + i, mp4p->mp4_chunk_offs_table.byte_data_size );

            mp4p->mp4_chunk_offs_table.byte_data_idx = 0;
            mp4p->mp4_chunk_offs_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_COFFSET_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  -- stcoffset: COMPLETE! %" PRIu32 "\n",
                     mp4p->label, version_flags >> 24 );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_chunk_offs_table.byte_data_size + 16;

            /* increment raw_data index */
            mp4p->search_idx += mp4p->mp4_chunk_offs_table.byte_data_size + 16;
        }
        else
        {
            memcpy( mp4p->mp4_chunk_offs_table.byte_data, mp4p->data_raw + i, mp4p->data_raw_w_idx - i );

            mp4p->mp4_chunk_offs_table.byte_data_idx = mp4p->data_raw_w_idx - i;
            mp4p->mp4_chunk_offs_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table:  -- stcoffset: PARTIAL_CPY! "
                                             "%" PRIu32 " n.%" PRIu32 " (raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label,
                     i, mp4p->data_raw_w_idx - i, mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            // *idx = (*idx) + data_raw_w_idx-i +16;
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );

            mp4p->search_idx = mp4p->data_raw_w_idx; // += data_raw_w_idx-i+16;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] chunk_offset_table: partial_cpy search_idx=%" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx );
        }
    }


    if ( NULL != err )
    {
        *err = myerr;
    }

    return i;
}


static inline uint32_t mp4_parse_box_SampleToChunkTable( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx )
{
    int8_t      myerr = 0;

    char*       id4 = NULL;

    uint32_t    i = 0;

    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    uint32_t    u32_tmp = 0;

    // uint8_t tmp_variant =0;


    mp4parse_t* mp4p = mp4parse;

#ifdef DEBUG
    if ( NULL == idx )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );

    if ( ( mp4p->search_status & SEARCH_STSC_FOUND ) &&
         ( !( mp4p->search_status & SEARCH_STSC_COMPLETE ) ) )
    {
        if ( ( mp4p->data_raw_w_idx - i ) >= ( mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: INCREMENTAL copy %" PRIu32 " (sstatus %" PRIu32 ")\n",
                     mp4p->label,
                     ( mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx ),
                     ( mp4p->search_status & SEARCH_STSC_COMPLETE ) );

            memcpy( mp4p->mp4_sample_chunk_table.byte_data + mp4p->mp4_sample_chunk_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + ( mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx );

            /* increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: s.idx %" PRIu32 " + %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx,
                     ( mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx ) );

            // mp4p->search_idx += (mp4p->mp4_sample_chunk_table.byte_data_size - mp4p->mp4_sample_chunk_table.byte_data_idx);
            mp4p->search_idx = *idx;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: s.idx %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->search_idx );


            /* reset internals */
            mp4p->mp4_sample_chunk_table.byte_data_idx        = 0;
            mp4p->mp4_sample_chunk_table.sample_in_chunk_idx  = 0;
            mp4p->mp4_sample_chunk_table.sample_in_chunk_offs = 0;

            mp4p->mp4_sample_chunk_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STSC_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- sample_chunk_table: ---COMPLETE! \n",
                     mp4p->label );
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- sample_chunk_table: INCREMENTAL copy "
                                             "%" PRIu32 "\n", mp4p->label, ( mp4p->data_raw_w_idx - i ) );

            memcpy( mp4p->mp4_sample_chunk_table.byte_data + mp4p->mp4_sample_chunk_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->data_raw_w_idx - i ) );

            mp4p->mp4_sample_chunk_table.byte_data_idx += ( mp4p->data_raw_w_idx - i );

            mp4p->mp4_sample_chunk_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- sample_chunk_table: ---PARTIAL_CPY! "
                                             "%" PRIu32 " [%" PRIu32 " of %" PRIu32 "]\n",
                     mp4p->label,
                     mp4p->data_raw_w_idx - i,
                     mp4p->mp4_sample_chunk_table.byte_data_idx,
                     mp4p->mp4_sample_chunk_table.byte_data_size );

            /* */
            mp4p->data_bitstream_skip_f = TRUE;

            myerr = MP4PARSE_ERR_NEED_MORE_BITS;
        }
    }
    else if ( ( id4[ 0 ] == 's' ) &&
              ( id4[ 1 ] == 't' ) &&
              ( id4[ 2 ] == 's' ) &&
              ( id4[ 3 ] == 'c' ) )
    {
        if ( ( mp4p->search_status & SEARCH_STSC_FOUND ) && ( mp4p->search_status & SEARCH_STSC_COMPLETE ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] sample_chunk_table: invalid stream (stsc flag already set)\n",
                     mp4p->label );

            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* mark this box as found */
        mp4p->search_status |= SEARCH_STSC_FOUND;

        /*
         * sample to chunk table extends fullbox
         */
        u32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
        box_size = u32_tmp;
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: FOUND stsc: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                 mp4p->label,
                 u32_tmp, i - 4 );

        box_idx = i - 4;

        i += 4;
        uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: version %" PRIu32 "\n",
                 mp4p->label,
                 version_flags >> 24 );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: flags   %" PRIu32 "\n",
                 mp4p->label,
                 ( version_flags << 8 ) >> 8 );

        i += 4;
        uint32_t            entry_count = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        /* sizetable raw data */
        i += 4;

        if ( NULL == mp4p->mp4_sample_chunk_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: allocation memory table %" PRIu32 " ( entry=%" PRIu32 ")\n",
                     mp4p->label,
                     box_size - ( i - box_idx ), entry_count );

            mp4p->mp4_sample_chunk_table.byte_data = (uint8_t*) calloc( sizeof( uint8_t ), box_size - ( i - box_idx ) );

            mp4p->mp4_sample_chunk_table.entry_count     = entry_count;
            mp4p->mp4_sample_chunk_table.entry_count_idx = 0;
            mp4p->mp4_sample_chunk_table.byte_data_size  = box_size - ( i - box_idx );
            mp4p->mp4_sample_chunk_table.byte_data_idx   = 0;

            mp4p->mp4_sample_chunk_table.byte_data_complete_f = FALSE;
        }

        if ( NULL == mp4p->mp4_sample_chunk_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: error on allocation stsc memory table\n",
                     mp4p->label );

            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_sample_chunk_table.byte_data_size ) )
        {
            memcpy( mp4p->mp4_sample_chunk_table.byte_data, mp4p->data_raw + i, mp4p->mp4_sample_chunk_table.byte_data_size );

            mp4p->mp4_sample_chunk_table.byte_data_idx        = 0;
            mp4p->mp4_sample_chunk_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STSC_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: COMPLETE! %" PRIu32 "\n",
                     mp4p->label, version_flags >> 24 );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_sample_chunk_table.byte_data_size + 16;

            /* increment raw_data index */
            //            mp4p->search_idx += mp4p->mp4_sample_chunk_table.byte_data_size +16;

            mp4p->search_idx = *idx;
        }
        else
        {
            memcpy( mp4p->mp4_sample_chunk_table.byte_data, mp4p->data_raw + i, mp4p->data_raw_w_idx - i );

            mp4p->mp4_sample_chunk_table.byte_data_idx        = mp4p->data_raw_w_idx - i;
            mp4p->mp4_sample_chunk_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: -- stsc: PARTIAL_CPY! %" PRIu32 " n.%" PRIu32 " "
                                                                                                                               "(raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label,
                     i, mp4p->data_raw_w_idx - i, mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            // *idx = (*idx) + data_raw_w_idx-i +16;
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: partial_cpy search_idx=%" PRIu32 "\n", mp4p->label, mp4p->search_idx );

            mp4p->search_idx = mp4p->data_raw_w_idx; // += data_raw_w_idx-i+16;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] sample_chunk_table: partial_cpy search_idx=%" PRIu32 "\n", mp4p->label, mp4p->search_idx );
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return i;
}


static uint32_t mp4_parse_box_stts( int8_t* err, mp4parse_t* mp4parse, uint32_t* idx )
{
    int8_t      myerr = 0;
    char*       id4 = NULL;
    uint32_t    i = 0;
    uint32_t    box_size = 0;
    uint32_t    box_idx = 0;

    mp4parse_t* mp4p = mp4parse;

    /* init pointers */
    i = *idx;

    id4 = (char*) ( mp4p->data_raw + i );

    if ( ( mp4p->search_status & SEARCH_STTS_FOUND ) && ( !( mp4p->search_status & SEARCH_STTS_COMPLETE ) ) )
    {
        if ( ( mp4p->data_raw_w_idx - i ) >= ( mp4p->mp4_time_sample_table.byte_data_size - mp4p->mp4_time_sample_table.byte_data_idx ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: INCREMENTAL copy %" PRIu32 " (sstatus %" PRIu32 ")\n",
                     mp4p->label,
                     ( mp4p->mp4_time_sample_table.byte_data_size - mp4p->mp4_time_sample_table.byte_data_idx ),
                     ( mp4p->search_status & SEARCH_STTS_COMPLETE ) );

            memcpy( mp4p->mp4_time_sample_table.byte_data + mp4p->mp4_time_sample_table.byte_data_idx,
                    mp4p->data_raw + i,
                    ( mp4p->mp4_time_sample_table.byte_data_size - mp4p->mp4_time_sample_table.byte_data_idx ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + ( mp4p->mp4_time_sample_table.byte_data_size - mp4p->mp4_time_sample_table.byte_data_idx );

            /* increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: s.idx %" PRIu32 " + %" PRIu32 "\n",
                     mp4p->label, mp4p->search_idx, ( mp4p->mp4_time_sample_table.byte_data_size - mp4p->mp4_time_sample_table.byte_data_idx ) );

            mp4p->search_idx = *idx;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: s.idx %" PRIu32 "\n", mp4p->label, mp4p->search_idx );

            /* reset internals */
            mp4p->mp4_time_sample_table.byte_data_idx        = 0;
            mp4p->mp4_time_sample_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STTS_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- time_sample_table: ---COMPLETE!\n", mp4p->label );
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- time_sample_table: INCREMENTAL copy %" PRIu32 "\n", mp4p->label, ( mp4p->data_raw_w_idx - i ) );

            memcpy( mp4p->mp4_time_sample_table.byte_data + mp4p->mp4_time_sample_table.byte_data_idx, mp4p->data_raw + i, ( mp4p->data_raw_w_idx - i ) );

            mp4p->mp4_time_sample_table.byte_data_idx += ( mp4p->data_raw_w_idx - i );

            mp4p->mp4_time_sample_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- time_sample_table: ---PARTIAL_CPY! %" PRIu32 " [%" PRIu32 " of %" PRIu32 "]\n",
                     mp4p->label, mp4p->data_raw_w_idx - i, mp4p->mp4_time_sample_table.byte_data_idx, mp4p->mp4_time_sample_table.byte_data_size );

            /* */
            mp4p->data_bitstream_skip_f = TRUE;

            myerr = MP4PARSE_ERR_NEED_MORE_BITS;
        }
    }
    else if ( ( id4[ 0 ] == 's' ) &&
              ( id4[ 1 ] == 't' ) &&
              ( id4[ 2 ] == 't' ) &&
              ( id4[ 3 ] == 's' ) )
    {
        if ( ( mp4p->search_status & SEARCH_STTS_FOUND ) && ( mp4p->search_status & SEARCH_STTS_COMPLETE ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] time_sample_table: invalid stream (stts flag already set)\n", mp4p->label );

            *err = MP4PARSE_ERR_UNKNOWN;
            return 0;
        }

        /* SANITY CHECK minimum num of bytes needed */
        if ( mp4p->data_raw_w_idx < ( *idx + 16 ) )
        {
            if ( NULL != err )
            {
                *err = MP4PARSE_ERR_NEED_MORE_BITS;
            }

            return 0;
        }

        /* mark this box as found */
        mp4p->search_status |= SEARCH_STTS_FOUND;

        /*
         * time to sample table extends fullbox
         */

        box_size = ntohl( *( (uint32_t*) ( mp4p->data_raw + i - 4 ) ) );
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: FOUND stts: size=%" PRIu32 " (s.idx %" PRIu32 ")\n", mp4p->label, box_size, i - 4 );

        box_idx = i - 4;

        i += 4;
        uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stts: version %" PRIu32 "\n", mp4p->label, version_flags >> 24 );

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stts: flags   %" PRIu32 "\n", mp4p->label, ( version_flags << 8 ) >> 8 );

        i += 4;
        uint32_t            entry_count = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );

        /* sizetable raw data */
        i += 4;

        if ( NULL == mp4p->mp4_time_sample_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stts: allocation memory table %" PRIu32 " ( entry=%" PRIu32 ")\n",
                     mp4p->label, box_size - ( i - box_idx ), entry_count );

            mp4p->mp4_time_sample_table.byte_data = (uint8_t*) calloc( sizeof( uint8_t ), box_size - ( i - box_idx ) );

            mp4p->mp4_time_sample_table.entry_count     = entry_count;
            mp4p->mp4_time_sample_table.byte_data_size  = box_size - ( i - box_idx );
            mp4p->mp4_time_sample_table.byte_data_idx   = 0;

            mp4p->mp4_time_sample_table.byte_data_complete_f = FALSE;
        }

        if ( NULL == mp4p->mp4_time_sample_table.byte_data )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stsc: error on allocation stts memory table\n", mp4p->label );

            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else if ( mp4p->data_raw_w_idx >= ( i + mp4p->mp4_time_sample_table.byte_data_size ) )
        {
            memcpy( mp4p->mp4_time_sample_table.byte_data, mp4p->data_raw + i, mp4p->mp4_time_sample_table.byte_data_size );

            mp4p->mp4_time_sample_table.byte_data_idx        = 0;
            mp4p->mp4_time_sample_table.byte_data_complete_f = TRUE;

            mp4p->search_status |= SEARCH_STTS_COMPLETE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stts: COMPLETE! %" PRIu32 "\n", mp4p->label, version_flags >> 24 );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = ( *idx ) + mp4p->mp4_time_sample_table.byte_data_size + 16;

            /* increment raw_data index */
            mp4p->search_idx = *idx;
        }
        else
        {
            memcpy( mp4p->mp4_time_sample_table.byte_data, mp4p->data_raw + i, mp4p->data_raw_w_idx - i );

            mp4p->mp4_time_sample_table.byte_data_idx        = mp4p->data_raw_w_idx - i;
            mp4p->mp4_time_sample_table.byte_data_complete_f = FALSE;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: -- stsc: PARTIAL_CPY! %" PRIu32 " n.%" PRIu32 " "
                     "(raw_idx=%" PRIu32 ", i=%" PRIu32 ") (1ST %" PRIu32 ")\n",
                     mp4p->label, i, mp4p->data_raw_w_idx - i, mp4p->data_raw_w_idx, i, ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) ) );

            /* */
            mp4p->data_bitstream_skip_f = FALSE;

            /* increment local data index */
            *idx = mp4p->data_raw_w_idx;

            // increment raw_data index */
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: partial_cpy search_idx=%" PRIu32 "\n", mp4p->label, mp4p->search_idx );

            mp4p->search_idx = mp4p->data_raw_w_idx;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] time_sample_table: partial_cpy search_idx=%" PRIu32 "\n", mp4p->label, mp4p->search_idx );
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return i;
}


static inline void mp4_find_audioSpecificInfo( int8_t* err,
                                               mp4parse_t* mp4parse,
                                               uint8_t** audio_specific_info, uint32_t* audio_specific_info_size )
{
    int8_t      myerr = 0;
    int32_t     i32_tmp = 0;
    uint8_t     u8_tmp  = 0;
    int32_t     offs = 0;
    uint32_t    i;
    uint32_t    last_i;
    char*       id4;
    int32_t     box_size = 0;
    mp4parse_t* mp4p = mp4parse;
    uint8_t     search_complete_f;

    /*
     * start from where we left
     */

    i      = mp4p->search_idx;
    last_i = i;

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: start idx=%" PRIu32 " (raw_idx=%" PRIu32 ")\n",
             mp4p->label,
             i,
             mp4p->data_raw_w_idx );

    /*
     * loop over raw stream
     */

    search_complete_f = SEARCH_COMPLETE(mp4p);
    while ( ( i < ( mp4p->data_raw_w_idx - 4 ) ) && ( 0 == myerr ) && ( !search_complete_f ) )
    {
        id4 = (char*) ( mp4p->data_raw + i );

        /* FTYP */
        if ( ( id4[ 0 ] == 'f' ) &&
             ( id4[ 1 ] == 't' ) &&
             ( id4[ 2 ] == 'y' ) &&
             ( id4[ 3 ] == 'p' ) )
        {
            if ( mp4p->search_status & SEARCH_FTYP_FOUND )
            {
                LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] audioSpecificInfo: invalid stream (multiple ftyp)\n",
                         mp4p->label );
                myerr = MP4PARSE_ERR_UNKNOWN;
                return;
            }

            /* SANITY CHECK minimum num of bytes needed */
            if ( mp4p->data_raw_w_idx < ( i + 16 ) )
            {
                if ( NULL != err )
                {
                    *err = MP4PARSE_ERR_NEED_MORE_BITS;
                }

                return;
            }

            /* mark this finding */
            mp4p->search_status |= SEARCH_FTYP_FOUND;

            /*
             * decode box
             */
            offs = -4;
            i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
            box_size = i32_tmp;

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: FOUND ftyp: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                     mp4p->label,
                     box_size, i - 4 );

            if ( mp4p->data_raw_w_idx < ( i + box_size ) )
            {
                if ( NULL != err )
                {
                    *err = MP4PARSE_ERR_NEED_MORE_BITS;
                }

                return;
            }

            offs = 4;
            if ( ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "M4A ", 4 ) ) ||
                 ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "m4a ", 4 ) ) ||
                 ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "mp42", 4 ) ) ||
                 ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "MP42", 4 ) ) )
            {
                mp4p->search_status |= SEARCH_FTYP_AUDIO_FOUND;
            }

            uint32_t box_idx = i - 4;
            mp4p->search_idx = i + 4;

            /* parse content */
            i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:   mjr_brand: %c %c %c %c\n",
                     mp4p->label,
                     (char) ( i32_tmp >> 24 ),
                     (char) ( i32_tmp >> 16 ),
                     (char) ( i32_tmp >> 8 ),
                     (char) ( i32_tmp >> 0 ) );

            offs += 4;
            i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
            if ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "isom", 4 ) )
            {
                mp4p->search_status |= SEARCH_FTYP_ISO_FOUND;
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:   ISO file!\n", mp4p->label );
            }

            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:   min_brand: %c %c %c %c\n",
                     mp4p->label,
                     (char) ( i32_tmp >> 24 ),
                     (char) ( i32_tmp >> 16 ),
                     (char) ( i32_tmp >> 8 ),
                     (char) ( i32_tmp >> 0 ) );

            offs += 4;
            while ( offs < ( box_size - 4 ) )
            {
                i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
                if ( 0 == strncmp( (char*) ( mp4p->data_raw + i + offs ), "isom", 4 ) )
                {
                    mp4p->search_status |= SEARCH_FTYP_ISO_FOUND;
                }

                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:   comp_brand: %c %c %c %c\n",
                         mp4p->label,
                         (char) ( i32_tmp >> 24 ),
                         (char) ( i32_tmp >> 16 ),
                         (char) ( i32_tmp >> 8 ),
                         (char) ( i32_tmp >> 0 ) );

                offs += 4;
            }

            // jump box payload if we have enough data for it
            if ( mp4p->data_raw_w_idx > ( box_idx + box_size ) )
            {
                mp4p->search_idx = box_idx + box_size;
            }
        } /*ftyp*/

        /* MOOV */
        u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "moov", SEARCH_MOOV_FOUND, FALSE );
        if ( u8_tmp && ( !( mp4p->search_status & SEARCH_FTYP_FOUND ) ) )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] [WARNING] audioSpecificInfo: not an iso file (moov without ftyp)\n",
                     mp4p->label );
        }

        /* DISCARD variable-size boxes before moov: mdat */
        id4 = (char*) ( mp4p->data_raw + i );
        if ( ( 0 == strncmp( id4, "mdat", 4 ) ) ||
             ( 0 == strncmp( id4, "free", 4 ) ) ||
             ( 0 == strncmp( id4, "pdin", 4 ) ) ||
             ( ( mp4p->search_status & SEARCH_DISCARD_FOUND ) &&
               ( !( mp4p->search_status & SEARCH_DISCARD_COMPLETE ) ) ) )
        {
            /* we need to mark for MDAT skipping, it means the MOOV map is at the end */
            if ( 0 == strncmp( id4, "mdat", 4 ) )
            {
                u8_tmp = mp4_parse_box_discard( &myerr, mp4p, &i, id4, SEARCH_MDAT_FOUND );
            }
            else
            {
                u8_tmp = mp4_parse_box_discard( &myerr, mp4p, &i, id4, SEARCH_DISCARD_FOUND );
            }
        }

        if ( mp4p->search_status & SEARCH_MOOV_FOUND )
        {
            /* MVHD */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "mvhd", SEARCH_EMPTY_FLAG, TRUE );
            /* TRAK */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "trak", SEARCH_TRAK_FOUND, FALSE );
        }

        if ( mp4p->search_status & SEARCH_TRAK_FOUND )
        {
            /* TKHD */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "tkhd", SEARCH_EMPTY_FLAG, TRUE );
            /* MDIA */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "mdia", SEARCH_MDIA_FOUND, FALSE );
        }

        if ( mp4p->search_status & SEARCH_MDIA_FOUND )
        {
            /* MDHD */
            u8_tmp = mp4_parse_box_mdhd( &myerr, mp4p, &i );
        }

        if ( ( mp4p->search_status & SEARCH_MDIA_FOUND ) &&
             ( !( mp4p->search_status & SEARCH_HDLR_SOUN_FOUND ) ) )
        {
            /* MINF */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "minf", SEARCH_MINF_FOUND, FALSE );

            /* HDLR */
            id4 = (char*) ( mp4p->data_raw + i );

            if ( ( id4[ 0 ] == 'h' ) &&
                 ( id4[ 1 ] == 'd' ) &&
                 ( id4[ 2 ] == 'l' ) &&
                 ( id4[ 3 ] == 'r' ) )
            {
                /*
                 * media handler: extends fullbox
                 */
                offs = -4;
                i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
                box_size = i32_tmp;
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: FOUND hdlr: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                         mp4p->label, i32_tmp, i - 4 );

                uint32_t            box_idx = i - 4;

                i += 4;
                uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- hdlr: version %" PRIu32 "\n",
                         mp4p->label, version_flags >> 24 );

                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- hdlr: flags   %" PRIu32 "\n",
                         mp4p->label, ( version_flags << 8 ) >> 8 );

                i += 4;
                uint32_t GCC_UNUSED pre_defined = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- hdlr: pre_defined %" PRIu32 "\n",
                         mp4p->label, pre_defined );

                i += 4;
                uint32_t GCC_UNUSED handler_type = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- hdlr: type %c%c%c%c\n",
                         mp4p->label,
                         (char) ( handler_type >> 24 ),
                         (char) ( handler_type >> 16 ),
                         (char) ( handler_type >> 8 ),
                         (char) ( handler_type >> 0 ) );

                if ( 0 == strncmp( (char*) ( mp4p->data_raw + i ), "soun", 4 ) )
                {
                    mp4p->search_status |= SEARCH_HDLR_SOUN_FOUND;
                }
                else
                {
                    /* not an audio track, clear flags and search for the next one */
                    mp4p->search_status &= ( SEARCH_FTYP_FOUND |
                                             SEARCH_FTYP_AUDIO_FOUND |
                                             SEARCH_FTYP_ISO_FOUND |
                                             SEARCH_MOOV_FOUND );

                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: LOG: this is not an audio track (%c%c%c%c),"
                                                     "skip & search the next one.\n",
                             mp4p->label,
                             *( (char*) ( mp4p->data_raw + i + 0 ) ),
                             *( (char*) ( mp4p->data_raw + i + 1 ) ),
                             *( (char*) ( mp4p->data_raw + i + 2 ) ),
                             *( (char*) ( mp4p->data_raw + i + 3 ) ) );

                    // skip it
                    if ( mp4p->data_raw_w_idx > ( box_idx + box_size ) )
                    {
                        mp4p->search_idx = box_idx + box_size;
                        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  SKIP hdlr\n", mp4p->label );
                    }
                }

                mp4p->search_idx = i + 4;
            } /*hdlr*/
        }

        if ( mp4p->search_status & ( SEARCH_MINF_FOUND | SEARCH_HDLR_SOUN_FOUND ) )
        {
            /* SMHD */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "smhd", SEARCH_EMPTY_FLAG, TRUE );
            /* DINF */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "dinf", SEARCH_EMPTY_FLAG, TRUE );
            /* STBL */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "stbl", SEARCH_EMPTY_FLAG, FALSE );
            /* HDLR */
            u8_tmp = mp4_parse_box( &myerr, mp4p, &i, "hdlr", SEARCH_EMPTY_FLAG, TRUE );

            /* STSD */
            id4 = (char*) ( mp4p->data_raw + i );

            if ( ( id4[ 0 ] == 's' ) &&
                 ( id4[ 1 ] == 't' ) &&
                 ( id4[ 2 ] == 's' ) &&
                 ( id4[ 3 ] == 'd' ) )
            {
                /*
                 * decode box
                 */
                offs = -4;
                i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i + offs ) ) );
                box_size = i32_tmp;

                /* stds is the box we want to fully decode */
                /* return&wait until we have it all */
                if ( mp4p->data_raw_w_idx < ( i - 4 + box_size ) )
                {
                    myerr = MP4PARSE_ERR_NEED_MORE_BITS;
                    return;
                }

                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: FOUND stsd: size=%" PRIu32 " (s.idx %" PRIu32 ")\n",
                         mp4p->label,
                         i32_tmp, i - 4 );

                /* mark this finding */
                mp4p->search_status |= SEARCH_STSD_FOUND;

                uint32_t GCC_UNUSED box_idx = i - 4;
                mp4p->search_idx = i + 4;

                /* check here for optional box_extend and uuid */

                /* parse inside */
                i += 4;
                uint32_t GCC_UNUSED version_flags = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- stds: version %" PRIu32 "\n",
                         mp4p->label,
                         version_flags >> 24 );

                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- stds: flags   %" PRIu32 "\n",
                         mp4p->label,
                         ( version_flags << 8 ) >> 8 );


                // i+=4;
                // int32_t fbox_i = ntohl(*((uint32_t*)(data_raw+i)));

                i += 4;
                uint32_t            fbox_entry_cnt = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- stds: entry_cnt %" PRIu32 "\n",
                         mp4p->label, fbox_entry_cnt );

                uint32_t            k = 0;
                i += 4;

                for ( k = 0; k <= fbox_entry_cnt; k += 4 )
                {
                    i32_tmp = ntohl( *( (uint32_t*) ( mp4p->data_raw + i ) ) );
                    i += 4;
                    id4 = (char*) ( mp4p->data_raw + i );
                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo:  -- FOUND sub-esds: %c%c%c%c, size %" PRIu32 "  \n",
                             mp4p->label,
                             (char) id4[ 0 ],
                             (char) id4[ 1 ],
                             (char) id4[ 2 ],
                             (char) id4[ 3 ],
                             i32_tmp );

                    if ( 0 == strncmp( (char*) ( mp4p->data_raw + i ), "mp4a", 4 ) )
                    {
                        mp4p->search_status |= SEARCH_TRAK_AAC_FOUND;

                        mp4_parse_box_AudioSampleEntry( &myerr, mp4p, i - 4 );
                    }

                    i += i32_tmp - 4;
                }
            } /*stsd*/


            /* SAMPLE TO CHUNK BOX */
            id4 = (char*) ( mp4p->data_raw + i );
            if ( ( 0 == strncmp( id4, "stsc", 4 ) ) ||
                 ( ( mp4p->search_status & SEARCH_STSC_FOUND ) &&
                   ( !( mp4p->search_status & SEARCH_STSC_COMPLETE ) ) ) )
            {
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: stsc i=%" PRIu32 " (raw_idx=%" PRIu32 ")"
                         "(bitstream_idx=%" PRIu64 ") (err=%" PRIu8 ")\n",
                         mp4p->label,
                         i, mp4p->data_raw_w_idx, mp4p->data_bitstream_idx, myerr );

                mp4_parse_box_SampleToChunkTable( &myerr, mp4p, &i );
            }
            /* SAMPLE SIZE BOX */
            id4 = (char*) ( mp4p->data_raw + i );
            if ( ( 0 == strncmp( id4, "stz2", 4 ) ) ||
                 ( 0 == strncmp( id4, "stsz", 4 ) ) ||
                 ( ( mp4p->search_status & ( SEARCH_STSZ_FOUND | SEARCH_STZ2_FOUND ) ) &&
                   ( !( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) ) )
               )
            {
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: stsz i=%" PRIu32 " (raw_idx=%" PRIu32 ")"
                         "(bitstream_idx=%" PRIu64 ") (err=%" PRIu8 ")\n",
                         mp4p->label,
                         i, mp4p->data_raw_w_idx, mp4p->data_bitstream_idx, myerr );

                mp4_parse_box_SampleSizeTable( &myerr, mp4p, &i );
            }
            /* CHUNK OFFSET BOX */
            id4 = (char*) ( mp4p->data_raw + i );
            if ( ( 0 == strncmp( id4, "stco", 4 ) ) ||
                 ( 0 == strncmp( id4, "co64", 4 ) ) ||
                 ( ( mp4p->search_status & ( SEARCH_STCO_FOUND | SEARCH_CO64_FOUND ) ) &&
                   ( !( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) ) )
               )
            {
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: stco i=%" PRIu32 " (raw_idx=%" PRIu32 ")"
                         "(bitstream_idx=%" PRIu64 ") (err=%" PRIu8 ")\n",
                         mp4p->label,
                         i, mp4p->data_raw_w_idx, mp4p->data_bitstream_idx, myerr );

                mp4_parse_box_ChunkOffsetTable( &myerr, mp4p, &i );
            }
            /* TIME TO SAMPLE BOX */
            id4 = (char*) ( mp4p->data_raw + i );
            if ( ( 0 == strncmp( id4, "stts", 4 ) ) ||
                 ( ( mp4p->search_status & ( SEARCH_STTS_FOUND ) ) &&
                   ( !( mp4p->search_status & SEARCH_STTS_COMPLETE ) ) )
               )
            {
                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] audioSpecificInfo: stts i=%" PRIu32 " (raw_idx=%" PRIu32 ")"
                         "(bitstream_idx=%" PRIu64 ") (err=%" PRIu8 ")\n",
                         mp4p->label, i, mp4p->data_raw_w_idx, mp4p->data_bitstream_idx, myerr );

                mp4_parse_box_stts( &myerr, mp4p, &i );
            }
        } /* soun found */

        if (i == last_i)
        {
            /*
             * i was not incremented during box processing so we need to increment it manually.
             */

            i++;
        }
        last_i = i;
        search_complete_f = SEARCH_COMPLETE(mp4p);
    }

    /*
     * if we got the DSC we pass the location and the size for it
     */
    if ( ( mp4p->mp4_audio_specific_info != NULL ) &&
         ( mp4p->mp4_audio_specific_info_size != 0 ) )
    {
        *audio_specific_info      = mp4p->mp4_audio_specific_info;
        *audio_specific_info_size = mp4p->mp4_audio_specific_info_size;
    }

    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s]s.return. (s.idx=%" PRIu32 ")(b.idx %" PRIu64 ") (%" PRIu8 " %" PRIu8 ")\n\n",
             mp4p->label,
             mp4p->search_idx,
             mp4p->data_bitstream_idx,
             ( ( mp4p->search_status & SEARCH_STSIZE_COMPLETE ) != 0 ),
             ( ( mp4p->search_status & SEARCH_COFFSET_COMPLETE ) != 0 ) );

    if ( NULL != err )
    {
        *err = myerr;
    }
}


static int8_t mp4_get_sample_limits( int8_t* err, mp4parse_t* mp4parse,
                                     uint64_t* chunk_bitstream_offset,
                                     uint64_t* sample_chunk_offset,
                                     uint32_t* sample_size )
{
    int8_t      myerr = 0;
    // size_t tmp_offset=0;
    uint64_t    size = 0;
    uint64_t    offs = 0;


    mp4parse_t* mp4p = mp4parse;


#ifdef DEBUG
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }
#endif

    /*
     * input sanity checks
     */
    if ( ( NULL == chunk_bitstream_offset ) ||
         ( NULL == sample_chunk_offset ) ||
         ( NULL == sample_size ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return 0;
    }

    /*
     * tables sanity checks
     */

#ifdef DEBUG
    if ( ( mp4p->mp4_sample_size_table.byte_data_complete_f == FALSE ) ||
         ( mp4p->mp4_chunk_offs_table.byte_data_complete_f == FALSE ) )
    {
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] get_sample_limits: incomplete tables\n", mp4p->label );
        myerr = MP4PARSE_ERR_UNKNOWN;
    }
#endif

    if ( ( mp4p->mp4_sample_size_table.sample_count_idx >=
           mp4p->mp4_sample_size_table.sample_count ) )
    {
        LOG_ERR( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] [ERROR] get_sample_limits: exceeding sample count\n", mp4p->label );
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( ( mp4p->mp4_chunk_offs_table.entry_count_idx >=
           mp4p->mp4_chunk_offs_table.entry_count ) )
    {
        LOG_ERR( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] [ERROR] get_sample_limits: exceeding offset entry count\n", mp4p->label );
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        /*
         * decode size table
         */
        if ( mp4p->mp4_sample_size_table.table_variant == 0 )
        {
            if ( mp4p->mp4_sample_size_table.sample_size != 0 )
            {
                size = mp4p->mp4_sample_size_table.sample_size;
            }
            else
            {
                size = ntohl( *( (uint32_t*) ( mp4p->mp4_sample_size_table.byte_data +
                                               mp4p->mp4_sample_size_table.byte_data_idx ) ) );

                mp4p->mp4_sample_size_table.sample_count_idx++;

                mp4p->mp4_sample_size_table.byte_data_idx += sizeof( uint32_t );
            }
        }
        else
        {
            uint8_t fsize = mp4p->mp4_sample_size_table.field_size;

            size = *( mp4p->mp4_sample_size_table.byte_data +
                      mp4p->mp4_sample_size_table.byte_data_idx );

            if ( fsize < 8 )
            {
                size = ( size >> ( ( mp4p->mp4_sample_size_table.sample_count_idx & 0x01 ) << 3 ) ) & 0xFF;
            }

            if ( fsize > 8 )
            {
                size = ( size << 8 ) | ( *( mp4p->mp4_sample_size_table.byte_data + mp4p->mp4_sample_size_table.byte_data_idx + 1 ) );
            }

            mp4p->mp4_sample_size_table.sample_count_idx++;

            uint8_t tmp_inc = ( fsize > 8 ) ? 2 : 1;
            tmp_inc >>= ( ( fsize < 8 ) && ( mp4p->mp4_sample_size_table.sample_count_idx & 0x01 ) ) ? 1 : 0;

            mp4p->mp4_sample_size_table.byte_data_idx += tmp_inc;
        }

        /*
         * decode sample_to_chunk table
         */
        uint32_t*           stsc_entry_ptr       = (uint32_t*) ( mp4p->mp4_sample_chunk_table.byte_data +
                                                                 3 * sizeof( uint32_t ) * mp4p->mp4_sample_chunk_table.entry_count_idx );

        uint32_t            stsc_sample_per_chunk = ntohl( *( stsc_entry_ptr + 1 ) );

        uint32_t GCC_UNUSED stsc_first_chunk      = ntohl( *( stsc_entry_ptr + 0 ) );


        mp4p->mp4_sample_chunk_table.sample_in_chunk_idx++;

        *sample_chunk_offset = mp4p->mp4_sample_chunk_table.sample_in_chunk_offs;
        mp4p->mp4_sample_chunk_table.sample_in_chunk_offs += size;


        /* check if we have to select the next chunk */
        if ( mp4p->mp4_sample_chunk_table.sample_in_chunk_idx > stsc_sample_per_chunk )
        {
            mp4p->mp4_sample_chunk_table.sample_in_chunk_idx  = 1;

            // mp4_sample_chunk_table.sample_in_chunk_offs = 0;
            *sample_chunk_offset = 0;
            mp4p->mp4_sample_chunk_table.sample_in_chunk_offs = size;

            /* increment chunk offset index based on offset_table variant */
            mp4p->mp4_chunk_offs_table.entry_count_idx++;

            if ( mp4p->mp4_chunk_offs_table.table_variant == 0 )
            {
                mp4p->mp4_chunk_offs_table.byte_data_idx += sizeof( uint32_t );
            }
            else
            {
                mp4p->mp4_chunk_offs_table.byte_data_idx += sizeof( uint64_t );
            }

            /* check if we have to update the sample_to_chunk entry */
            if ( mp4p->mp4_sample_chunk_table.entry_count > mp4p->mp4_sample_chunk_table.entry_count_idx )
            {
                // uint32_t *tmp_stsc_entry_ptr        = ((uint32_t*)(mp4_sample_chunk_table.byte_data)) + 3 * (mp4_sample_chunk_table.entry_count_idx +1);
                uint32_t*           tmp_stsc_entry_ptr       = (uint32_t*) ( mp4p->mp4_sample_chunk_table.byte_data +
                                                                             3 * sizeof( uint32_t ) * ( mp4p->mp4_sample_chunk_table.entry_count_idx + 1 ) );

                uint32_t            tmp_stsc_first_chunk      = ntohl( *( tmp_stsc_entry_ptr + 0 ) );
                uint32_t GCC_UNUSED tmp_stsc_sample_per_chunk = ntohl( *( tmp_stsc_entry_ptr + 1 ) );

                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] get_sample_limits: CHUNK check, curr %" PRIu32 ", next %" PRIu32 ", "
                         "next size %" PRIu32 " (curr sample %" PRIu32 ", tot samples %" PRIu32 ")\n",
                         mp4p->label,
                         mp4p->mp4_chunk_offs_table.entry_count_idx,
                         tmp_stsc_first_chunk,
                         tmp_stsc_sample_per_chunk,
                         mp4p->mp4_sample_size_table.sample_count_idx,
                         mp4p->mp4_sample_size_table.sample_count );

                /* check if we have to switch entry  */
                /* chunks are counted starting from 1*/
                if ( ( mp4p->mp4_chunk_offs_table.entry_count_idx + 1 ) == tmp_stsc_first_chunk )
                {
                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] get_sample_limits: NEW CHUNK SIZE on chunk num %" PRIu32 ", new size=%" PRIu32 "\n",
                             mp4p->label,
                             tmp_stsc_first_chunk, tmp_stsc_sample_per_chunk );

                    mp4p->mp4_sample_chunk_table.entry_count_idx++;
                    mp4p->mp4_sample_chunk_table.byte_data_idx += sizeof( uint32_t ) * 3;
                }
            }
        }

        /*
         * decode offset table
         */
        if ( mp4p->mp4_chunk_offs_table.table_variant == 0 )
        {
            offs = ntohl( *( (uint32_t*) ( mp4p->mp4_chunk_offs_table.byte_data +
                                           mp4p->mp4_chunk_offs_table.byte_data_idx ) ) );
        }
        else
        {
            offs = SWAP64( *( (uint64_t*) ( mp4p->mp4_chunk_offs_table.byte_data +
                                            mp4p->mp4_chunk_offs_table.byte_data_idx ) ) );
        }

        /* sample with size ZERO is legal! */
        if ( offs != 0 )
        {
            *chunk_bitstream_offset = offs;
            *sample_size = size;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return 0;
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PRIVATE                                                                        */
/* ****************************************************************************** */
/* ****************************************************************************** */

/*
 * Function: check_ready
 */
static inline int8_t check_ready_mp4parse( int8_t* err, mp4parse_t* mp4parse )
{
    int8_t myerr = 0;
    int8_t ret = TRUE;

    if ( NULL == mp4parse )
    {
        return FALSE;
    }

    /* place here all the sanity checks to declare the component "READY" */
    if ( ( TRUE == ret ) && ( MP4PARSE_CSTATUS_CREATED != mp4parse->status ) )
    {
        ret = FALSE;
    }

    /* this MUST be the only place that can bump the status to "READY" */
    if ( TRUE == ret )
    {
        mp4parse->status = MP4PARSE_CSTATUS_READY;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }

    return ret;
}


/*
 * Function: match
 */
inline int8_t match_mp4parse( int8_t* err, mp4parse_t* mp4parse, mp4parse_ctype_t checktype )
{
    int8_t ret = FALSE;

    /* err it's just a placeholder */
    int8_t myerr = NO_ERR;

    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return ret;
    }

    if ( ( MP4PARSE_CTYPE_RESERVED < mp4parse->type ) && \
         ( MP4PARSE_CTYPE_MAX > mp4parse->type ) )
    {
        ret = ( MP4PARSE_WMARK == mp4parse->wmark ) ? TRUE : FALSE;
    }

    if ( ( ret == TRUE ) && ( MP4PARSE_CTYPE_ANYTYPE != checktype ) )
    {
        ret = ( checktype == mp4parse->type ) ? TRUE : FALSE;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
    return ret;
}


/*
 * Function:
 *
 */
static void init_mp4parse( int8_t* err, mp4parse_t* mp4parse )
{
    int8_t      myerr = NO_ERR;

    mp4parse_t* mp4p = mp4parse;

    if ( NULL != mp4p )
    {
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][-] init_mp4parse\n" );

        /* init random seed */
        // srand((uint32_t)time(NULL));

        /* component base */
        mp4p->id    = 1; // should be random();
        mp4p->wmark = MP4PARSE_WMARK;
        mp4p->type  = MP4PARSE_CTYPE_UNKNOWN;

        /* We only support one mode: single threaded */
        mp4p->type = MP4PARSE_CTYPE_SINGLE_THREAD;

        /* versioning stuff: the default is "0.0.0.none" */
        mp4p->major = 0;
        mp4p->minor = 0;
        strncpy( mp4p->revision, "0000000000000000", ( 16 + 1 ) ); /* revision is 32 char max */
        strncpy( mp4p->build,    "none",             ( 4 + 1 ) );  /* build is 5 char max */

        /* override versioning by build machine at compilation time */
#if ( defined( VERSION_MAJOR ) && defined( VERSION_MINOR ) && defined( VERSION_REVISION ) && defined( VERSION_TYPE ) )
        mp4p->major = VERSION_MAJOR;
        mp4p->minor = VERSION_MINOR;
        strncpy( mp4p->revision, VERSION_REVISION, 16 );
        strncpy( mp4p->build,    VERSION_TYPE,     4 );
#endif /* versioning override */

        /* CTRL register is internal */
        mp4p->ctrl = 0;
        strncpy( mp4p->label, "mp4p_cmp\0", 14 );


        /* in/out threads setup */
        TSP_MUTEX_INIT( myerr, mp4p->ctrl_mtx );


        /* mp4parse */
        TSP_SEMAPHORE_INIT( myerr, mp4p->mp4_thread_sem );

        /* input buffer fifo init */
        mp4p->data_fifo_wrap  = BUF_NUM - 1;
        mp4p->data_fifo_w_idx = 0;
        mp4p->data_fifo_r_idx = 0;

        /* data raw fifo */
        mp4p->data_raw_w_idx  = 0;
        mp4p->data_raw_wrap   = BUF_RAW_SIZE - 1;

        /* bitstream indexing */
        mp4p->data_bitstream_idx    = 0;
        mp4p->data_bitstream_skip_f = FALSE;

        /* to allow skip bitstream */
        mp4p->data_size_prev = 0;

        /* search variables */
        mp4p->search_idx     = 0;
        mp4p->search_status  = SEARCH_NONE_FOUND;

        /* search results */
        mp4p->mp4_audio_specific_info      = NULL;
        mp4p->mp4_audio_specific_info_size = 0;

        /* stsz */
        mp4p->mp4_sample_size_table.sample_count         = 0;
        mp4p->mp4_sample_size_table.sample_count_idx     = 0;
        mp4p->mp4_sample_size_table.byte_data            = NULL;
        mp4p->mp4_sample_size_table.byte_data_size       = 0;
        mp4p->mp4_sample_size_table.byte_data_idx        = 0;
        mp4p->mp4_sample_size_table.byte_data_complete_f = FALSE;


        /* stco */
        mp4p->mp4_chunk_offs_table.entry_count           = 0;
        mp4p->mp4_chunk_offs_table.entry_count_idx       = 0;
        mp4p->mp4_chunk_offs_table.byte_data             = NULL;
        mp4p->mp4_chunk_offs_table.byte_data_size        = 0;
        mp4p->mp4_chunk_offs_table.byte_data_idx         = 0;
        mp4p->mp4_chunk_offs_table.byte_data_complete_f  = FALSE;

        /* stsc */
        mp4p->mp4_sample_chunk_table.entry_count          = 0;
        mp4p->mp4_sample_chunk_table.entry_count_idx      = 0;
        mp4p->mp4_sample_chunk_table.byte_data            = NULL;
        mp4p->mp4_sample_chunk_table.byte_data_size       = 0;
        mp4p->mp4_sample_chunk_table.byte_data_idx        = 0;
        mp4p->mp4_sample_chunk_table.byte_data_complete_f = FALSE;

        /* stts */
        mp4p->mp4_time_sample_table.entry_count           = 0;
        mp4p->mp4_time_sample_table.byte_data             = NULL;
        mp4p->mp4_time_sample_table.byte_data_size        = 0;
        mp4p->mp4_time_sample_table.byte_data_idx         = 0;
        mp4p->mp4_time_sample_table.byte_data_complete_f  = FALSE;

        /* audio track */
        mp4p->mp4_audio_track_info.creation_time     = 0;
        mp4p->mp4_audio_track_info.modification_time = 0;
        mp4p->mp4_audio_track_info.timescale         = 0;
        mp4p->mp4_audio_track_info.duration          = 0;

        /* internals */
        mp4p->tpm.tool_type = TPM_TOOL_UNKNOWN;

        /* OUTPUT callbacks*/

        /* INPUT callbacks*/

        /* ALERT callbacks */

        /* external app ptr */

        /* update status */
        mp4p->status = MP4PARSE_CSTATUS_CREATED;

        LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] init_mp4parse: done.\n", mp4parse->label );
    }
    else
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }
}


static void destroy_mp4parse( int8_t* err, mp4parse_t** mp4parse )
{
    int8_t            myerr = NO_ERR;

    mp4parse_t*       mp4p = NULL;

    volatile uint8_t* ctrl = NULL;

    /*
     * SANITY CHECKS
     */
#if defined( DEBUG )
    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) || ( NULL == *mp4parse ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }
        return;
    }

    /* check for previous errors */

    /*
     * if(NULL!=err)
     * {
     *  if(NO_ERR!=(*err))
     *  {
     *      return;
     *  }
     * }
     */
#endif

    if ( NO_ERR == myerr )
    {
        mp4p = *mp4parse;

        ctrl = &mp4p->ctrl;

        /* mute the output push */
        mp4p->output_push_callback = NULL;

        /* DSP: trigger the thread until is gone */
        while ( ( *ctrl ) & MP4PARSE_CTRL_DSP_STATUS )
        {
            TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
            SYSCLK_USLEEP( uSLEEP10ms );
        }
        TSP_THREAD_JOIN( NULL, mp4p->mp4_thread );
        TSP_THREAD_DELETE( NULL, mp4p->mp4_thread );
        TSP_SEMAPHORE_DESTROY( NULL, mp4p->mp4_thread_sem );


        /* MP4 search results */

        /* deallocate search result tables */
        if ( NULL != mp4p->mp4_sample_size_table.byte_data )
        {
            free( mp4p->mp4_sample_size_table.byte_data );
            mp4p->mp4_sample_size_table.byte_data = NULL;
        }

        if ( NULL != mp4p->mp4_chunk_offs_table.byte_data )
        {
            free( mp4p->mp4_chunk_offs_table.byte_data );
            mp4p->mp4_chunk_offs_table.byte_data = NULL;
        }

        if ( NULL != mp4p->mp4_sample_chunk_table.byte_data )
        {
            free( mp4p->mp4_sample_chunk_table.byte_data );
            mp4p->mp4_sample_chunk_table.byte_data = NULL;
        }

        if ( NULL != mp4p->mp4_time_sample_table.byte_data )
        {
            free( mp4p->mp4_time_sample_table.byte_data );
            mp4p->mp4_time_sample_table.byte_data = NULL;
        }

        if ( NULL != mp4p->mp4_audio_specific_info )
        {
            free( mp4p->mp4_audio_specific_info );
            mp4p->mp4_audio_specific_info = NULL;
        }

        /* deallocate raw fifo */
        if ( NULL != mp4p->data_raw )
        {
            free( mp4p->data_raw );
            mp4p->data_raw = NULL;
        }

        /* deallocate buffer fifo */
        if ( NULL != mp4p->data_fifo )
        {
            free( mp4p->data_fifo );
            mp4p->data_fifo = NULL;
        }

#if ( ENABLE_FDK_WORKAROUND )
        if ( mp4p->fdk_handle )
        {
            /* BEGIN - WORKAROUND: FDK library */
            /* make a call to decoderFill to detect the OUT_OF_MEMORY error */
            /* which internally will _Close() the aac decoder already */
            uint32_t  tmpBytes      = 0xFFFF;
            uint32_t* tmpBytesPtr  = &tmpBytes;
            uint32_t  tmpBytesValid = 0;
            uint32_t  tmpByteSize   = sizeof( uint32_t );

            mp4p->fdk_err = aacDecoder_Fill( mp4p->fdk_handle, (unsigned char**) &tmpBytesPtr,
                                             (const UINT*) &tmpByteSize, (UINT*) &tmpBytesValid );

            if ( AAC_DEC_OK != mp4p->fdk_err )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_dsp_loop: FDK (internal) aacDecClose()."
                         "Setting the fdk handle to NULL (OUT-OF-MEMORY)\n",
                         mp4p->label );

                mp4p->fdk_handle = NULL;
            }
            /* END - WORKAROUND */
        }
#endif

        if ( mp4p->fdk_handle )
        {
            aacDecoder_Close( mp4p->fdk_handle );
        }

        if ( NULL != mp4p->data_coalesce )
        {
            free( mp4p->data_coalesce );
            mp4p->data_coalesce = NULL;
        }

        if ( NULL != mp4p->data_decode )
        {
            free( mp4p->data_decode );
            mp4p->data_decode = NULL;
        }

        /* finally remove the component */
        if ( NULL != mp4p )
        {
            TSP_MUTEX_DESTROY( NULL, mp4p->ctrl_mtx );

            free( mp4p );
            *mp4parse = NULL;
        }

        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][-] mp4parse_destroy. done. \n" );
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


static inline uint32_t fdkchmap_to_chmap( int8_t* err, CStreamInfo* info )
{
    uint32_t chmap = 0;

    if ( NULL == info )
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        return chmap;
    }

    /* FDK is set for WAV mapping by default, so we have a simple table */
    switch ( info->numChannels )
    {
        case 2:
            chmap = ( AACDEC_CHANNEL_MAP_FL | AACDEC_CHANNEL_MAP_FR );
            break;

        case 6:
            chmap = ( AACDEC_CHANNEL_MAP_FL | AACDEC_CHANNEL_MAP_FR | AACDEC_CHANNEL_MAP_FC | AACDEC_CHANNEL_MAP_LFE1 | AACDEC_CHANNEL_MAP_LS | AACDEC_CHANNEL_MAP_RS );
            break;

        case 8:
            chmap  = ( AACDEC_CHANNEL_MAP_FL | AACDEC_CHANNEL_MAP_FR | AACDEC_CHANNEL_MAP_FC | AACDEC_CHANNEL_MAP_LFE1 | AACDEC_CHANNEL_MAP_FLC | AACDEC_CHANNEL_MAP_FRC );
            chmap |= ( AACDEC_CHANNEL_MAP_SIL | AACDEC_CHANNEL_MAP_SIR );
            break;

        default:
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-][ERROR] fdkchmap_to_chmap unsupported AAC channel mapping chnum=%d\n",
                     ( info->numChannels ) );
            chmap = 0;
            break;
    }

    if ( NULL != err )
    {
        *err = -1;
    }

    return chmap;
}


tsp_loop_return mp4parse_dsp_loop( tsp_loop_param data )
{
    int8_t            myerr = NO_ERR;

    volatile uint8_t* ctrl = NULL;

    mp4parse_t*       mp4p = NULL;

    uint64_t          chunk_bitstream_offset = 0;
    uint64_t          sample_chunk_offset    = 0;
    uint32_t          sample_size            = 0;

    /* INTERNALS */
    uint8_t           fdk_f = TRUE;
    uint8_t           fdk_pcm_info_f = TRUE;
    uint32_t          fdk_valid  = 0;

    uint8_t           fseek_f        = TRUE;
    uint8_t           fseek_chunk_f  = TRUE;

    uint32_t          local_offs = 0;
    uint32_t          local_size = 0;



    /* retrieve the component base */
    mp4p = (mp4parse_t*) data;

    /* SANITY CHECKS */
    if ( mp4p == NULL )
    {
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-][ERROR] mp4parse_dsp_loop, no mp4parse base component\n" );
        myerr = MP4PARSE_ERR_UNKNOWN;
        TSP_THREAD_EXIT( );
        TSP_THREAD_RETURN_NULL( );
    }
    else
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-][ERROR] mp4parse_dsp_loop, not a valid mp4parse component\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
            TSP_THREAD_EXIT( );
            TSP_THREAD_RETURN_NULL( );
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &mp4p->ctrl;
        }
    }

    /* component must be in READY state */
    if ( MP4PARSE_CSTATUS_READY != mp4p->status )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s][ERROR] mp4parse_dsp_loop, mp4parse NOT in ready state\n", mp4p->label );
    }

    /* redundant */
    if ( ctrl == NULL )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s][ERROR] mp4parse_dsp_loop, no ctrl\n", mp4p->label );
    }

    /*
     * INNER LOOP here
     */
    if ( NO_ERR == myerr )
    {
        /* set the status flag */
        TSP_MUTEX_LOCK( NULL, mp4p->ctrl_mtx );
        *ctrl = ( *ctrl ) | MP4PARSE_CTRL_DSP_STATUS;
        TSP_MUTEX_UNLOCK( NULL, mp4p->ctrl_mtx );

        if ( ( *ctrl ) & MP4PARSE_CTRL_DSP_STATUS )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4parse_dsp_loop (STARTING) \n", mp4p->label );
            do
            {
                /** DSP thread
                 *  wake up on new data input and parse the available data in the DSP queue
                 *  note: as fast as possible for free_run clock, or paste based on DSP input SYSCLK values
                 */
                uint8_t search_complete_f = SEARCH_COMPLETE(mp4p);

                while ( ( ( *ctrl ) & MP4PARSE_CTRL_DSP_STATUS ) &&
                        ( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) &&
                        ( ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status == 0 ) ||
                          ( FALSE == search_complete_f ) ) )
                {
                    TSP_SEMAPHORE_WAIT( NULL, mp4p->mp4_thread_sem );

                    search_complete_f = SEARCH_COMPLETE(mp4p);
                }

                /*
                 * inline CMD filter
                 */

                /* exit on EOS */
                if ( ( AACDEC_BUFFER_TYPE_CMD_EOS == mp4p->data_fifo[ mp4p->data_fifo_r_idx & mp4p->data_fifo_wrap ].type ) ||
                     ( AACDEC_BUFFER_TYPE_MAX <= mp4p->data_fifo[ mp4p->data_fifo_r_idx & mp4p->data_fifo_wrap ].type ) )
                {
                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] end-of-stream packet cmd.\n",
                             mp4p->label );

                    myerr = MP4PARSE_ERR_END_OF_STREAM;
                }


                /*
                 * INIT the aac decoder (once) with the audio specific info from the stream search
                 */
                if ( ( TRUE == fdk_f ) && ( NO_ERR == myerr ) )
                {
                    fdk_f = FALSE;

                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4parse_dsp_loop: initialize FDK (%p %" PRIu32 ") \n",
                             mp4p->label,
                             mp4p->mp4_audio_specific_info,
                             mp4p->mp4_audio_specific_info_size );

                    mp4p->fdk_err = aacDecoder_ConfigRaw( mp4p->fdk_handle,
                                                          &mp4p->mp4_audio_specific_info,
                                                          (const UINT*) &mp4p->mp4_audio_specific_info_size );

                    if ( mp4p->fdk_err != AAC_DEC_OK )
                    {
                        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_dsp_loop: unable to initialize FDK with the ASC provided. (fdk_err=%" PRIu16 ")\n",
                                 mp4p->label, mp4p->fdk_err );

                        myerr = MP4PARSE_ERR_UNKNOWN;
                    }
                    else
                    {
                        /* set extra/optional parameters : sample interleaving */
                        mp4p->fdk_err = aacDecoder_SetParam( mp4p->fdk_handle,
                                                             AAC_PCM_OUTPUT_INTERLEAVED,
                                                             1 /* interleaved */ );

                        if ( mp4p->fdk_err != AAC_DEC_OK )
                        {
                            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_dsp_loop: unable to set FDK optional paramters.\n",
                                     mp4p->label );

                            myerr = MP4PARSE_ERR_UNKNOWN;
                        }

                        /* set extra/optional parameters : channel mapping WAV style  */
                        mp4p->fdk_err = aacDecoder_SetParam( mp4p->fdk_handle,
                                                             AAC_PCM_OUTPUT_CHANNEL_MAPPING,
                                                             1 /* WAV order*/ );

                        if ( mp4p->fdk_err != AAC_DEC_OK )
                        {
                            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_dsp_loop: unable to set FDK optional paramters.\n",
                                     mp4p->label );

                            myerr = MP4PARSE_ERR_UNKNOWN;
                        }
                    }
                }

                if ( NO_ERR == myerr )
                {
                    if ( TRUE == fseek_f )
                    {
                        /* PROFILING/TraceX */
                        TPM_USRLOG( NULL, &mp4p->tpm,
                                    TPM_MP4_DSP_PKT_SEEK,
                                    mp4p->data_fifo_w_idx,
                                    mp4p->data_fifo_r_idx,
                                    mp4p->data_raw_w_idx,
                                    0 );

                        /*
                         * FSEEK : skip the input stream up to the first chunk of aac data
                         */

                        if ( TRUE == fseek_chunk_f )
                        {
                            fseek_chunk_f = FALSE;

                            /*
                             * chunk the input stream in sample frames
                             */
                            mp4_get_sample_limits( &myerr, mp4p, &chunk_bitstream_offset, &sample_chunk_offset, &sample_size );

                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] @@@@@@@@AAA@@@@@@@@ Got new frame, "
                                     "c.offs=%" PRIu32 " s.offs=%" PRIu32 " s.size=%" PRIu32 " \n",
                                     mp4p->label,
                                     (uint32_t) chunk_bitstream_offset, (uint32_t) sample_chunk_offset, (uint32_t) sample_size );

                            /* PROFILING/TraceX */
                            TPM_USRLOG( NULL, &mp4p->tpm,
                                        TPM_MP4_DSP_CHUNK_SEEK,
                                        chunk_bitstream_offset,
                                        sample_chunk_offset,
                                        sample_size,
                                        0 );
                        }

                        // if( (uint32_t)(mp4p->data_fifo[(mp4p->data_fifo_r_idx+1) & mp4p->data_fifo_wrap].bitstream_idx) <
                        //    (uint32_t)(chunk_bitstream_offset+sample_chunk_offset + sample_size) )

                        if ( ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx +  mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length ) <
                             ( chunk_bitstream_offset + sample_chunk_offset + sample_size ) )
                        {
                            // if(mp4p->data_fifo[(mp4p->data_fifo_r_idx+1) & mp4p->data_fifo_wrap].status!=0)
                            if ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status != 0 )
                            {
                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4parse_dsp_loop: FSEEK-> %" PRIu32 " of %" PRIu32 " "
                                         "{fifo_r_idx=%" PRIu32 " fifo_w_idx=%" PRIu32 ", w_bitstream_idx=%" PRIu32 " }\n",
                                         mp4p->label,
                                         (uint32_t) mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx,
                                         (uint32_t) ( chunk_bitstream_offset + sample_chunk_offset + sample_size ),
                                         mp4p->data_fifo_r_idx, mp4p->data_fifo_w_idx,
                                         (uint32_t) ( mp4p->data_fifo[ ( mp4p->data_fifo_r_idx + 1 ) & mp4p->data_fifo_wrap ].bitstream_idx )
                                       );


                                /* release/skip the current buffer since it is not needed */
                                mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status  = 0;

                                /* next buffer */
                                mp4p->data_fifo_r_idx = ( mp4p->data_fifo_r_idx + 1 ) & mp4p->data_fifo_wrap;
                            }
                            else
                            {
                                myerr = MP4PARSE_ERR_NEED_MORE_BITS;
                            }
                        }
                        else
                        {
                            fseek_f = FALSE;

                            LOG_MSG( mp4p->log_lvl, LOG_LOW,
                                     "[MP4PARSE][%s] \n\n>>>>>>>>>>>>\nSTART DECODING: w_idx=%" PRIu32 ", r_idx=%" PRIu32 ", data_bitstream_idx=%" PRIu64 ", "
                                     "b.idx=%" PRIu64 ", chunk_bitstream_offs=%" PRIu64 " sample_offs=%" PRIu64 "\n",
                                     mp4p->label,
                                     mp4p->data_fifo_w_idx,
                                     mp4p->data_fifo_r_idx,
                                     mp4p->data_bitstream_idx,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx,
                                     chunk_bitstream_offset,
                                     sample_chunk_offset );


                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] 1ST buffer data_r_idx=%" PRIu32 ", raw_w.idx=%" PRIu32 " r_buf.size "
                                     "%" PRIu32 ", r_buf.raw_idx=%" PRIu32 ", r_buf.raw=%p\n",
                                     mp4p->label,
                                     mp4p->data_fifo_r_idx,
                                     mp4p->data_raw_w_idx,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw );

                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] <<<<<<<<<<<<<<<<<<<<<\n\n", mp4p->label );
                        }
                    }
                    else
                    {
                        /*
                         * COALESCE & DECODE
                         */
                        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] DECODE!\n", mp4p->label );


                        /* *************************************************** */
                        /* ADJUST: check if we have to move the r_idx          */
                        /* *************************************************** */

                        uint32_t tmp32 = ( mp4p->data_fifo_r_idx ) & mp4p->data_fifo_wrap;

                        /* loop until the next slot has valid data */
                        while ( ( 0 != mp4p->data_fifo[ tmp32 ].status ) &&
                                ( ( mp4p->data_fifo[ tmp32 ].bitstream_idx + mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length ) <=
                                  ( chunk_bitstream_offset + sample_chunk_offset + local_size ) ) )
                        {
                            /* release current read slot */
                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] RELEASE buffer r_idx=%" PRIu32 "\n",
                                     mp4p->label,
                                     mp4p->data_fifo_r_idx );

                            mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status  = 0;

                            mp4p->data_fifo_r_idx = ( mp4p->data_fifo_r_idx + 1 ) & mp4p->data_fifo_wrap;

                            tmp32 = ( mp4p->data_fifo_r_idx + 1 ) & mp4p->data_fifo_wrap;
                        }

                        /* *************************************************** */
                        /* DECODER: compute push size and limits               */
                        /* *************************************************** */

                        if ( 0 != mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status )
                        {
                            /* compute buffer offset */
                            local_offs = ( ( chunk_bitstream_offset + sample_chunk_offset + local_size ) -
                                           mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx );

                            /* compute data size */
                            uint32_t push_size = ( sample_size - local_size );

                            if ( ( local_offs + push_size ) > mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length )
                            {
                                push_size = mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length - local_offs;
                            }

                            /* *********************************************** */
                            /* DECODER: fill the fdk-sdk with the data bits    */
                            /* *********************************************** */
                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] DEC:  b.offs=%" PRIu64 " b.length=%" PRIu32 ", l.offs=%" PRIu32 ", bit_offs=%" PRIu64 ", "
                                     "sample_offs=%" PRIu64 ", [%" PRIu64 "=%" PRIu64 "], local_size=%" PRIu32 " (sample_size=%" PRIu32 ")\n",
                                     mp4p->label,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].length,
                                     local_offs,
                                     chunk_bitstream_offset,
                                     sample_chunk_offset,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx + local_offs,
                                     chunk_bitstream_offset + sample_chunk_offset,
                                     local_size,
                                     sample_size );

                            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] FDK-DEC: FILL bitstream_idx=%" PRIu64 ", buf_ptr=%p, fill_size=%" PRIu32 "\n",
                                     mp4p->label,
                                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx + local_offs,
                                     ( mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw + local_offs ),
                                     push_size );

                            fdk_valid = push_size;

                            uint8_t* data_ptr = &mp4p->data_raw[ mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx + local_offs ];

                            if ( push_size == sample_size )
                            {
                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &mp4p->tpm,
                                            TPM_MP4_DSP_FDK_FILL_BEGIN,
                                            chunk_bitstream_offset,
                                            sample_chunk_offset,
                                            sample_size,
                                            push_size );

                                mp4p->data_coalesce_ptr = mp4p->data_coalesce;

                                mp4p->fdk_err = aacDecoder_Fill( mp4p->fdk_handle, &data_ptr, (const UINT*) &push_size, (UINT*) &fdk_valid );
                                if ( mp4p->fdk_err != AAC_DEC_OK )
                                {
                                    LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] FDK-DEC: error, Fill failed: %x\n",
                                             mp4p->label,
                                             mp4p->fdk_err );

                                    break;
                                }

                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &mp4p->tpm,
                                            TPM_MP4_DSP_FDK_FILL_END,
                                            chunk_bitstream_offset,
                                            sample_chunk_offset,
                                            sample_size,
                                            push_size );

                                local_size += ( push_size - fdk_valid );
                            }
                            else if ( push_size > sample_size )
                            {
                                LOG_ERR( 0, LOG_ALWAYS,
                                         "[MP4PARSE][%s] [ERROR] COALESCING chunk too large (bitstream failure?)  %" PRIu32 " vs %" PRIu32 "\n",
                                         mp4p->label,
                                         push_size,
                                         sample_size );

                                myerr = MP4PARSE_ERR_UNKNOWN;

                                continue;
                            }
                            else
                            {
                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] --> COALESCING %" PRIu32 " vs %" PRIu32 "\n",
                                         mp4p->label,
                                         push_size,
                                         sample_size );

                                /* COALESCING for M4A seems is needed for FDK */

                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] FDK-DEC: COALESCE r_idx=%" PRIu32 ", raw_idx=%" PRIu32 ", "
                                         "local_offs=%" PRIu32 " [size=%" PRIu32 "]\n",
                                         mp4p->label,
                                         mp4p->data_fifo_r_idx,
                                         mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx,
                                         local_offs,
                                         push_size );

                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &mp4p->tpm,
                                            TPM_MP4_DSP_FDK_COALESCE,
                                            chunk_bitstream_offset,
                                            sample_chunk_offset,
                                            sample_size,
                                            push_size );

                                /* ToDo: check for error here */
                                memcpy( mp4p->data_coalesce_ptr, data_ptr, push_size );

                                mp4p->data_coalesce_ptr += push_size;

                                /* count all data as cpied */
                                fdk_valid = 0;

                                local_size += ( push_size - fdk_valid );

                                /* now feed the coalesced sample */
                                push_size = local_size;

                                fdk_valid = local_size;

                                if ( local_size == sample_size )
                                {
                                    LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] FDK-DEC: COALESCE done! [size=%" PRIu32 "]\n",
                                             mp4p->label,
                                             push_size );

                                    mp4p->fdk_err = aacDecoder_Fill( mp4p->fdk_handle, &mp4p->data_coalesce, (const UINT*) &push_size, (UINT*) &fdk_valid );

                                    if ( mp4p->fdk_err != AAC_DEC_OK )
                                    {
                                        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] FDK-DEC: error, Fill failed: %x\n",
                                                 mp4p->label,
                                                 mp4p->fdk_err );
                                        break;
                                    }

                                    mp4p->data_coalesce_ptr = mp4p->data_coalesce;
                                }
                            }

                            if ( local_size == sample_size )
                            {
                                /* ***************************** */
                                /* DECODER: decode_samples       */
                                /* if we pushed a whole frame    */
                                /* ***************************** */

                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] FDK-DEC: DECODE bitstream_buf=%" PRIu64 ", in_buf=%p, "
                                         "in_size=%" PRIu32 " [raw_buf start %p, raw_idx=%" PRIu32 "]\n",
                                         mp4p->label,
                                         mp4p->data_fifo[ mp4p->data_fifo_r_idx ].bitstream_idx + local_offs,
                                         ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw + local_offs ),
                                         push_size,
                                         &mp4p->data_raw[ 0 ],
                                         mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx );

                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &mp4p->tpm,
                                            TPM_MP4_DSP_FDK_DECODE_BEGIN,
                                            chunk_bitstream_offset,
                                            sample_chunk_offset,
                                            sample_size,
                                            push_size );

                                mp4p->fdk_err = aacDecoder_DecodeFrame( mp4p->fdk_handle, mp4p->data_decode, BUF_8K, 0 );

                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &mp4p->tpm,
                                            TPM_MP4_DSP_FDK_DECODE_END,
                                            chunk_bitstream_offset,
                                            sample_chunk_offset,
                                            sample_size,
                                            push_size );

                                if ( mp4p->fdk_err == AAC_DEC_NOT_ENOUGH_BITS )
                                {
                                    LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] Decode failed: aac_not_enough_bits %x\n",
                                             mp4p->label,
                                             mp4p->fdk_err );

                                    continue;
                                }
                                else if ( mp4p->fdk_err != AAC_DEC_OK )
                                {
                                    LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] Decode failed: %x\n",
                                             mp4p->label,
                                             mp4p->fdk_err );

                                    myerr = MP4PARSE_ERR_UNKNOWN;

                                    continue;
                                }

                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] DECODED OK!\n", mp4p->label );

                                /* PCM output callback */
                                if ( ( NO_ERR == myerr ) && ( NULL != mp4p->output_push_callback ) )
                                {
                                    /* describe sample */
                                    CStreamInfo* info = aacDecoder_GetStreamInfo( mp4p->fdk_handle );

                                    if ( !info || ( info->sampleRate <= 0 ) )
                                    {
                                        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] DECODED : No stream info\n", mp4p->label );
                                    }
                                    else
                                    {
                                        if ( TRUE == fdk_pcm_info_f )
                                        {
                                            /* our FDK only sends s16LE output */
                                            mp4p->pcm_info.bps      = 16;
                                            mp4p->pcm_info.cbps     = 16;
                                            mp4p->pcm_info.chnum    = info->numChannels;
                                            mp4p->pcm_info.sr       = info->sampleRate;
                                            mp4p->pcm_info.chmap    = fdkchmap_to_chmap( NULL, info );
                                            mp4p->pcm_info.endian   = 0; /* little */
                                            mp4p->pcm_info.sign     = 1; /* signed */
                                            mp4p->pcm_info.floating = 0; /* int */

                                            fdk_pcm_info_f = FALSE;
                                        }

                                        /* PROFILING/TraceX */
                                        TPM_USRLOG( NULL, &mp4p->tpm,
                                                    TPM_MP4_DSP_PCM_PUSH,
                                                    mp4p->pcm_info.chnum,
                                                    mp4p->pcm_info.sr,
                                                    mp4p->pcm_info.chmap,
                                                    mp4p->pcm_info.cbps );

                                        mp4p->output_push_callback(
                                            &myerr,
                                            mp4p,
                                            mp4p->decoder,
                                            &mp4p->pcm_info,
                                            mp4p->data_decode,
                                            ( info->frameSize * info->numChannels ),
                                            NULL );
                                    }
                                }
                            }

                            if ( local_size == sample_size )
                            {
                                mp4_get_sample_limits( &myerr, mp4p, &chunk_bitstream_offset, &sample_chunk_offset, &sample_size );

                                LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] @@@@@@@@@@@@@ Got new frame, c.offs=%" PRIu32 " "
                                         "s.offs=%" PRIu32 " s.size=%" PRIu32 " \n",
                                         mp4p->label,
                                         (uint32_t) chunk_bitstream_offset, (uint32_t) sample_chunk_offset, (uint32_t) sample_size );

                                local_size = 0;
                            }
                        }
                    }
                }
            } while ( ( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) && ( NO_ERR == myerr ) );
        }
        else
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s][ERROR] mp4parse_dsp_loop could not update the ctrl register.\n", mp4p->label );
        }
    }
    /* *** */

    /*
     * IMPORTANT:
     * DSP thread is done, signal a "inline" pcm bufffer with ID=0 to signal that there is no more data
     */
    if ( NULL != mp4p->output_push_callback )
    {
        /* ID code for the in-band signaling */
        uint32_t id_done = 0;

        /* optional payload for the in-band signaling */
        int16_t  buf_payload = 0;

        if ( NO_ERR == myerr )
        {
            id_done = PCMBUF_ID_CMD_EOP_OK;

            mp4p->output_push_callback(
                &myerr,
                mp4p,
                mp4p->decoder,
                &mp4p->pcm_info,
                &buf_payload, // mp4p->data_decode,
                0,            /* payload size */
                &id_done );
        }
        else
        {
            id_done = PCMBUF_ID_CMD_EOP_ERR;

            /* tbd: form a proper payload
             * once the error format is defined */
            mp4p->output_push_callback(
                &myerr,
                mp4p,
                mp4p->decoder,
                &mp4p->pcm_info,
                &buf_payload,
                0, /* payload size */
                &id_done );
        }
    }

    /*
     * quit.
     */
    TSP_MUTEX_LOCK( NULL, mp4p->ctrl_mtx );
    *ctrl = ( *ctrl ) & ( ~( MP4PARSE_CTRL_DSP_STATUS ) );
    TSP_MUTEX_UNLOCK( NULL, mp4p->ctrl_mtx );

    LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] mp4parse_dsp_loop quit. (%" PRIu8 ") \n", mp4p->label, myerr );


    /* exit with zero */
    TSP_THREAD_EXIT( );
    TSP_THREAD_RETURN_NULL( );
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PUBLIC                                                                         */
/* ****************************************************************************** */
/* ****************************************************************************** */



/* ************************ */
/* SYSTEM                   */
/* ************************ */

void mp4parse_new( int8_t* err,
                   void**  mp4parse,
                   void*   decoder,
                   uint8_t logging_level )
{
    int8_t      myerr = NO_ERR;

    mp4parse_t* mp4p = NULL;

    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( NULL != ( *mp4parse ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, input mp4parse ptr not NULL, FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    /*
     * MAIN
     */

    /* allocate component base */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( mp4p = (mp4parse_t*) calloc( 1, sizeof( mp4parse_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, mp4parse_t create FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            mp4p->memory_footprint = sizeof( mp4parse_t );
        }
    }

    /* allocate input buffer queue */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( mp4p->data_fifo = (data_buf_t*) calloc( BUF_NUM, sizeof( data_buf_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, input buffer fifo create FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            mp4p->memory_footprint += sizeof( data_buf_t ) * BUF_NUM;
        }
    }

    /* allocate input raw queue */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( mp4p->data_raw = (uint8_t*) calloc( BUF_RAW_SIZE, sizeof( uint8_t ) ) ) )

        // if(NULL == (mp4p->data_raw = (uint8_t*)malloc(sizeof(uint8_t)*BUF_RAW_SIZE)) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, input raw fifo create FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            mp4p->memory_footprint += sizeof( uint8_t ) * BUF_RAW_SIZE;
        }
    }

    /* allocate coalesce */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( mp4p->data_coalesce = (uint8_t*) calloc( BUF_8K, sizeof( uint8_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, coalesce buf create FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            mp4p->memory_footprint += sizeof( uint8_t ) * BUF_8K;
        }
    }

    /* allocate decode */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( mp4p->data_decode = (int16_t*) calloc( BUF_8K / 2, sizeof( int16_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, decode buf create FAIL\n" );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            mp4p->memory_footprint += sizeof( uint8_t ) * BUF_8K / 2;
        }
    }

    /* set default values */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][-] mp4parse_new, mp4parse_t size=%" PRIu32 "\n", (uint32_t) sizeof( mp4parse_t ) );

        /* PRE-INIT */
        mp4p->id    = 1;
        mp4p->wmark = MP4PARSE_WMARK;
        mp4p->type  = MP4PARSE_CTYPE_UNKNOWN;
        // mp4p->type  = MP4PARSE_CTYPE_SINGLE_THREAD;

        mp4p->decoder = decoder;
        mp4p->fdk_handle = NULL;
        mp4p->output_push_callback = NULL;

        /* logging */
        mp4p->log_lvl = LOG_MEDIUM;
        mp4p->log_err = LOG_ALWAYS;

        /* params/config values for sub-components  */
    }


    if ( NO_ERR == myerr )
    {
        /* force the type */
        mp4p->type = MP4PARSE_CTYPE_RESERVED;

        /* force the status*/
        mp4p->status = MP4PARSE_CSTATUS_UNKNOWN;

        /* init the component */
        init_mp4parse( &myerr, mp4p );

        /* set the initial log level*/
        mp4parse_set_log_verbosity( NULL, mp4p, logging_level );

        LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4parse_new\n", mp4p->label );
    }


    /*
     * POST-INIT: open FDK-AAC decoder
     */
    if ( NO_ERR == myerr )
    {
        mp4p->fdk_handle = aacDecoder_Open( TT_MP4_RAW, 1 );
        if (mp4p->fdk_handle == NULL)
        {
            myerr = MP4PARSE_ERR_OUT_OF_MEMORY;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] mp4parse_new, SUCCESS\n",            mp4p->label );
        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] mp4parse_new, memory=%" PRIu64 "\n", mp4p->label, mp4p->memory_footprint );

        /* return cmp */
        *mp4parse = (void*) mp4p;
    }
    else
    {
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_new, FAIL\n" );

        destroy_mp4parse( NULL, &mp4p );

        *mp4parse = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void mp4parse_destroy( int8_t* err,
                       void**  mp4parse )
{
    int8_t            myerr = NO_ERR;

    mp4parse_t*       mp4p = NULL;

    volatile uint8_t* ctrl = NULL;

    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) || ( NULL == *mp4parse ) )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    mp4p = (mp4parse_t*) *mp4parse;


    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_destroy, mp4parse invalid.\n" );
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &mp4p->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    /* requirement: to allow destroy we need to be ready for it */
    if ( NO_ERR == myerr )
    {
        if ( ( mp4p->status != MP4PARSE_CSTATUS_CREATED ) &&
             ( mp4p->status != MP4PARSE_CSTATUS_STOPPED ) &&
             ( mp4p->status != MP4PARSE_CSTATUS_FLUSHED ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_destroy, not in a STOPPED/FLUSHED state.  \n", mp4p->label );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[MP4PARSE][%s] mp4parse_destroy \n", mp4p->label );

        /* update the status */
        mp4p->status = MP4PARSE_CSTATUS_STOPPED;

        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, mp4p->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( MP4PARSE_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, mp4p->ctrl_mtx );

        /* force the exit */
        destroy_mp4parse( &myerr, (mp4parse_t**) mp4parse );

        /* erase the reference! */
        *mp4parse = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


/*
 * Function: mp4parse_set_log_verbosity
 *
 */
void mp4parse_set_log_verbosity( int8_t* err, void* mp4parse, uint8_t lvl )
{
    int8_t      myerr = NO_ERR;

    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;

    /* sanity check */
    if ( NULL == mp4p )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][-] [ERROR] mp4parse_set_log_verbosity, mp4parse invalid.\n" );
        }
    }

    if ( NO_ERR == myerr )
    {
        mp4p->log_lvl = lvl;
        mp4p->log_err = LOG_ALWAYS;

        LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4parse_set_log_verbosity: %" PRIu8 "\n", mp4p->label, lvl );
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void mp4parse_start( int8_t* err,
                     void*   mp4parse )
{
    int8_t            myerr = NO_ERR;

    mp4parse_t*       mp4p = (mp4parse_t*) mp4parse;

    volatile uint8_t* ctrl = NULL;

    uint8_t           tmp8 = 0;
    uint32_t          tmp32 = 0;

    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &mp4p->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        /* make sure we have all we need*/
        check_ready_mp4parse( &myerr, mp4p );
    }
    else
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    /* SET the on/off CTRL register flag                                  */
    /* internal threads will exit immediately if you do not set this flag */
    if ( NO_ERR == myerr )
    {
        TSP_MUTEX_LOCK( NULL, mp4p->ctrl_mtx );
        *ctrl = ( *ctrl ) | MP4PARSE_CTRL_ONOFF_STATUS;
        TSP_MUTEX_UNLOCK( NULL, mp4p->ctrl_mtx );
    }

    /*
     * now start the component (inner thread)
     */

    /* START audio thread: DSP */
    if ( NO_ERR == myerr )
    {
        TSP_THREAD_CREATE( myerr,
                           mp4p->mp4_thread,
                           mp4p->mp4_thread_attr,
                           mp4parse_dsp_loop,
                           mp4p,
                           MP4PARSE_DEFAULT_PRIORITY,
                           MP4PARSE_DEFAULT_STACK_SIZE,
                           MP4PARSE_DEFAULT_LABEL );

        if ( myerr != NO_ERR )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] [ERROR] cannot create MP4PARSE thread\n", mp4p->label );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }
    else
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[MP4PARSE][%s] [ERROR] skipping create MP4PARSE thread\n", mp4p->label );
    }


    /* control loop for subsystem healty status */
    if ( myerr == NO_ERR )
    {
        if ( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS )
        {
            /*
             * wait until the subsystems are ready
             */
            tmp32 = 0;

            do
            {
                tmp8 = ( mp4p->ctrl & MP4PARSE_CTRL_DSP_STATUS );

                if ( 0 == ( tmp32 & 0x0FF ) )
                {
                    LOG_MSG( mp4p->log_lvl, LOG_LOW,
                             "[MP4PARSE][%s] waiting for sub-components CTRL=%" PRIu8 "\n", mp4p->label, mp4p->ctrl );
                }

                tmp32++;

                SYSCLK_USLEEP( uSLEEP1ms );
            } while ( !( tmp8 ) && ( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) && ( tmp32 < MP4PARSE_TIMEOUT_MS ) );


            /* flag an error if we did not start in TIMEOUT ms */
            if ( !tmp8 )
            {
                LOG_ERR( 0, LOG_ALWAYS,
                         "[MP4PARSE][%s] [ERROR] mp4parse_start, could not start,CTRL=%" PRIu8 "\n", mp4p->label, mp4p->ctrl );

                myerr = MP4PARSE_ERR_UNKNOWN;
            }
            else
            {
                LOG_MSG( mp4p->log_lvl, LOG_LOW,
                         "[MP4PARSE][%s] mp4parse_running, CTRL=%" PRIu8 "\n", mp4p->label, mp4p->ctrl );

                /* ready update status */
                mp4p->status = MP4PARSE_CSTATUS_RUNNED;
            }
        } // (*ctrl) | MP4PARSE_CTRL_ONOFF_STATUS
    }

    /* auto shutoff on failure */
    if ( myerr != NO_ERR )
    {
        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, mp4p->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( MP4PARSE_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, mp4p->ctrl_mtx );
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void mp4parse_stop( int8_t* err,
                    void*   mp4parse )
{
    int8_t      myerr = NO_ERR;

    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;


    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */

    /*
     * if(NULL!=err)
     * {
     *  if(NO_ERR!=(*err))
     *  {
     *      return;
     *  }
     * }
     */

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( MP4PARSE_CSTATUS_READY > mp4p->status )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] mp4parse_stop.\n", mp4p->label );
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        if ( MP4PARSE_CSTATUS_STOPPED != mp4p->status )
        {
            LOG_MSG( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] mp4parse STOPPED.\n", mp4p->label );

            mp4p->status = MP4PARSE_CSTATUS_STOPPED;
        }
    }
    else
    {
        if ( NULL == mp4p )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] [ERROR] mp4parse not ready for STOP (status=%" PRIu16 ")\n", mp4p->label, mp4p->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void mp4parse_flush( int8_t* err,
                     void*   mp4parse )
{
    int8_t            myerr = NO_ERR;

    volatile uint8_t* ctrl = NULL;

    mp4parse_t*       mp4p = (mp4parse_t*) mp4parse;


    /* sanity check on the pointers */
    if ( ( NULL == mp4parse ) )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */

    /*
     * if(NULL!=err)
     * {
     *  if(NO_ERR!=(*err))
     *  {
     *      return;
     *  }
     * }
     *
     *
     */

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &mp4p->ctrl;
        }
    }

    if ( NO_ERR == myerr )
    {
        // if(MP4PARSE_CSTATUS_READY > mp4p->status)
        if ( MP4PARSE_CSTATUS_STOPPED != mp4p->status )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s][ERROR] mp4parse_flush, not in a STOPPED state\n", mp4p->label );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( ctrl == NULL )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s][ERROR] mp4parse_flush, no ctrl\n", mp4p->label );
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4parse_flush, FLUSHING...\n", mp4p->label );

        while ( ( NO_ERR == myerr ) &&
                ( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) &&
                ( ( *ctrl ) & MP4PARSE_CTRL_DSP_STATUS ) &&
                ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status ) )
        {
            /* trigger DSP parser for decoding */
            TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
            SYSCLK_USLEEP( uSLEEP10ms );
        }

        LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] mp4parse_flush, FLUSHING... done.\n", mp4p->label );

        mp4p->status = MP4PARSE_CSTATUS_FLUSHED;
    }
    else
    {
        if ( NULL == mp4p )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR] mp4parse_flish mp4parse is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] [ERROR] mp4parse not ready for FLUSH (status=%" PRIu16 ")\n", mp4p->label, mp4p->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


/* ***************************** */
/* SETTERS                       */
/* ***************************** */
void mp4parse_set_tpconfig( int8_t*    err,
                            void*      mp4parse,
                            void*      tpconfig,
                            tpm_tool_t tool_type )
{
    int8_t      myerr = NO_ERR;
    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;

    /* sanity check */
    if ( NULL == mp4p )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != mp4p ) )
    {
        if ( tool_type < TPM_TOOL_MAX )
        {
            mp4p->tpm.tool_type   = tool_type;
            mp4p->tpm.tool_config = tpconfig;
        }
        else
        {
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void mp4parse_set_start_offset( int8_t*  err,
                                void*    mp4parse,
                                uint32_t start_offset_ms)
{
    int8_t      myerr = NO_ERR;
    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;

    /* sanity check */
    if ( NULL == mp4p )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        if (MP4PARSE_CSTATUS_CREATED == mp4p->status)
        {
            mp4p->start_offset_ms = start_offset_ms;
        }
        else
        {
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void mp4parse_set_stream_position( int8_t*  err, void* mp4parse, uint32_t stream_position )
{
    int8_t      myerr = NO_ERR;
    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;

    /* sanity check */
    if ( NULL == mp4p )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        if (MP4PARSE_CSTATUS_CREATED <= mp4p->status)
        {
            mp4p->data_bitstream_idx       = stream_position;
            mp4p->wait_for_stream_position = 0;
        }
        else
        {
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void mp4parse_set_output_push_callback( int8_t*             err,
                                        void*               mp4parse,
                                        uint32_t ( *        output_push_callback )(
                                            int8_t*         err,
                                            void*           mp4parse,
                                            void*           decoder,
                                            mp4_pcm_info_t* pcm_info,
                                            int16_t*        buf,
                                            uint32_t        buf_len,
                                            uint32_t*       buf_id ) )
{
    int8_t      myerr = NO_ERR;
    mp4parse_t* mp4p = (mp4parse_t*) mp4parse;

    /* sanity check */
    if ( NULL == mp4p )
    {
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != mp4p ) )
    {
        if ( NULL == mp4p->output_push_callback )
        {
            mp4p->output_push_callback = output_push_callback;
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] set_output_push_CBACK: failed\n", mp4p->label );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


uint32_t mp4parse_input_push_buf( int8_t*  err,
                                  void*    mp4parse,
                                  char*    buf,
                                  uint32_t buf_len,
                                  uint8_t  buf_type )
{
    int8_t              myerr = NO_ERR;
    mp4parse_t*         mp4p = NULL;
    volatile uint8_t*   ctrl = NULL;
    uint32_t            push_byte_cnt = 0;

    uint8_t GCC_UNUSED* mp4_box      = NULL;
    uint32_t GCC_UNUSED mp4_box_size = 0;

    uint8_t             search_complete_f = FALSE;
    uint8_t             do_reset_f;

    /* sanity check */
    if ( NULL == mp4parse )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }
        /* bail out on invalid component */
        return 0;
    }

    /* check for previous errors */
#if 0
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return 0;
        }
    }
#endif

    /* retrieve component base obj */
    mp4p = (mp4parse_t*) mp4parse;

    /* retrieve the control register */
    ctrl =  (volatile uint8_t*) &mp4p->ctrl;

    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
            goto _exit;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        /* check for running component */
        if ( ( MP4PARSE_CSTATUS_RUNNED != mp4p->status ) ||
             ( !( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] [ERROR] input_push: mp4parse is NOT running (CTRL=%" PRIu8 ")\n", mp4p->label, mp4p->ctrl );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    /* sanity check:
     * DSP must be alive
     */
    if ( !( ( *ctrl ) & MP4PARSE_CTRL_DSP_STATUS ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[MP4PARSE][%s] [ERROR] input_push: mp4parse DSP anomalous exit (CTRL=%" PRIu8 ")\n", mp4p->label, mp4p->ctrl );

        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* sanity check:
     * buffer must be valid in size
     */
    if ( ( NULL == buf ) || ( buf_len == 0 ) || ( buf_len > RTP_PKT_MTU_SIZE ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[MP4PARSE][%s] [ERROR] input_push_buf: invalid buffer\n", mp4p->label );
        myerr = MP4PARSE_ERR_UNKNOWN;
    }

    /* bail out if needed */
    if ( NO_ERR != myerr )
    {
        goto _exit;
    }

    if (mp4p->wait_for_stream_position != 0)
    {
        /*
         * If we're waiting for a new stream position, drop this packet and return.
         */

        push_byte_cnt = buf_len;
        goto _exit;
    }

    /*
     * MAIN: add pkt into the input queue
     */

    /* validate search BEFORE each push */

    search_complete_f = SEARCH_COMPLETE(mp4p);

    if ( !search_complete_f )
    {
        if ( ( BUF_NUM <= ( mp4p->data_fifo_w_idx + 1 ) ) ||
             ( BUF_RAW_SIZE <= ( mp4p->data_raw_w_idx + buf_len ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] MP4 SEARCH fifo overflow error, %" PRIu32 ", %" PRIu32 "\n",
                     mp4p->label,
                     mp4p->data_fifo_w_idx,
                     mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw_idx );

            /* no more space in the queue for atoms search */
            myerr = MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW;
            goto _exit;
        }


        /*
         * skip incrementing if we need to discard data
         */
        if ( mp4p->data_bitstream_skip_f == TRUE )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] ==>SAME SLOT! data_raw_w_idx=%" PRIu32 " data_size_prev=%" PRIu64 "\n",
                     mp4p->label,
                     mp4p->data_raw_w_idx,
                     mp4p->data_size_prev );

            mp4p->data_raw_w_idx -= mp4p->data_size_prev;
        }

        /*
         * getBuffer
         */
        LOG_MSG( mp4p->log_lvl, LOG_HIGH, "[MP4PARSE][%s] slot #%" PRIu32 "\n", mp4p->label, mp4p->data_fifo_w_idx );

        // mp4p->data_fifo[ mp4p->data_fifo_w_idx ].status  = 1;
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].length  = buf_len;
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw_idx = mp4p->data_raw_w_idx;
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw     = &mp4p->data_raw[ mp4p->data_raw_w_idx ];
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].bitstream_idx = mp4p->data_bitstream_idx;
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].type    = buf_type;

        /*
         * copy
         */
        LOG_MSG( 0, LOG_LOW, "[MP4PARSE][%s] input_push_buf (s) data.idx=%" PRIu32 ", raw.idx=%" PRIu32 ", size=%" PRIu32 "\n",
                 mp4p->label,
                 mp4p->data_fifo_w_idx,
                 mp4p->data_raw_w_idx,
                 buf_len );

        memcpy( mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw, buf, buf_len );

        /* mark the slot only AFTER being populated */
        mp4p->data_fifo[ mp4p->data_fifo_w_idx ].status  = 1;


        push_byte_cnt = buf_len;

        /* PROFILING/TraceX */
        TPM_USRLOG( NULL, &mp4p->tpm,
                    TPM_MP4_INPUT_PARSE_PUSH,
                    mp4p->data_fifo_w_idx,
                    mp4p->data_fifo_r_idx,
                    mp4p->data_raw_w_idx,
                    buf_len );

        /*
         * grow raw data fifo
         */
        mp4p->data_bitstream_idx += buf_len;

        mp4p->data_raw_w_idx += buf_len;

        /* keep track of the previous push */
        mp4p->data_size_prev = buf_len;

        /*
         * initial search: find or store in local data fifo
         */

        mp4_find_audioSpecificInfo( &myerr, mp4p, &mp4_box, &mp4_box_size );

        /* move index */
        if ( mp4p->data_bitstream_skip_f == FALSE )
        {
            mp4p->data_fifo_w_idx = ( mp4p->data_fifo_w_idx + 1 ) & mp4p->data_fifo_wrap;
        }
        else
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] ==>SAME SLOT!\n", mp4p->label );
        }

        /* validate search AFTER each push and reset fifo if needed */
        search_complete_f = SEARCH_COMPLETE(mp4p);

        if ( search_complete_f )
        {
            LOG_MSG( mp4p->log_lvl, LOG_LOW, "[MP4PARSE][%s] SEARCH COMPLETE, reset fifo\n", mp4p->label );

            do_reset_f = 1;
            if (mp4p->start_offset_ms > 0)
            {
                if (mp4_initiate_start_offset(mp4p) == 0)
                {
                    /*
                     * Reset the input buffers.
                     */
                    memset(mp4p->data_fifo, 0, BUF_NUM * sizeof(data_buf_t));
                    mp4p->data_fifo_r_idx = 0;
                    mp4p->data_fifo_w_idx = 0;
                    mp4p->data_raw_w_idx  = 0;
                    do_reset_f            = 0;
                }
            }

            if (do_reset_f)
            {
                /* keep last slot if we're not waiting for a new stream position */
                uint32_t i_curr = ( mp4p->data_fifo_w_idx - 1 ) & mp4p->data_fifo_wrap;

                uint32_t i;
                for ( i = 0; i < BUF_NUM; i++ )
                {
                    if ( i != i_curr )
                    {
                        mp4p->data_fifo[ i ].status  = 0;
                        mp4p->data_fifo[ i ].length  = 0;
                        mp4p->data_fifo[ i ].raw_idx = 0;
                        mp4p->data_fifo[ i ].raw     = NULL;
                    }
                }

                mp4p->data_fifo_r_idx = i_curr;
            }
        }
    }
    else
    {
        /*
         * sanity check on descriptor fifo
         */
        if ( mp4p->data_fifo[ ( mp4p->data_fifo_w_idx + 0 ) & mp4p->data_fifo_wrap ].status != 0 )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[MP4PARSE][%s] [ERROR]] overflow on INPUT FIFO, w_idx=%" PRIu32 " (status=%" PRIu8 ")\n",
                     mp4p->label,
                     mp4p->data_fifo_w_idx,
                     mp4p->data_fifo[ mp4p->data_fifo_w_idx ].status );

            myerr = MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW;
            // myerr = MP4PARSE_ERR_UNKNOWN;

            /* trigger DSP parser for decoding */
            TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
        }

        /*
         * sanity check on raw fifo
         */
        {
            // uint32_t w_fifo_idx = (mp4p->data_fifo_w_idx-1) & mp4p->data_fifo_wrap;

            uint32_t GCC_UNUSED w_raw_idx = mp4p->data_fifo[ ( mp4p->data_fifo_w_idx - 1 ) & mp4p->data_fifo_wrap ].raw_idx;
            uint32_t            r_raw_idx = mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx;

            /* check  written data */
            if ( ( mp4p->data_fifo[ ( mp4p->data_fifo_w_idx - 1 ) & mp4p->data_fifo_wrap ].status ) &&
                 ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status ) )

            // if( (mp4p->data_fifo[w_fifo_idx].status ) &&
            // (mp4p->data_fifo[mp4p->data_fifo_r_idx].status) &&
            // (w_fifo_idx!=mp4p->data_fifo_r_idx))
            {
                // if( buf_len > ((r_raw_idx - w_raw_idx + BUF_RAW_SIZE) & mp4p->data_raw_wrap))

                if ( buf_len > ( ( r_raw_idx - mp4p->data_raw_w_idx + BUF_RAW_SIZE ) & mp4p->data_raw_wrap ) )
                {
#if 0
                    LOG_ERR( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] [ERROR] BUF_CHECK overflow on INPUT RAW FIFO, "
                                                        "w_raw_idx=%" PRIu32 " (prev %" PRIu32 ") r_raw_idx=%" PRIu32 " diff=%" PRIu32 "->%" PRIu32 "->%" PRIu32 ") wrap=%" PRIu32 "\n",
                             mp4p->label,
                             mp4p->data_raw_w_idx,
                             w_raw_idx,
                             r_raw_idx,
                             r_raw_idx - w_raw_idx,
                             ( r_raw_idx - w_raw_idx + BUF_RAW_SIZE ),
                             ( ( r_raw_idx - w_raw_idx + BUF_RAW_SIZE ) & mp4p->data_raw_wrap ),
                             (uint32_t) mp4p->data_raw_wrap
                           );
#endif
                    myerr = MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW;

                    /* trigger DSP parser for decoding */
                    TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
                }
            }
        }

        /*
         *  REWIND sanity check
         */
        if ( NO_ERR == myerr )
        {
            /* circular buffer on raw data, potentially needs (buf_len, max PKT_SIZE) space */
            // if ( ( mp4p->data_raw_w_idx + RTP_PKT_MTU_SIZE ) >= BUF_RAW_SIZE )
            if ( ( mp4p->data_raw_w_idx + buf_len ) >= BUF_RAW_SIZE )
            {
                /* we are going to rewind the fifo, make sure we do not overlap */
                /* with the current read idx */

                if ( ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].status != 0 ) &&
                     ( mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx <= buf_len ) )
                {
                    LOG_ERR( mp4p->log_lvl, LOG_MEDIUM,
                             "[MP4PARSE][%s] [ERROR] BUF_CHECK overflow on REWIND for INPUT RAW FIFO, "
                             "w_raw_idx=%" PRIu32 " fifo_r_idx=%" PRIu32 " r_raw_idx=%" PRIu32 "\n",
                             mp4p->label,
                             mp4p->data_raw_w_idx,
                             mp4p->data_fifo_r_idx,
                             mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx
                           );

                    myerr = MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW;

                    /* trigger DSP parser for decoding */
                    TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
                }
                else
                {
                    LOG_MSG( mp4p->log_lvl, LOG_MEDIUM, "[MP4PARSE][%s] input_push_buf (p) --> REWIND f.idx=%" PRIu32 ", raw.w.idx=%" PRIu32 ", raw.r.idx=%" PRIu32 ", size=%" PRIu32 "\n",
                             mp4p->label,
                             mp4p->data_fifo_w_idx,
                             mp4p->data_fifo[ ( mp4p->data_fifo_w_idx - 1 ) & mp4p->data_fifo_wrap ].raw_idx,
                             mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx,
                             BUF_RAW_SIZE );

                    mp4p->data_raw_w_idx = 0;
                }
            }
        }

        if ( NO_ERR == myerr )
        {
            /* PROFILING/TraceX */
            TPM_USRLOG( NULL, &mp4p->tpm,
                        TPM_MP4_INPUT_AAC_PUSH,
                        mp4p->data_fifo_w_idx,
                        mp4p->data_fifo_r_idx,
                        mp4p->data_raw_w_idx,
                        buf_len );

            /* describe the buffer */
            LOG_MSG( 0, LOG_LOW, "[MP4PARSE][%s] input_push_buf (p) f.idx=%" PRIu32 ", raw.w.idx=%" PRIu32 ", raw.w=%" PRIu32 ", raw.r=%" PRIu32 ", size=%" PRIu32 " \n",
                     mp4p->label,
                     mp4p->data_fifo_w_idx,
                     mp4p->data_raw_w_idx,
                     mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw_idx,
                     mp4p->data_fifo[ mp4p->data_fifo_r_idx ].raw_idx,
                     buf_len
                   );

            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw_idx = mp4p->data_raw_w_idx;
            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw     = &mp4p->data_raw[ mp4p->data_raw_w_idx ];

            /*
             * copy
             */

            memcpy( mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw, buf, buf_len );

            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].length        = buf_len;
            // mp4p->data_fifo[ mp4p->data_fifo_w_idx ].raw_idx       = mp4p->data_raw_w_idx;
            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].bitstream_idx = mp4p->data_bitstream_idx;
            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].type          = buf_type;


            /* mark the slot only AFTER being populated */
            mp4p->data_fifo[ mp4p->data_fifo_w_idx ].status  = 1;


            /* indexing fifo for next data */
            mp4p->data_fifo_w_idx = ( mp4p->data_fifo_w_idx + 1 ) & mp4p->data_fifo_wrap;

            /* indexing raw data */
            mp4p->data_raw_w_idx = ( mp4p->data_raw_w_idx + buf_len ) & mp4p->data_raw_wrap;

            /* bitstream is increasing over 64 bit */
            mp4p->data_bitstream_idx += buf_len;

            /* trigger DSP parser for decoding */
            TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
        }


        if ( myerr == MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW )
        {
            /* trigger DSP parser for decoding */
            TSP_SEMAPHORE_POST( NULL, mp4p->mp4_thread_sem );
        }
    }


_exit:
    if ( NULL != err )
    {
        *err = myerr;
    }

    return push_byte_cnt;
}


/* ******************************************************************************* */
/* GETTERS                                                                         */
/* ******************************************************************************* */
void mp4parse_get_trackInfo( int8_t*           err,
                             void*             mp4parse,
                             mp4_track_info_t* mp4_track_info )
{
    int8_t            myerr = NO_ERR;
    mp4parse_t*       mp4p = NULL;
    volatile uint8_t* ctrl = NULL;

    /* sanity check */
    if ( ( NULL == mp4parse ) || ( NULL == mp4_track_info ) )
    {
        if ( NULL != err )
        {
            *err = MP4PARSE_ERR_UNKNOWN;
        }

        return;
    }

    /* retrieve component base obj */
    mp4p = (mp4parse_t*) mp4parse;

    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_mp4parse( NULL, mp4p, MP4PARSE_CTYPE_ANYTYPE ) )
        {
            myerr = MP4PARSE_ERR_UNKNOWN;
            return;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        /* retrieve the control register */
        ctrl =  (volatile uint8_t*) &mp4p->ctrl;

        /* check for running component */
        if ( ( MP4PARSE_CSTATUS_RUNNED != mp4p->status ) ||
             ( !( ( *ctrl ) & MP4PARSE_CTRL_ONOFF_STATUS ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[MP4PARSE][%s] [ERROR] input_push: mp4parse is NOT running (CTRL=%" PRIu8 ")\n", mp4p->label, mp4p->ctrl );
            myerr = MP4PARSE_ERR_UNKNOWN;
        }
    }

    /* retrieve info */
    if ( NO_ERR == myerr )
    {
        if ( mp4p->mp4_audio_track_info.duration != 0 )
        {
            mp4_track_info->creation_time     = mp4p->mp4_audio_track_info.creation_time;
            mp4_track_info->modification_time = mp4p->mp4_audio_track_info.modification_time;
            mp4_track_info->timescale         = mp4p->mp4_audio_track_info.timescale;
            mp4_track_info->duration          = mp4p->mp4_audio_track_info.duration;
        }
        else
        {
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}
