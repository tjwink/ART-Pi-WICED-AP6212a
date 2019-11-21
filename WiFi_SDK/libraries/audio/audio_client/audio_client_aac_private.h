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

/**
 * @file
 *
 * AAC decoder internal definitions.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "aacdecoder_lib.h"

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_CLIENT_DECODER_AAC_THREAD_PRIORITY    ( WICED_DEFAULT_LIBRARY_PRIORITY )
#define AUDIO_CLIENT_DECODER_AAC_STACK_SIZE         ( 8 * 1024 )

#define COALESCE_BUF_SIZE                           ( 48 * 1024 )
#define DECODE_BUF_SIZE                             ( 8 * 2 * 1024 )

#define ADTS_HEADER_LEN                             (7) /* Without CRC */

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    ADTS_HDR_AAC_MAIN = 0,     /* AAC Main               */
    ADTS_HDR_AAC_LC,           /* Low Complexity         */
    ADTS_HDR_AAC_SSR,          /* Scalable Sampling Rate */
    ADTS_HDR_AAC_LTP,          /* RESERVED for MPEG-2, Long Term Prediction for MPEG-4 */
    ADTS_HDR_AAC_PROFILE_MAX
} adts_hdr_profile_t;

typedef enum
{
    ADTS_HDR_SAMPLING_RATE_96000 = 0,
    ADTS_HDR_SAMPLING_RATE_88200,
    ADTS_HDR_SAMPLING_RATE_64000,
    ADTS_HDR_SAMPLING_RATE_48000,
    ADTS_HDR_SAMPLING_RATE_44100,
    ADTS_HDR_SAMPLING_RATE_32000,
    ADTS_HDR_SAMPLING_RATE_24000,
    ADTS_HDR_SAMPLING_RATE_22050,
    ADTS_HDR_SAMPLING_RATE_16000,
    ADTS_HDR_SAMPLING_RATE_12000,
    ADTS_HDR_SAMPLING_RATE_11025,
    ADTS_HDR_SAMPLING_RATE_8000,
    ADTS_HDR_SAMPLING_RATE_7350,
    ADTS_HDR_SAMPLING_RATE_MAX
} adts_hdr_sampling_rate_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**
 *
 * PCM INFO:
 * expanded structure for pcm details, this is used only for in/out API,
 *
 */
typedef struct
{
    uint8_t  bps;               /* bps:  16 or 24                           */
    uint8_t  cbps;              /* cbps: equal to bps (16/24) or 32         */
    uint8_t  chnum;             /* number of channels                       */
    uint32_t sr;                /* sample rate: 44100, 48000, 96000, 192000 */
} aac_pcm_info_t;

/**
 * ADTS headers, see ISO/IEC 13818-7 section 6.2.1 / 6.2.2
 */

typedef struct
{
    unsigned id                              : 1;
    unsigned layer                           : 2;
    unsigned protection_absent               : 1;
    unsigned profile                         : 2;
    unsigned sampling_frequency_idx          : 4;
    unsigned private_bit                     : 1;
    unsigned channel_configuration           : 3;
    unsigned original_copy                   : 1;
    unsigned home                            : 1;
} adts_fixed_hdr_t;

typedef struct
{
    unsigned copyright_identification_bit    : 1;
    unsigned copyright_identification_start  : 1;
    unsigned aac_frame_length                : 13;
    unsigned adts_buffer_fullness            : 11;
    unsigned num_raw_data_blocks_in_frame    : 2;
} adts_variable_hdr_t;

typedef struct
{
    uint8_t             is_valid;
    adts_fixed_hdr_t    fixed;
    adts_variable_hdr_t variable;
} adts_hdr_t;


/**
 * AAC/M4A information on audio track
 * iso/iec 14496-12 8.4.2.1
 */
typedef struct
{
    uint64_t creation_time;
    uint64_t modification_time;
    uint32_t timescale;
    uint64_t duration;
} m4a_track_info_t;

typedef struct
{
    uint8_t* data;
    uint32_t data_size;
    uint32_t data_idx;
} m4a_table_data_t;

/* STSZ/STZ2 (14496-12) */
typedef struct
{
    uint8_t             table_variant;
    uint32_t            sample_size;
    uint32_t            sample_count;
    uint32_t            sample_count_idx;
    uint8_t             field_size;
    m4a_table_data_t    table;
} m4a_sample_size_table_t;


/* STCO/CO64 (14496-12) */
typedef struct
{
    uint8_t             table_variant;
    uint32_t            entry_count;
    uint32_t            entry_count_idx;
    m4a_table_data_t    table;
} m4a_chunk_offset_table_t;


/* STSC (14496-12) */
typedef struct
{
    uint32_t            entry_count;
    uint32_t            entry_count_idx;
    uint32_t            sample_in_chunk_idx;
    uint32_t            sample_in_chunk_offset;
    m4a_table_data_t    table;
} m4a_sample_chunk_table_t;


/* STTS (14496-12) */
typedef struct
{
    uint32_t            entry_count;
    m4a_table_data_t    table;
} m4a_time_sample_table_t;


typedef struct
{
    uint32_t                    tag;

    wiced_bool_t                eos_signal;
    wiced_bool_t                no_skip_output;

    wiced_bool_t                pcm_first_buffer;
    wiced_audio_config_t        audio_config;

    audio_client_t*             audio_client;

    /*
     *  Audio format information.
     */

    uint16_t                    num_channels;
    uint32_t                    sample_rate;
    uint32_t                    byte_rate;
    uint16_t                    block_align;
    uint16_t                    bits_per_sample;

    /*
     * Some housekeeping variables.
     */

    uint32_t                    audio_frames_total;
    uint32_t                    audio_frames_played;
    uint32_t                    audio_frames_offset;

    /*
     * FDK housekeeping.
     */

    HANDLE_AACDECODER           fdk_handle;
    uint32_t                    fdk_internal_buf_size;
    uint32_t                    search_status;
    uint8_t*                    data_decode;

    /*
     * ADTS housekeeping.
     */

    uint8_t                     adts_header[ADTS_HEADER_LEN];
    uint8_t                     adts_header_idx;

    /*
     * M4A specific housekeeping.
     */

    volatile uint32_t           wait_for_stream_position;
    uint8_t*                    m4a_audio_specific_info;
    uint32_t                    m4a_audio_specific_info_size;
    wiced_bool_t                m4a_decoder_configured;

    uint32_t                    collecting_box_type;
    uint32_t                    collecting_box_size;

    m4a_track_info_t            m4a_track_info;
    m4a_sample_size_table_t     m4a_sample_size_table;
    m4a_chunk_offset_table_t    m4a_chunk_offset_table;
    m4a_time_sample_table_t     m4a_time_sample_table;
    m4a_sample_chunk_table_t    m4a_sample_chunk_table;

    wiced_bool_t                m4a_stream_seek;
    wiced_bool_t                m4a_chunk_lookup;

    uint64_t                    m4a_chunk_bitstream_offset;
    uint64_t                    m4a_sample_chunk_offset;
    uint32_t                    m4a_sample_size;
    uint32_t                    m4a_sample_bytes_pushed;

    uint8_t*                    data_coalesce;
    uint32_t                    data_widx;
    uint32_t                    data_ridx;
    uint64_t                    data_bitstream_idx;         /* Stream byte offset of beginning of data_coalesce */
    uint64_t                    bitstream_idx;              /* How many bytes have been received in the stream  */

    uint32_t                    data_skip_bytes;

    /*
     * Don't forget space for our stack.
     */

    uint8_t                     stack[AUDIO_CLIENT_DECODER_AAC_STACK_SIZE];

} aac_decoder_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

int aac_start_offset_callback(aac_decoder_t* dec, uint32_t start_offset_ms, uint32_t start_offset_bytes);

void aac_release_buffer(audio_client_t* client, data_buf_t* dbuf);
wiced_result_t aac_pcm_output_push(aac_decoder_t* dec, aac_pcm_info_t* pcm_info, uint8_t* buf, uint32_t buf_len);

wiced_result_t adts_process_data(audio_client_t* client);
wiced_result_t m4a_process_data(audio_client_t* client);


#ifdef __cplusplus
} /*extern "C" */
#endif
