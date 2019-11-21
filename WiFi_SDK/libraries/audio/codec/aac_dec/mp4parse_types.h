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

#ifndef _H_MP4PARSE_TYPES_H_
#define _H_MP4PARSE_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>



/**
 * MP4PARSE api error types
 */
typedef enum
{
    MP4PARSE_ERR_NONE                  =  0,
    MP4PARSE_ERR_UNKNOWN               = -1,
    MP4PARSE_ERR_NEED_MORE_BITS        = -2,
    MP4PARSE_BITSTREAM_FAILURE         = -3,
    MP4PARSE_ERR_OUT_OF_MEMORY         = -4,
    MP4PARSE_ERR_INPUT_QUEUE_OVERFLOW  = -5,
    MP4PARSE_ERR_END_OF_STREAM         = -6
} MP4PARSE_ERR;


/**
 * MP4PARSE PCM callback inline buffer id.
 * to be used for two purposes:
 * a) in band signaling when needed by the upper player
 * b) to randomly mark get/release of output buffers
 *    for zero copy push (on release the random id is checked)
 *
 * NOTE: the  mp4parse_set_output_push_callback id is a UINT32_T
 */
typedef enum
{
    PCMBUF_ID_CMD_EOP_OK    = 0,     /* end of playback ok */
    PCMBUF_ID_CMD_EOP_ERR   = 1,     /* end of playback error, see payload */

    /*
     * all intermediate values
     * are reserved
     */
    PCMBUF_ID_PCM_RNDMIN   = ( 1 << 31 ),

    /*
     * the UPPER 31 bits are used for random id marks.
     */
    PCMBUF_ID_PCM_RNDMAX   = ( 0xFFFFFFFE ),
    PCMBUF_ID_MAX          = ( 0xFFFFFFFF )
} PCMBUF_ID;



/**
 *
 * PCM INFO:
 * expanded structure for pcm details, this is used only for in/out API,
 * but not for each buffer descriptor.
 *
 */
typedef struct mp4_pcm_info_
{
    uint8_t  bps;               /* bps:  16 or 24                           */
    uint8_t  cbps;              /* cbps: equal to bps (16/24) or 32         */
    uint8_t  chnum;             /* number of channels                       */
    uint32_t sr;                /* sample rate: 44100, 48000, 96000, 192000 */
    uint32_t chmap;             /* 32bit channel map BITMASK                */
    unsigned endian : 1;        /* endianness: 0=little 1=big               */
    unsigned sign : 1;          /* signedness: 0=unsigned 1=signed          */
    unsigned interleave : 1;    /* interleaving: 0=coalesced 1=interleaved  */
    unsigned floating : 1;      /* floating: 0=integer 1=floating           */
} mp4_pcm_info_t;


/**
 * AAC/M4A informations on audio track
 * iso/iec 14496-12 8.4.2.1
 */
typedef struct mp4_track_info_
{
    uint64_t creation_time;
    uint64_t modification_time;
    uint32_t timescale;
    uint64_t duration;
} mp4_track_info_t;


/* STSZ/STZ2 (14496-12) */
typedef struct mp4_sample_size_table_
{
    uint8_t  table_variant;
    uint32_t sample_size;
    uint32_t sample_count;
    uint32_t sample_count_idx;
    uint8_t  field_size;
    uint8_t* byte_data;
    uint32_t byte_data_size;
    uint32_t byte_data_idx;
    uint8_t  byte_data_complete_f;
} mp4_sample_size_table_t;


/* STCO/CO64 (14496-12) */
typedef struct mp4_chunk_offs_table_
{
    uint8_t  table_variant;
    uint32_t entry_count;
    uint32_t entry_count_idx;
    uint8_t* byte_data;
    uint32_t byte_data_size;
    uint32_t byte_data_idx;
    uint8_t  byte_data_complete_f;
} mp4_chunk_offs_table_t;


/* STSC (14496-12) */
typedef struct mp4_sample_chunk_table_
{
    uint32_t entry_count;
    uint32_t entry_count_idx;
    uint8_t* byte_data;
    uint32_t byte_data_size;
    uint32_t byte_data_idx;
    uint8_t  byte_data_complete_f;
    uint32_t sample_in_chunk_idx;
    uint32_t sample_in_chunk_offs;
} mp4_sample_chunk_table_t;


/* STTS (14496-12) */
typedef struct mp4_time_sample_table_
{
    uint32_t entry_count;
    uint8_t* byte_data;
    uint32_t byte_data_size;
    uint32_t byte_data_idx;
    uint8_t  byte_data_complete_f;
} mp4_time_sample_table_t;


/* empty box */
typedef struct mp4_box_discard_
{
    uint32_t byte_data_size;
    uint32_t byte_data_idx;
    uint8_t  byte_data_complete_f;
} mp4_box_discard_t;



/**
 * *****************************************************************
 * Component TYPE & STATUS definition
 * *****************************************************************
 */

/* control register status flags */
#define MP4PARSE_CTRL_ONOFF_STATUS    ( 1 << 0 )
#define MP4PARSE_CTRL_DSP_STATUS      ( 1 << 2 )


/* inner components watermarking
 * definitions are GLOBALS to force uniqueness of the WMARKs
 */
#define MP4PARSE_WMARK       ( (uint32_t) ( 0xefb0bb12 ) )

/**
 * component type (ctype)
 */
typedef enum
{
    MP4PARSE_CTYPE_UNKNOWN       = 0,
    MP4PARSE_CTYPE_ANYTYPE       = 1,
    MP4PARSE_CTYPE_RESERVED      = 2,
    MP4PARSE_CTYPE_SINGLE_THREAD = 3,
    MP4PARSE_CTYPE_MULTI_THREAD  = 4,
    /**/
    MP4PARSE_CTYPE_MAX
} mp4parse_ctype_t;


/**
 * component status (cstatus)
 */
typedef enum
{
    MP4PARSE_CSTATUS_UNKNOWN  = 0,
    MP4PARSE_CSTATUS_ANYTYPE  = 1,
    MP4PARSE_CSTATUS_RESERVED = 2,
    MP4PARSE_CSTATUS_CREATED  = 3,
    MP4PARSE_CSTATUS_READY    = 4,
    MP4PARSE_CSTATUS_RUNNED   = 5,
    MP4PARSE_CSTATUS_STOPPED  = 6,
    MP4PARSE_CSTATUS_FLUSHED  = 7,
    /**/
    MP4PARSE_CSTATUS_MAX
} mp4parse_cstatus_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
