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

#ifndef _H_ADTSPARSE_TYPES_H_
#define _H_ADTSPARSE_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>

/* needed for PCM defines */
#include "mp4parse_types.h"



/**
 * ADTSPARSE api error types
 */
typedef enum
{
    ADTSPARSE_ERR_NONE                  =  0,
    ADTSPARSE_ERR_UNKNOWN               = -1,
    ADTSPARSE_ERR_NEED_MORE_BITS        = -2,
    ADTSPARSE_BITSTREAM_FAILURE         = -3,
    ADTSPARSE_ERR_OUT_OF_MEMORY         = -4,
    ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW  = -5,
    ADTSPARSE_ERR_END_OF_STREAM         = -6
} ADTSPARSE_ERR;


/**
 *
 * PCM INFO:
 * expanded structure for pcm details, this is used only for in/out API,
 * but not for each buffer descriptor.
 *
 */
typedef mp4_pcm_info_t adts_pcm_info_t;

typedef mp4_track_info_t adts_track_info_t;

/**
 * ADTS headers, see ISO/IEC 13818-7 section 6.2.1 / 6.2.2
 */
typedef struct adts_fixed_hdr_
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

typedef struct adts_variable_hdr_
{
    unsigned copyright_identification_bit    : 1;
    unsigned copyright_identification_start  : 1;
    unsigned aac_frame_length                : 13;
    unsigned adts_buffer_fullness            : 11;
    unsigned num_raw_data_blocks_in_frame    : 2;
} adts_variable_hdr_t;


typedef struct adts_hrd_
{
    uint8_t             is_valid;
    adts_fixed_hdr_t    fixed;
    adts_variable_hdr_t variable;
} adts_hdr_t;

typedef enum
{
    ADTS_HDR_AAC_MAIN = 0,     /* AAC Main               */
    ADTS_HDR_AAC_LC,           /* Low Complexity         */
    ADTS_HDR_AAC_SSR,          /* Scalable Sampling Rate */
    ADTS_HDR_AAC_LTP,          /* RESERVED for MPEG-2,
                                * Long Term Prediction for MPEG-4 */
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

#define ADTS_HDR_ID_MPEG2 ( 1 )
#define ADTS_HDR_ID_MPEG4 ( 0 )

/**
 * *****************************************************************
 * Component TYPE & STATUS definition
 * *****************************************************************
 */

/* control register status flags */
#define ADTSPARSE_CTRL_ONOFF_STATUS    ( 1 << 0 )
#define ADTSPARSE_CTRL_DSP_STATUS      ( 1 << 2 )


/* inner components watermarking
 * definitions are GLOBALS to force uniqueness of the WMARKs
 */
#define ADTSPARSE_WMARK       ( (uint32_t) ( 0xefb0bb13 ) )

/**
 * component type (ctype)
 */
typedef enum
{
    ADTSPARSE_CTYPE_UNKNOWN       = 0,
    ADTSPARSE_CTYPE_ANYTYPE       = 1,
    ADTSPARSE_CTYPE_RESERVED      = 2,
    ADTSPARSE_CTYPE_SINGLE_THREAD = 3,
    ADTSPARSE_CTYPE_MULTI_THREAD  = 4,
    /**/
    ADTSPARSE_CTYPE_MAX
} adtsparse_ctype_t;


/**
 * component status (cstatus)
 */
typedef enum
{
    ADTSPARSE_CSTATUS_UNKNOWN  = 0,
    ADTSPARSE_CSTATUS_ANYTYPE  = 1,
    ADTSPARSE_CSTATUS_RESERVED = 2,
    ADTSPARSE_CSTATUS_CREATED  = 3,
    ADTSPARSE_CSTATUS_READY    = 4,
    ADTSPARSE_CSTATUS_RUNNED   = 5,
    ADTSPARSE_CSTATUS_STOPPED  = 6,
    ADTSPARSE_CSTATUS_FLUSHED  = 7,
    /**/
    ADTSPARSE_CSTATUS_MAX
} adtsparse_cstatus_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
