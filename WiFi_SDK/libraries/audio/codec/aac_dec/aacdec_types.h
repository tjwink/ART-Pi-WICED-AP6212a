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

#ifndef _H_AACDEC_TYPES_H_
#define _H_AACDEC_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>


/**
 * \brief  AAC decoder error codes.
 */
typedef enum
{
    AACDEC_BUFFER_TYPE_UNKNOWN = 0,
    AACDEC_BUFFER_TYPE_DATA,        /* audio stream data */
    AACDEC_BUFFER_TYPE_CMD_EOS,     /* and of stream     */
    AACDEC_BUFFER_TYPE_MAX
} AACDEC_BUFFER_TYPE;

/**
 * \brief  AAC decoder error codes.
 */
typedef enum
{
    AACDEC_ERR_NONE                 =  0,
    AACDEC_ERR_UNKNOWN              = -1,
    AACDEC_ERR_NEED_MORE_BITS       = -2,
    AACDEC_BITSTREAM_FAILURE        = -3,
    AACDEC_ERR_OUT_OF_MEMORY        = -4,
    AACDEC_ERR_INPUT_QUEUE_OVERFLOW = -5,
    AACDEC_ERR_END_OF_STREAM      = -6
} AACDEC_ERR;


/**
 *  STREAM TYPES
 */
typedef enum
{
    AACDEC_STREAM_UNKNOWN  = 0, /*search*/
    AACDEC_STREAM_M4A      = 1,
    AACDEC_STREAM_ADTS     = 2,
    AACDEC_STREAM_LATM     = 3,
    AACDEC_STREAM_ADIF     = 4,
    AACDEC_STREAM_RAW      = 5
} aacdec_stream_t;


/**
 *  AACDEC config paramters
 */
typedef enum
{
    AACDEC_PARAM_UNKNOWN   = 0,           /* unknown parameter                      */
    AACDEC_PARAM_ASC       = 1,           /* ASC:  mp4 syntax audio specific config */
    AACDEC_PARAM_STBL      = 2,           /* STBL: mp4 syntax sample table          */
    AACDEC_PARAM_STCO      = 3,           /* STCO: mp4 syntax chunk offset table    */
    AACDEC_PARAM_STSC      = 4,           /* STSC: mp4 syntax sample to chunk table */
    AACDEC_PARAM_TPCFG     = 5,           /* TPCFG: thread profiling config (tracex)*/

    /*
     * place ext params here
     */

    AACDEC_PARAM_START_OFFSET       = 10,   /* Playback starting offset in ms         */
    AACDEC_PARAM_STREAM_POSITION    = 11,   /* Set current stream position in bytes   */

    AACDEC_PARAM_MAX       = 0xFFFFFFFF,  /* uint32_t max param */
} aacdec_param_t;


/*
 *  There are only 8 "standard" valid channel maps for AAC.
 *  we keep the same channel map definition as of Apollo spec since it is a superset.
 *
 *  validation of the correct channel map is done in the code of aacdec_new() call.
 *
 *  FDK channel map is dependent on two vectors from CStreamInfo: AudioChannelTypes and ChannelIndices
 *
 *  0: Defined in AOT Specifc Config
 *  1: 1 channel:  front-center
 *  2: 2 channels: front-left, front-right
 *  3: 3 channels: front-center, front-left, front-right
 *  4: 4 channels: front-center, front-left, front-right, back-center
 *  5: 5 channels: front-center, front-left, front-right, back-left, back-right
 *  6: 6 channels: front-center, front-left, front-right, back-left, back-right, LFE-channel
 *  7: 8 channels: front-center, front-left, front-right, side-left, side-right, back-left, back-right, LFE-channel
 *  8-15: Reserved
 */
typedef uint32_t aacdec_chmap_t;

enum
{
    AACDEC_CHANNEL_MAP_NONE  = 0,           /* None or undefined     */
    AACDEC_CHANNEL_MAP_FL    = ( 1 << 0 ),  /* Front Left            */
    AACDEC_CHANNEL_MAP_FR    = ( 1 << 1 ),  /* Front Right           */
    AACDEC_CHANNEL_MAP_FC    = ( 1 << 2 ),  /* Front Center          */
    AACDEC_CHANNEL_MAP_LFE1  = ( 1 << 3 ),  /* LFE-1                 */
    AACDEC_CHANNEL_MAP_BL    = ( 1 << 4 ),  /* Back Left             */
    AACDEC_CHANNEL_MAP_BR    = ( 1 << 5 ),  /* Back Right            */
    AACDEC_CHANNEL_MAP_FLC   = ( 1 << 6 ),  /* Front Left Center     */
    AACDEC_CHANNEL_MAP_FRC   = ( 1 << 7 ),  /* Front Right Center    */
    AACDEC_CHANNEL_MAP_BC    = ( 1 << 8 ),  /* Back Center           */
    AACDEC_CHANNEL_MAP_LFE2  = ( 1 << 9 ),  /* LFE-2                 */
    AACDEC_CHANNEL_MAP_SIL   = ( 1 << 10 ), /* Side Left             */
    AACDEC_CHANNEL_MAP_SIR   = ( 1 << 11 ), /* Side Right            */
    AACDEC_CHANNEL_MAP_TPFL  = ( 1 << 12 ), /* Top Front Left        */
    AACDEC_CHANNEL_MAP_TPFR  = ( 1 << 13 ), /* Top Front Right       */
    AACDEC_CHANNEL_MAP_TPFC  = ( 1 << 14 ), /* Top Front Center      */
    AACDEC_CHANNEL_MAP_TPC   = ( 1 << 15 ), /* Top Center            */
    AACDEC_CHANNEL_MAP_TPBL  = ( 1 << 16 ), /* Top Back Left         */
    AACDEC_CHANNEL_MAP_TPBR  = ( 1 << 17 ), /* Top Back Right        */
    AACDEC_CHANNEL_MAP_TPSIL = ( 1 << 18 ), /* Top Side Left         */
    AACDEC_CHANNEL_MAP_TPSIR = ( 1 << 19 ), /* Top Side Right        */
    AACDEC_CHANNEL_MAP_TPBC  = ( 1 << 20 ), /* Top Back Center       */
    AACDEC_CHANNEL_MAP_BTFC  = ( 1 << 21 ), /* Bottom Front Center   */
    AACDEC_CHANNEL_MAP_BTFL  = ( 1 << 22 ), /* Bottom Front Left     */
    AACDEC_CHANNEL_MAP_BTFR  = ( 1 << 23 ), /* Bottom Front Right    */
    AACDEC_CHANNEL_MAP_TPLS  = ( 1 << 24 ), /* Top Left Surround     */
    AACDEC_CHANNEL_MAP_TPRS  = ( 1 << 25 ), /* Top Right Surround    */
    AACDEC_CHANNEL_MAP_LS    = ( 1 << 26 ), /* Middle Left Surround  */
    AACDEC_CHANNEL_MAP_RS    = ( 1 << 27 ), /* Middle Right Surround */
    AACDEC_CHANNEL_MAP_BLC   = ( 1 << 28 ), /* Back Left Center      */
    AACDEC_CHANNEL_MAP_BRC   = ( 1 << 29 ), /* Back Rigth Center     */
    AACDEC_CHANNEL_ZERO_030  = ( 1 << 30 ), /* reserved: zero bit    */
    AACDEC_CHANNEL_VIRTUAL   = ( 1 << 31 ), /* virtual chhannel      */
    /* aliases */
    AACDEC_CHANNEL_MAP_L     = AACDEC_CHANNEL_MAP_FL,
    AACDEC_CHANNEL_MAP_MONO  = AACDEC_CHANNEL_MAP_FL,
    AACDEC_CHANNEL_MAP_R     = AACDEC_CHANNEL_MAP_FR,
    AACDEC_CHANNEL_MAP_C     = AACDEC_CHANNEL_MAP_FC,
    AACDEC_CHANNEL_MAP_LFE   = AACDEC_CHANNEL_MAP_LFE1,
    AACDEC_CHANNEL_MAP_MAX   = AACDEC_CHANNEL_MAP_BRC
};

/**
 *
 * PCM INFO:
 * expanded structure for pcm details, this is used only for in/out API,
 * but not for each buffer descriptor.
 *
 */
typedef struct aacdec_pcm_info_
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
} aacdec_pcm_info_t;


/**
 * AAC/M4A informations on audio track
 * iso/iec 14496-12 8.4.2.1
 */
typedef struct aacdec_track_info_
{
    uint64_t creation_time;
    uint64_t modification_time;
    uint32_t timescale;
    uint64_t duration;
} aacdec_track_info_t;


/**
 * Callback for the start playback from offset.
 * Return value of zero signifies that a range request for the specified
 * byte position in the stream will be issued. All buffers received prior
 * to the impending flush should be discarded. All buffers received after
 * the flush will be based upon the offset within the file.
 */
typedef int (*aacdec_start_offset_cb_t)(void* aacdec, void* player, uint32_t start_offset_ms, uint32_t start_offset_bytes);


/**
 * *****************************************************************
 * FDK-AAC interface
 * *****************************************************************
 */

typedef void* AACDEC_HANDLE;

/**
 * *****************************************************************
 * Component TYPE & STATUS definition
 * *****************************************************************
 */

/* control register status flags */
#define AACDEC_CTRL_ONOFF_STATUS    ( 1 << 0 )
#define AACDEC_CTRL_INPUT_STATUS    ( 1 << 1 )
#define AACDEC_CTRL_OUTPUT_STATUS   ( 1 << 4 )


/* inner components watermarking
 * definitions are GLOBALS to force uniqueness of the WMARKs
 */
#define AACDEC_WMARK       ( (uint32_t) ( 0xefb07711 ) )

/**
 * component type (ctype)
 */
typedef enum
{
    AACDEC_CTYPE_UNKNOWN       = 0,
    AACDEC_CTYPE_ANYTYPE       = 1,
    AACDEC_CTYPE_RESERVED      = 2,
    AACDEC_CTYPE_SINGLE_THREAD = 3,
    AACDEC_CTYPE_MULTI_THREAD  = 4,
    /**/
    AACDEC_CTYPE_MAX
} aacdec_ctype_t;


/**
 * component status (cstatus)
 */
typedef enum
{
    AACDEC_CSTATUS_UNKNOWN  = 0,
    AACDEC_CSTATUS_ANYTYPE  = 1,
    AACDEC_CSTATUS_RESERVED = 2,
    AACDEC_CSTATUS_CREATED  = 3,
    AACDEC_CSTATUS_READY    = 4,
    AACDEC_CSTATUS_RUNNED   = 5,
    AACDEC_CSTATUS_STOPPED  = 6,
    AACDEC_CSTATUS_FLUSHED  = 7,
    /**/
    AACDEC_CSTATUS_MAX
} aacdec_cstatus_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
