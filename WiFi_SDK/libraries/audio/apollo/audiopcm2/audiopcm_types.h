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
 * @file audiopcm_types.h
 *
 * Collection of exported defintions and component type
 *
 */
#ifndef _H_AUDIOPCM_TYPES_H_
#define _H_AUDIOPCM_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>



/**
 * *****************************************************************
 * audiopcm DEFAULTS
 * *****************************************************************
 */

#define MAX_DEMUX_AUDIO_CHANNEL_NUM ( 2 )
#define MAX_DEMUX_AUDIO_CHANNEL_LIST_SIZE ( 32 )


/* standard MTU, no jumbo frames    */
/* MTU size must be a multiple of 2 */
#define RTP_PKT_MTU_SIZE  ( (uint32_t) ( 1500 ) )

/* DSP buffer format */
#define DSP_MAX_CH_NUM    ( MAX_DEMUX_AUDIO_CHANNEL_NUM )
#define DSP_MAX_BUF_DATA  ( RTP_PKT_MTU_SIZE )
#define DSP_CH0           ( 0 )
#define DSP_CH1           ( 1 )

/* CONCEALMENT support */
#define CONCEALMENT_NONE  ( (uint8_t) 0 )
#define CONCEALMENT_PLC   ( ( (uint8_t) 1 ) << 0 )
#define CONCEALMENT_SLC   ( ( (uint8_t) 1 ) << 1 )
#define CONCEALMENT_ASLC  ( ( (uint8_t) 1 ) << 2 )
#define CONCEALMENT_ALL   ( CONCEALMENT_PLC | CONCEALMENT_SLC )

/* DEFAULT paramters for audiopcm component */
#define APCM_DEFAULT_BL_MAX                  ( 36 )

/* DEFAULT paramters for audiopcm latency&concealment */
#define APCM_DEFAULT_TARGET_AUDIO_LATENCY_MS ( 50 )
#define APCM_DEFAULT_TARGET_JITTER_MS        ( 10 )
#define APCM_DEFAULT_TARGET_RTT_MS           ( 6 )
#define APCM_DEFAULT_CONCEALMENT             ( CONCEALMENT_ALL )
#define APCM_DEFAULT_DMX                     ( MAX_DEMUX_AUDIO_CHANNEL_NUM )
#define APCM_DEFAULT_LOG_LVL                 ( 0 )



/**
 * *****************************************************************
 * audiopcm RTP_PKT_TYPE
 * *****************************************************************
 */

/* Apollo PKT definition */
#define RTP_PKT_PAYLOAD_TYPE_PCM_AUDIO ( (uint8_t) 98 )
#define RTP_PKT_PAYLOAD_TYPE_CPY_AUDIO ( (uint8_t) 99 )
#define RTP_PKT_PAYLOAD_TYPE_FEC_AUDIO ( (uint8_t) 100 )
#define RTP_PKT_PAYLOAD_TYPE_USR_CHMAP ( (uint8_t) 101 )



/**
 * *****************************************************************
 * audiopcm CHANNEL LIST
 * *****************************************************************
 */

/*
 * This is the definition of supported channel type from Apollo specs.
 * This is used to define which channel the audiopcm should extract
 * from the multi channel payload PCM format.
 */
enum
{
    APCM_CHANNEL_MAP_NONE  = 0,           /* None or undefined     */
    APCM_CHANNEL_MAP_FL    = ( 1 << 0 ),  /* Front Left            */
    APCM_CHANNEL_MAP_FR    = ( 1 << 1 ),  /* Front Right           */
    APCM_CHANNEL_MAP_FC    = ( 1 << 2 ),  /* Front Center          */
    APCM_CHANNEL_MAP_LFE1  = ( 1 << 3 ),  /* LFE-1                 */
    APCM_CHANNEL_MAP_BL    = ( 1 << 4 ),  /* Back Left             */
    APCM_CHANNEL_MAP_BR    = ( 1 << 5 ),  /* Back Right            */
    APCM_CHANNEL_MAP_FLC   = ( 1 << 6 ),  /* Front Left Center     */
    APCM_CHANNEL_MAP_FRC   = ( 1 << 7 ),  /* Front Right Center    */
    APCM_CHANNEL_MAP_BC    = ( 1 << 8 ),  /* Back Center           */
    APCM_CHANNEL_MAP_LFE2  = ( 1 << 9 ),  /* LFE-2                 */
    APCM_CHANNEL_MAP_SIL   = ( 1 << 10 ), /* Side Left             */
    APCM_CHANNEL_MAP_SIR   = ( 1 << 11 ), /* Side Right            */
    APCM_CHANNEL_MAP_TPFL  = ( 1 << 12 ), /* Top Front Left        */
    APCM_CHANNEL_MAP_TPFR  = ( 1 << 13 ), /* Top Front Right       */
    APCM_CHANNEL_MAP_TPFC  = ( 1 << 14 ), /* Top Front Center      */
    APCM_CHANNEL_MAP_TPC   = ( 1 << 15 ), /* Top Center            */
    APCM_CHANNEL_MAP_TPBL  = ( 1 << 16 ), /* Top Back Left         */
    APCM_CHANNEL_MAP_TPBR  = ( 1 << 17 ), /* Top Back Right        */
    APCM_CHANNEL_MAP_TPSIL = ( 1 << 18 ), /* Top Side Left         */
    APCM_CHANNEL_MAP_TPSIR = ( 1 << 19 ), /* Top Side Right        */
    APCM_CHANNEL_MAP_TPBC  = ( 1 << 20 ), /* Top Back Center       */
    APCM_CHANNEL_MAP_BTFC  = ( 1 << 21 ), /* Bottom Front Center   */
    APCM_CHANNEL_MAP_BTFL  = ( 1 << 22 ), /* Bottom Front Left     */
    APCM_CHANNEL_MAP_BTFR  = ( 1 << 23 ), /* Bottom Front Right    */
    APCM_CHANNEL_MAP_TPLS  = ( 1 << 24 ), /* Top Left Surround     */
    APCM_CHANNEL_MAP_TPRS  = ( 1 << 25 ), /* Top Right Surround    */
    APCM_CHANNEL_MAP_LS    = ( 1 << 26 ), /* Middle Left Surround  */
    APCM_CHANNEL_MAP_RS    = ( 1 << 27 ), /* Middle Right Surround */
    APCM_CHANNEL_MAP_BLC   = ( 1 << 28 ), /* Back Left Center      */
    APCM_CHANNEL_MAP_BRC   = ( 1 << 29 ), /* Back Rigth Center     */
    APCM_CHANNEL_ZERO_030  = ( 1 << 30 ), /* reserved: zero bit    */
    APCM_CHANNEL_VIRTUAL   = ( 1 << 31 ), /* virtual chhannel      */
    /* aliases */
    APCM_CHANNEL_MAP_L     = APCM_CHANNEL_MAP_FL,
    APCM_CHANNEL_MAP_MONO  = APCM_CHANNEL_MAP_FL,
    APCM_CHANNEL_MAP_R     = APCM_CHANNEL_MAP_FR,
    APCM_CHANNEL_MAP_C     = APCM_CHANNEL_MAP_FC,
    APCM_CHANNEL_MAP_LFE   = APCM_CHANNEL_MAP_LFE1,
    APCM_CHANNEL_MAP_MAX   = APCM_CHANNEL_MAP_BRC
};



/**
 *
 * audiopcm PCM INFO:
 * expanded structure for pcm details, this is used only for in/out API,
 * but not for each buffer descriptor. For the internal buffer we use the
 * compressed pcm_fmt_t to save memory (2 bytes)
 *
 */
typedef struct pcm_info_
{
    unsigned bps : 5;           /* bps:  16 or 24                           */
    unsigned cbps : 6;          /* cbps: equal to bps (16/24) or 32         */
    unsigned endian : 1;        /* endianness: 0=little 1=big               */
    unsigned sign : 1;          /* signedness: 0=unsigned 1=signed          */
    unsigned interleave : 1;    /* interleaving: 0=coalesced 1=interleaved  */
    unsigned floating : 1;      /* floating: 0=integer 1=floating           */
    unsigned pcm_fmt_usr : 1;   /* pcm_format overwrite flag: 1=overwrite   */
    unsigned chmap : 6;         /* chmap: 0-63 from the Apollo CHMAP table  */
    uint32_t sr;                /* sample rate: 44100, 48000, 96000, 192000 */
} pcm_info_t;


/**
 * audiopcm AUDIO INFO:
 * expanded structure for audio details (apollo syntax plus pcm details)
 * this is used only for in/out API, but not for each buffer. For internal buffers
 * we use the audio_descriptor_t which is a superset. audio_descriptor does refer
 * to the pcm_fmt_t
 */
typedef struct audio_info_
{
    /* Apollo rtp timing */
    uint16_t   rtp_sn;
    uint16_t   rtp_rsn;
    uint32_t   rtp_pts;

    /* Apollo spec timing */
    uint64_t   sys_clock;
    uint64_t   media_clock;
    uint8_t    media_clock_unit;
    /* media clock valid flag, 1:valid, 0:not valid */
    uint8_t    media_clock_valid_f;

    /* audio compression type */
    uint8_t    audio_compression_type;

    /* Apollo spec versioning */
    uint8_t    ver_mjr;
    uint8_t    ver_min;

    /* Apollo audio format */
    pcm_info_t src_pcm_info;

    /* auxiliary info */
    uint16_t   audio_frames_num;
} audio_info_t;


/**
 * *****************************************************************
 * audiopcm PCM format type
 * *****************************************************************
 */

/* PCM FORMAT: BPS, bit per sample */
#define PCM_FMT_BPS_SFT         ( 15 )
#define PCM_FMT_BPS_MSK         ( ( (uint16_t) 1 ) << PCM_FMT_BPS_SFT )
#define PCM_FMT_BPS_16          ( ( (uint16_t) 0 ) << PCM_FMT_BPS_SFT )
#define PCM_FMT_BPS_24          ( ( (uint16_t) 1 ) << PCM_FMT_BPS_SFT )

/* PCM FORMAT: CBPS, container bit per sample */
#define PCM_FMT_CBPS_SFT        ( 14 )
#define PCM_FMT_CBPS_MSK        ( ( (uint16_t) 1 ) << PCM_FMT_CBPS_SFT )
#define PCM_FMT_CBPS_BPS        ( ( (uint16_t) 0 ) << PCM_FMT_CBPS_SFT )
#define PCM_FMT_CBPS_32         ( ( (uint16_t) 1 ) << PCM_FMT_CBPS_SFT )

/* PCM FORMAT: endianness */
#define PCM_FMT_ENDIAN_SFT      ( 13 )
#define PCM_FMT_ENDIAN_MSK      ( ( (uint16_t) 1 ) << PCM_FMT_ENDIAN_SFT )
#define PCM_FMT_ENDIAN_L        ( ( (uint16_t) 0 ) << PCM_FMT_ENDIAN_SFT )
#define PCM_FMT_ENDIAN_B        ( ( (uint16_t) 1 ) << PCM_FMT_ENDIAN_SFT )

/* PCM FORMAT: signedness */
#define PCM_FMT_SIGN_SFT        ( 12 )
#define PCM_FMT_SIGN_MSK        ( ( (uint16_t) 1 ) << PCM_FMT_SIGN_SFT )
#define PCM_FMT_SIGN_U          ( ( (uint16_t) 0 ) << PCM_FMT_SIGN_SFT )
#define PCM_FMT_SIGN_S          ( ( (uint16_t) 1 ) << PCM_FMT_SIGN_SFT )

/* PCM FORMAT: interleaving */
#define PCM_FMT_INTERL_SFT      ( 11 )
#define PCM_FMT_INTERL_MSK      ( ( (uint16_t) 1 ) << PCM_FMT_INTERL_SFT )
#define PCM_FMT_INTERL_C        ( ( (uint16_t) 0 ) << PCM_FMT_INTERL_SFT )
#define PCM_FMT_INTERL_I        ( ( (uint16_t) 1 ) << PCM_FMT_INTERL_SFT )

/* PCM FORMAT: integer/floating */
#define PCM_FMT_FLOAT_SFT       ( 10 )
#define PCM_FMT_FLOAT_MSK       ( ( (uint16_t) 1 ) << PCM_FMT_FLOAT_SFT )
#define PCM_FMT_FLOAT_I         ( ( (uint16_t) 0 ) << PCM_FMT_FLOAT_SFT )
#define PCM_FMT_FLOAT_F         ( ( (uint16_t) 1 ) << PCM_FMT_FLOAT_SFT )

/* PCM FORMAT: SR, sample rate, hertz */
#define PCM_FMT_SR_SFT          ( 8 )
#define PCM_FMT_SR_MSK          ( ( (uint16_t) 3 ) << PCM_FMT_SR_SFT )
#define PCM_FMT_SR_44100        ( ( (uint16_t) 0 ) << PCM_FMT_SR_SFT )
#define PCM_FMT_SR_48000        ( ( (uint16_t) 1 ) << PCM_FMT_SR_SFT )
#define PCM_FMT_SR_96000        ( ( (uint16_t) 2 ) << PCM_FMT_SR_SFT )
#define PCM_FMT_SR_192000       ( ( (uint16_t) 3 ) << PCM_FMT_SR_SFT )

/* PCM FORMAT: CHMAPUSR */
#define PCM_FMT_CHMAP_USR_SFT   ( 7 )
#define PCM_FMT_CHMAP_USR_MSK   ( ( (uint16_t) 0x01 ) << PCM_FMT_CHMAP_USR_SFT )

/* PCM FORMAT: CHMAP */
#define PCM_FMT_CHMAP_SFT       ( 1 )
#define PCM_FMT_CHMAP_MSK       ( ( (uint16_t) 0x3F ) << PCM_FMT_CHMAP_SFT )
#define PCM_FMT_CHMAP_MONO      ( 0 )
#define PCM_FMT_CHMAP_STEREO    ( 1 )

/* fmt_table matching */
#define PCM_FMT_ANY             ( 254 )
#define PCM_FMT_UKNOWN          ( 255 )

#define PCM_FMT_ID_INVALID      ( 0 )
#define PCM_FMT_ID_UNKNOWN      ( PCM_FMT_UKNOWN )

/* pcm format codec */
#define PCM_FMT_CODEC_PCM16     ( 16 )
#define PCM_FMT_CODEC_PCM32     ( 32 )

/* pcm format don't care mask */
#define PCM_FMT_DNTCARE_MSK     ( ~( (uint16_t) ( 0x01 ) ) )

/* speedup macro */
#define PCM_FMT_GET( y, x, field )                                                              \
    do                                                                                          \
    {                                                                                           \
        ( y ) = ( ( ( x ) & ( PCM_FMT_ ## field ## _MSK ) ) >> ( PCM_FMT_ ## field ## _SFT ) ); \
    }                                                                                           \
    while ( 0 );

/* AUDIO COMPRESSION TYPE */
enum
{
    APCM_ACT_NONE      = 0,
    APCM_ACT_BASELINE  = 1,
    APCM_ACT_RESERVED1 = 2,
    APCM_ACT_RESERVED2 = 3
};

/**
 * *****************************************************************
 * audiopcm BUFFER definition (RTP, DSP, OUT and DESCRIPTOR buffers)
 * *****************************************************************
 */


/**
 *
 * audiopcm PCM format
 *
 */
typedef struct pcm_fmt_
{
    /* id, unique value from table_fmt_type which validate the fmt format itself */
    uint8_t  id;

    /* Apollo PCM format bitstring as per specification 2.0 */
    uint16_t bits;

    /*
     * bps:        1bit, bit 15, MSB
     * cbps:       1bit, bit 14
     * endian:     1bit, bit 13
     * sign:       1bit, bit 12
     * interleave: 1bit, bit 11
     * float:      1bit, bit 10
     * sr:         2bit, bit 8
     * pcm_fmt_usr:1bit, bit 7
     * chmap:      6bit, bit 1
     * O:          1bit, bit 0, LSB
     */
} pcm_fmt_t;



/**
 * audiopcm buffer info
 * this is the audio info structure attached to each audio packet (audio_pkt_fifo)
 */
#define AUDIO_DESCRIPTOR_HINTS_NULL                       ( 0 )
#define AUDIO_DESCRIPTOR_HINTS_DATA_VALID   ( ( (uint8_t) 1 ) << 0 )
#define AUDIO_DESCRIPTOR_HINTS_DATA_ERROR   ( ( (uint8_t) 1 ) << 1 )
typedef struct audio_descriptor_
{
    uint8_t   hints; /* stage/hints flags     */

    /* Apollo rtp timing */
    uint16_t  rtp_sn;
    uint16_t  rtp_rsn;
    uint32_t  rtp_pts;
    uint16_t  rtp_extension_length;

    /* Apollo spec timing */
    uint64_t  sys_clock;
    uint64_t  media_clock;
    uint8_t   media_clock_unit;
    uint8_t   algebraic_compression_type;
    /* media clock valid flag, 1:valid, 0:not valid */
    uint8_t   media_clock_valid_f;

    /* audio compression type */
    uint8_t   audio_compression_type;

    /* Apollo spec versioning */
    uint8_t   ver_mjr;
    uint8_t   ver_min;

    /* Apollo audio format */
    pcm_fmt_t src_pcm_fmt;

    uint8_t   usr_pcm_fmt_f; /* pcm format override */

    uint8_t   bl_sync;       /* burst length sync */
    uint8_t   bl;            /* burst length value */
    uint8_t   sl;            /* shuffle length value */

    /* auxiliary info */
    /* one audio frame is composed by one sample for each channel */
    /* audio_frame = audio_payload/(cbps*channel_cnt)             */
    uint16_t  audio_frames_num;
    uint16_t  audio_frames_bytes;
    uint16_t  audio_target_latency;

    /* this is needed to properly handle the case                   */
    /* where BL==0 and there should not be FEC or CPY incoming data */
    uint8_t   bl_hdr; /* burst length in the RTP header */
} audio_descriptor_t;


/**
 * RTP buffer structure for audio input RTP fifo
 * this is the structure for buffers of the audio_rtp_fifo,
 * which is also the main input queue of the audiopcm pipeline.
 */
#define RTP_HINTS_NULL                        ( 0 )
#define RTP_HINTS_DATA_VALID    ( ( (uint8_t) 1 ) << 0 )
#define RTP_HINTS_HEADER_VALID  ( ( (uint8_t) 1 ) << 1 )
#define RTP_HINTS_CLKS_UNKNOWN  ( ( (uint8_t) 1 ) << 2 )
#define RTP_HINTS_DATA_FROM_FEC ( ( (uint8_t) 1 ) << 3 )
#define RTP_HINTS_CPY_PKT       ( ( (uint8_t) 1 ) << 4 )
#define RTP_HINTS_FEC_PKT       ( ( (uint8_t) 1 ) << 5 )
#define RTP_HINTS_PKT_LISTED    ( ( (uint8_t) 1 ) << 6 )
#define RTP_HINTS_DATA_ERROR    ( ( (uint8_t) 1 ) << 7 )

typedef struct rtp_buf_
{
    uint8_t            hints;            /* stage/hints flags     */

    audio_descriptor_t audio_descriptor; /* source audio info */

    uint16_t           sn;               /* rtp sequence number   */
    uint8_t            payload_type;
    uint32_t           ssrc;             /* syncronization source */
    uint16_t           extension_id;
    uint16_t           extension_length;
    uint32_t           length;       /* data length           */

    /* maximum RTP payload is fixed as per RTP specs          */
    uint8_t            data[ RTP_PKT_MTU_SIZE ]; /* payload data          */

    /* pointer to raw pcm data */
    uint8_t*           data_pcm;

    /* internals */
    uint32_t           token_id;     /* read only for ZC, do not touch */
    uint32_t           fifo_idx;
} rtp_buf_t;



#define DSP_HINTS_NULL                        ( 0 )
#define DSP_HINTS_DATA_VALID    ( ( (uint8_t) 1 ) << 0 )
#define DSP_HINTS_DATA_LOST     ( ( (uint8_t) 1 ) << 1 )
#define DSP_HINTS_CLKS_UNKNOWN  ( ( (uint8_t) 1 ) << 2 )
#define DSP_HINTS_DATA_FROM_FEC ( ( (uint8_t) 1 ) << 3 )
#define DSP_HINTS_RESERVED_04   ( ( (uint8_t) 1 ) << 4 )
#define DSP_HINTS_RESERVED_05   ( ( (uint8_t) 1 ) << 5 )
#define DSP_HINTS_RESERVED_06   ( ( (uint8_t) 1 ) << 6 )
#define DSP_HINTS_DATA_ERROR    ( ( (uint8_t) 1 ) << 7 )

/**
 * DSP audio packet struct
 * this is the structure of the buffers into the audio_dsp_fifo
 */
typedef struct dsp_buf_
{
    uint8_t            hints;        /* stage/hints flags     */

    audio_descriptor_t audio_descriptor;

    // uint8_t refcnt;

    /* data_lo(pcm_clear) and data_hi(pcm_shuffled) */
    uint8_t*           data_lo[ DSP_MAX_CH_NUM ];
    uint8_t*           data_hi[ DSP_MAX_CH_NUM ];

    // uint32_t data_begin_idx;
    uint16_t           data_length_frames;
    uint32_t           data_length_bytes;

    /* internals */
    uint32_t           token_id;
    uint32_t           fifo_idx;
} dsp_buf_t;



#define OUT_HINTS_NULL                        ( 0 )
#define OUT_HINTS_DATA_VALID    ( ( (uint8_t) 1 ) << 0 )
#define OUT_HINTS_DATA_READY    ( ( (uint8_t) 1 ) << 1 )
#define OUT_HINTS_CLKS_UNKNOWN  ( ( (uint8_t) 1 ) << 2 )
#define OUT_HINTS_RESERVED_03   ( ( (uint8_t) 1 ) << 3 )
#define OUT_HINTS_RESERVED_04   ( ( (uint8_t) 1 ) << 4 )
#define OUT_HINTS_RESERVED_05   ( ( (uint8_t) 1 ) << 5 )
#define OUT_HINTS_RESERVED_06   ( ( (uint8_t) 1 ) << 6 )
#define OUT_HINTS_DATA_ERROR    ( ( (uint8_t) 1 ) << 7 )

/**
 * PCM audio packet struct
 * this is the structure of the buffers into the audio_dsp_fifo
 */
typedef struct output_buf_
{
    uint8_t            hints;        /* stage/hints flags     */

    /* output buf has both            */
    /* audio_descriptor (internal)    */
    /* and also audio_info (external) */
    audio_descriptor_t audio_descriptor;
    audio_info_t       audio_info;

    uint8_t*           data;

    uint16_t           data_length_frames;
    uint32_t           data_length_bytes;

    /* internals */
    uint32_t           dsp_buf_ref; // reference source dsp buffer
    uint32_t           fifo_idx;
} output_buf_t;


typedef struct audio_stats_
{
    /* Apollo spec versioning */
    // uint8_t   ver_mjr;
    // uint8_t   ver_min;

    /* counters for reports */
    uint64_t concealment_slcplc_cnt;
    uint64_t concealment_fec_cnt;
} audiopcm_stats_t;

/**
 * *****************************************************************
 * Component TYPE & STATUS definition
 * *****************************************************************
 */

/**
 * component type (ctype)
 */
typedef enum
{
    AUDIOPCM_CTYPE_UNKNOWN       = 0,
    AUDIOPCM_CTYPE_ANYTYPE       = 1,
    AUDIOPCM_CTYPE_RESERVED      = 2,
    AUDIOPCM_CTYPE_SINGLE_THREAD = 3,
    AUDIOPCM_CTYPE_MULTI_THREAD  = 4,
    AUDIOPCM_CTYPE_MAX
} audiopcm_ctype_t;


/**
 * component status (cstatus)
 */
typedef enum
{
    AUDIOPCM_CSTATUS_UNKNOWN  = 0,
    AUDIOPCM_CSTATUS_ANYTYPE  = 1,
    AUDIOPCM_CSTATUS_RESERVED = 2,
    AUDIOPCM_CSTATUS_CREATED  = 3,
    AUDIOPCM_CSTATUS_READY    = 4,
    AUDIOPCM_CSTATUS_RUNNED   = 5,
    AUDIOPCM_CSTATUS_STOPPED  = 6,
    AUDIOPCM_CSTATUS_FLUSHED  = 7,
    /**/
    AUDIOPCM_CSTATUS_MAX
} audiopcm_cstatus_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_AUDIOPCM_TYPES_H_ */
