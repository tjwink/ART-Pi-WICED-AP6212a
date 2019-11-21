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

#pragma once

#ifndef  __LIBFLAC_INTERFACE_H__
#define  __LIBFLAC_INTERFACE_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"
#include "platform_audio.h"
#include "audio_render.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

//#define FLAC_PACKET_SIZE                (8*1024)    /* this is the same as the FLAC decoder requet buffer size */

#define APP_QUEUE_MAX_ENTRIES               20

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum {
    FLAC_LIB_LOG_ERROR = 0,
    FLAC_LIB_LOG_INFO,
    FLAC_LIB_LOG_DEBUG,
} wiced_flac_log_level_t;


typedef enum {
    CHANNEL_MAP_NONE  = 0,          /* None or undefined    */
    CHANNEL_MAP_FL    = (1 << 0),   /* Front Left           */
    CHANNEL_MAP_FR    = (1 << 1),   /* Front Right          */
    CHANNEL_MAP_FC    = (1 << 2),   /* Front Center         */
    CHANNEL_MAP_LFE1  = (1 << 3),   /* LFE-1                */
    CHANNEL_MAP_BL    = (1 << 4),   /* Back Left            */
    CHANNEL_MAP_BR    = (1 << 5),   /* Back Right           */
    CHANNEL_MAP_FLC   = (1 << 6),   /* Front Left Center    */
    CHANNEL_MAP_FRC   = (1 << 7),   /* Front Right Center   */
    CHANNEL_MAP_BC    = (1 << 8),   /* Back Center          */
    CHANNEL_MAP_LFE2  = (1 << 9),   /* LFE-2                */
    CHANNEL_MAP_SIL   = (1 << 10),  /* Side Left            */
    CHANNEL_MAP_SIR   = (1 << 11),  /* Side Right           */
    CHANNEL_MAP_TPFL  = (1 << 12),  /* Top Front Left       */
    CHANNEL_MAP_TPFR  = (1 << 13),  /* Top Front Right      */
    CHANNEL_MAP_TPFC  = (1 << 14),  /* Top Front Center     */
    CHANNEL_MAP_TPC   = (1 << 15),  /* Top Center           */
    CHANNEL_MAP_TPBL  = (1 << 16),  /* Top Back Left        */
    CHANNEL_MAP_TPBR  = (1 << 17),  /* Top Back Right       */
    CHANNEL_MAP_TPSIL = (1 << 18),  /* Top Side Left        */
    CHANNEL_MAP_TPSIR = (1 << 19),  /* Top Side Right       */
    CHANNEL_MAP_TPBC  = (1 << 20),  /* Top Back Center      */
    CHANNEL_MAP_BTFC  = (1 << 21),  /* Bottom Front Center  */
    CHANNEL_MAP_BTFL  = (1 << 22),  /* Bottom Front Left    */
    CHANNEL_MAP_BTFR  = (1 << 23)   /* Bottom Front Right   */
} AUDIO_CHANNEL_MAP_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct wiced_flac_source_info_s {
        /* source info */
        uint64_t        source_current_sample;  /* last played sample                   */
        uint64_t        source_total_samples;   /* total samples in stream (if known)   */
        uint32_t        source_sample_rate;     /* sample rate                          */
        uint8_t         source_channels;        /* number of channels in source stream  */
        uint8_t         source_bps;             /* bits per sample in source stream     */
} wiced_flac_source_info_t;


typedef struct wiced_flac_buffer_info_s {
        /* filled by Application (what FLAC is expected to put in buffer)           */
        void*           data_buf;           /* pointer to the LPCM buffer           */
        uint32_t        data_buf_size;      /* size of buffer (in bytes)            */
        uint32_t        data_bps;           /* sample size (in bits)                */
        uint8_t         num_channels;       /* # channels in buffer                 */
        uint32_t        channel_mask;       /* channel mask                         */
        void*           opaque;             /* for app reference                    */

        /* filled by FLAC decoder */
        uint32_t        filled_data_offset;     /* filled offset from start of buffer   */
        uint32_t        filled_data_length;     /* filled length of data (in bytes)     */
        uint32_t        filled_channels;        /* filled channels in buffer            */
        uint64_t        filled_samples;         /* filled samples in the buffer         */
        uint64_t        filled_pts;             /* filled pts value                     */
        uint64_t        current_sample;         /* first sample in buffer, from start of stream */

        /* source info */
        uint64_t        source_total_samples;   /* total samples in stream (if known)   */
        uint32_t        source_sample_rate;     /* sample rate                          */
        uint8_t         source_channels;        /* number of channels in source stream  */
        uint8_t         source_bps;             /* bits per sample in source stream     */
        uint8_t         frame_size;             /* size of audio frame in bytes         */
} wiced_flac_buffer_info_t;


/** Callback functions for flac to get / send necessary buffers
 *
 */
typedef wiced_result_t (*wiced_flac_audio_buffer_get)(wiced_flac_buffer_info_t *buff_info);
typedef wiced_result_t (*wiced_flac_audio_buffer_push)(wiced_flac_buffer_info_t *buff_info);


/******************************************************
 *                    Structures
 ******************************************************/

/**
 * Parameters needed for the flac decoder
 *
 */
typedef struct wiced_flac_params_s {
        wiced_flac_audio_buffer_get     buff_get;   /* get a buffer to fill with LPCM and submit  */
        wiced_flac_audio_buffer_push    buff_push;  /* push a buffer with LPCM data in it to the audio render */
}wiced_flac_params_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
/******************************************************
 *               Variables Definitions
 ******************************************************/
/******************************************************
 *               Function Definitions
 ******************************************************/

/** Initialize the FLAC decoder
 *
 * @param player    :
 *
 * @return  sucess: pointer to internal structure used for subsequent calls
 *          fail:   NULL
 */
void* wiced_flac_decoder_init(wiced_flac_params_t* params);

/** De-Initialize the FLAC decoder
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_decoder_deinit(void* internal_ptr);

/** Start a stream-based FLAC audio decoder session
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_decoder_start(void* internal_ptr);

/** Submit a packet from the stream we are playing
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 * @param packet        : pointer to a wiced_packet with flac data
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_decoder_submit_packet(void* internal_ptr, wiced_packet_t* packet);

/** Stop a stream-based FLAC audio decoder session
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_decoder_stop(void* internal_ptr);

/** Get simple information about the currently playing source
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 * @param source_info   : pointer to structure to store information
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_get_source_info(void* internal_ptr, wiced_flac_source_info_t* source_info);

/** TODO: return metadata in a WICED format
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 * @param metadata      : ptr to store metadata (todo:)
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_get_metadata(void* internal_ptr, void *metadata);

/** Set log level
 *
 * @param internal_ptr  : pointer returned from wiced_flac_decoder_init()
 *
 * @param log_level         FLAC_LIB_LOG_OFF
 *                          FLAC_LIB_LOG_INFO
 *                          FLAC_LIB_LOG_DEBUG
 *
 * @return  WICED_SUCCESS
 *          WICED_BAD_ARG
 *          WICED_ERROR
 */
wiced_result_t wiced_flac_set_log_level(void* internal_ptr, wiced_flac_log_level_t log_level);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* __LIBFLAC_INTERFACE_H__    */
