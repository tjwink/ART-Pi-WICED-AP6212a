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
#ifndef _H_MP4PARSE_CORE_H_
#define _H_MP4PARSE_CORE_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "mp4parse_types.h"
#include "mp4parse.h"
#include "tsp.h"
#include "tpm.h"

#include "../fdk_aac_dec/libAACdec/include/aacdecoder_lib.h"


/**
 * descriptors queue:
 * MUST be power of two, min 256 buffers
 *
 * NOTE: decriptor queue size and raw queue size are related.
 *       considering a MAX_MTU of 1500 and considereing a VBR factor
 *       of 2 (or 3) the number of buffers in the decriptor queue should be
 *
 *       pow_2( VBR_factor * (raw_queue_size/MTU) )
 */
#define BUF_NUM         ( 1 << 8 )

/*
 * raw queue:
 * MUST be power of 2, 64k is the minimum since we can have
 * 2x32k meta into the AAC header before a moov box
 */
#define BUF_RAW_SIZE    ( (uint32_t) ( 1024 * ( 1 << 6 ) ) )

/* decode/output stereo buffer 8K */
#define BUF_8K          ( 8 * 2 * 1024 )


/**
 * input fifo buffer descriptors:
 * buffers descriptors are in a discrete queue
 * with references to the raw bytes queue for payloads
 * since the input stream is VBR we optimize buffer allocation
 * with local refs.
 */
typedef struct data_buf_
{
    volatile uint8_t  status;
    uint8_t           type;
    uint8_t*          raw;
    volatile uint32_t raw_idx;
    volatile uint32_t length;
    volatile uint64_t bitstream_idx;
} data_buf_t;



/**
 * audio component struct
 *
 * Note: compile optional fields (under #ifdef) MUST be at the very end,
 *       to avoid structure misalignement on binary exectution/debug
 *
 * Note: COMPONENT_TAG(), COMPONENT_SWVER(), COMPONENT_DBG() must be
 *       at the very top, and the order is FIXED.
 */

typedef struct mp4parse_
{
    /* COMPONENT_TAG() */
    uint32_t                 id;
    uint32_t                 wmark;
    uint32_t                 type;

    /* COMPONENT_SWVER() */
    uint8_t                  major;
    uint8_t                  minor;
    char                     revision[ 32 ];
    char                     build[ 5 ];

    /* COMPONENT_DBG() */
    uint8_t                  log_lvl;
    uint8_t                  log_err;
    char                     label[ 64 ];

    /* COMPONENT_ATTRIBUTES() */

    /**  simple CONTROL register. (8 bit flags to control the processing loop) */

    /**
     *- bit 0  : ON/OFF flag
     *- bit 1  : DSP thread flag
     *- bit 2  : reserved
     *- bit 3  : reserved
     *- bit 4  : reserved
     *- bit 5  : reserved
     *- bit 6  : reserved
     *- bit 7  : reserved
     *
     * Note: ctrl MUST be always the first byte of the struct after the COMPONENT_BASE_T()
     */

    uint8_t volatile         ctrl;
    tsp_mutex_t              ctrl_mtx;

    uint8_t                  userid;

    /* COMPONENT_STATUS() */
    mp4parse_cstatus_t       status;


    /* UPPER component */
    void*                    decoder;

    /* INNER parse&decoding thread */
    tsp_thread_t             mp4_thread;
    tsp_attr_t               mp4_thread_attr;
    tsp_semaphore_t          mp4_thread_sem;

    /*
     * INPUT fifo buffer descriptors
     */
    data_buf_t*              data_fifo;
    uint32_t                 data_fifo_wrap;
    uint32_t                 data_fifo_w_idx;
    uint32_t                 data_fifo_r_idx;


    /*
     * FIFO Data raw (bitstream)
     */
    uint8_t*                 data_raw;
    uint32_t                 data_raw_w_idx;
    uint32_t                 data_raw_wrap;

    /*
     * COALECHING/DECODE buffer
     */
    uint8_t*                 data_coalesce;
    uint8_t*                 data_coalesce_ptr;
    int16_t*                 data_decode;

    /*
     * bitstream flow indexing
     */
    uint64_t                 data_bitstream_idx;
    uint8_t                  data_bitstream_skip_f;

    /* to allow skip bitstream */
    uint64_t                 data_size_prev;

    /*
     * search/classify
     */
    uint32_t                 search_idx;
    uint32_t                 search_status;
    uint8_t                  search_complete_f;

    /* MP4/M4A search components */
    mp4_sample_size_table_t  mp4_sample_size_table;
    mp4_chunk_offs_table_t   mp4_chunk_offs_table;
    mp4_sample_chunk_table_t mp4_sample_chunk_table;
    mp4_time_sample_table_t  mp4_time_sample_table;
    mp4_box_discard_t        mp4_discard;


    /* MP4 search results */
    uint8_t*                 mp4_audio_specific_info;
    uint32_t                 mp4_audio_specific_info_size;
    mp4_track_info_t         mp4_audio_track_info;

    /* DECODER */
    HANDLE_AACDECODER        fdk_handle;
    AAC_DECODER_ERROR        fdk_err;

    mp4_pcm_info_t           pcm_info;

    /* OUTPUT CALLBACK */
    uint32_t ( * output_push_callback )(
        int8_t*         err,
        void*           mp4parse,
        void*           decoder,
        mp4_pcm_info_t* pcm_info,
        int16_t*        buf,
        uint32_t        buf_len,
        uint32_t*       buf_id );

    uint32_t                 start_offset_ms;
    volatile uint32_t        wait_for_stream_position;

    /* DBG only features */
    uint64_t                 memory_footprint;
    tpm_config_t             tpm;
} mp4parse_t;



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
