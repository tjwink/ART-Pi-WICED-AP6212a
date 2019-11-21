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
#ifndef _H_AACDEC_CORE_H_
#define _H_AACDEC_CORE_H_


#ifdef __cplusplus
extern "C" {
#endif

#define OUTPUT_BUF_SIZE ( 8 * 2 * 1024 )


#include "aacdec_types.h"
#include "aacdec.h"
#include "tsp.h"
#include "tpm.h"

/**
 * audio component struct
 *
 * Note: compile optional fields (under #ifdef) MUST be at the very end,
 *       to avoid structure misalignement on binary exectution/debug
 *
 * Note: COMPONENT_TAG(), COMPONENT_SWVER(), COMPONENT_DBG() must be
 *       at the very top, and the order is FIXED.
 */

typedef struct aacdec_
{
    /* COMPONENT_TAG() */
    uint32_t            id;
    uint32_t            wmark;
    uint32_t            type;

    /* COMPONENT_SWVER() */
    uint8_t             major;
    uint8_t             minor;
    char                revision[ 32 ];
    char                build[ 5 ];

    /* COMPONENT_DBG() */
    uint8_t             log_lvl;
    uint8_t             log_err;
    char                label[ 64 ];

    /* COMPONENT_ATTRIBUTES() */

    /**  simple CONTROL register. (8 bit flags to control the processing loop) */

    /**
     *- bit 0  : ON/OFF flag
     *- bit 1  : reserved
     *- bit 2  : reserved
     *- bit 3  : reserved
     *- bit 4  : reserved
     *- bit 5  : reserved
     *- bit 6  : reserved
     *- bit 7  : reserved
     *
     * Note: ctrl MUST be always the first byte of the struct after the COMPONENT_BASE_T()
     */

    uint8_t volatile    ctrl;
    tsp_mutex_t         ctrl_mtx;

    uint8_t             userid;

    /* COMPONENT_STATUS() */
    aacdec_cstatus_t    status;

    /* UPPER component */
    void*               player;

    /* stream transport type */
    aacdec_stream_t     stream_type;

    /* parser/decoder */
    void*               mp4parse;

    /* PCM format conversion */
    uint8_t             pcm_info_tgt_f;
    aacdec_pcm_info_t   pcm_info_tgt;
    uint32_t            pcm_info_tgt_chmap;
    uint8_t             pcm_info_downmix_f;

    uint8_t             pcm_info_src_f;
    aacdec_pcm_info_t   pcm_info_src;

    uint8_t             track_info_f;
    aacdec_track_info_t track_info;

    uint8_t*            data_output;

    /* OUTPUT CALLBACK */
    uint32_t ( * output_push_callback )(
        int8_t*            err,
        void*              aacdec,
        void*              player,
        aacdec_pcm_info_t* pcm_info,
        uint8_t*           buf,
        uint32_t           buf_len,
        uint32_t*          buf_id );

    uint32_t                    start_offset_ms;
    aacdec_start_offset_cb_t    start_offset_callback;

    /* DBG only features */
    uint64_t            memory_footprint;
    tpm_config_t        tpm;
} aacdec_t;



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
