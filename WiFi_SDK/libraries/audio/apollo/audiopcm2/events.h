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
 * @file audiopcm_events.h
 *
 *
 */
#ifndef _H_AUDIOPCM_EVENTS_H_
#define _H_AUDIOPCM_EVENTS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "audiopcm_types.h"


/** documenting events following "5 Ws" rules
 * http://en.wikipedia.org/wiki/Five_Ws
 *
 * for now only 3 Ws (who, what, where)
 */

/**
 * audiopcm event: component ("who")
 */
typedef enum
{
    AUDIOPCM_E_CMP_UNKNOWN = 0,
    AUDIOPCM_E_CMP_ANYTYPE,
    AUDIOPCM_E_CMP_RESERVED,
    AUDIOPCM_E_CMP_AUDIO_SYNTAX,
    AUDIOPCM_E_CMP_INPUT_THREAD,
    AUDIOPCM_E_CMP_OUTPUT_THREAD,
    AUDIOPCM_E_CMP_RTP_THREAD,
    AUDIOPCM_E_CMP_DSP_THREAD,
    AUDIOPCM_E_CMP_CLK_THREAD,
    AUDIOPCM_E_CMP_MAX,
} event_component_t;

/**
 * audiopcm event: object ("where")
 */
typedef enum
{
    AUDIOPCM_E_OBJ_UNKNOWN = 0,
    AUDIOPCM_E_OBJ_ANYTYPE,
    AUDIOPCM_E_OBJ_RESERVED,
    AUDIOPCM_E_OBJ_AUDIO_STREAM,
    AUDIOPCM_E_OBJ_AUDIO_STREAM_SAMPLERATE,
    AUDIOPCM_E_OBJ_AUDIO_STREAM_BITPERSAMPLE,
    AUDIOPCM_E_OBJ_AUDIO_STREAM_BURSTLENGHT,
    AUDIOPCM_E_OBJ_AUDIO_RTP_BUF,
    AUDIOPCM_E_OBJ_AUDIO_RTP_QUEUE,
    AUDIOPCM_E_OBJ_AUDIO_DSP_BUF,
    AUDIOPCM_E_OBJ_AUDIO_DSP_QUEUE,
    AUDIOPCM_E_OBJ_AUDIO_SAMPLE_QUEUE,
    AUDIOPCM_E_OBJ_AUDIO_STEREO_BUF,
    AUDIOPCM_E_OBJ_AUDIO_RENDER_CBAK,
    AUDIOPCM_E_OBJ_AUDIO_OUT_BUF,
    AUDIOPCM_E_OBJ_AUDIO_OUT_QUEUE,
    AUDIOPCM_E_OBJ_MAX,
} event_object_t;

/**
 * audiopcm event: info ("what")
 */
typedef enum
{
    AUDIOPCM_E_INFO_UNKNOWN = 0,
    AUDIOPCM_E_INFO_ANYTYPE,
    AUDIOPCM_E_INFO_RESERVED,
    AUDIOPCM_E_INFO_ILLEGAL,
    AUDIOPCM_E_INFO_OVERFLOW,
    AUDIOPCM_E_INFO_UNDERFLOW,
    AUDIOPCM_E_INFO_CHANGE,
    AUDIOPCM_E_INFO_INVALID,
    AUDIOPCM_E_INFO_DISCONTINUITY,
    AUDIOPCM_E_INFO_BIG_GAP,
    AUDIOPCM_E_INFO_BL_SYNC_FAIL,
    AUDIOPCM_E_INFO_BL_SYNC_MISMATCH,
    AUDIOPCM_E_INFO_PLC_INIT_FAIL,
    AUDIOPCM_E_INFO_PLC_DECODE_FAIL,
    AUDIOPCM_E_INFO_OUTOFORDER,
    AUDIOPCM_E_INFO_MAX,
} event_info_t;


/**
 * audiopcm event: info ("what")
 */
typedef enum
{
    AUDIOPCM_E_FILE_UNKNOWN = 0,
    AUDIOPCM_E_FILE_ANYTYPE,
    AUDIOPCM_E_FILE_RESERVED,
    AUDIOPCM_E_FILE_AUDIOPCM,
    AUDIOPCM_E_FILE_RTP,
    AUDIOPCM_E_FILE_DSP,
    AUDIOPCM_E_FILE_OUT,
    AUDIOPCM_E_FILE_CLK,
    AUDIOPCM_E_FILE_WDOG,
    AUDIOPCM_E_FILE_CONCEALMENT,
} event_file_t;

/**
 * audiopcm event: who/where/what
 */
typedef struct event_
{
    uint32_t          id;

    /* who, where, why */
    event_component_t comp;
    event_object_t    obj;
    event_info_t      info;

    /* internal debug */
    event_file_t      dbg_file;
    uint32_t          dbg_line;
} event_t;

/*
 * define the string type
 */
typedef char event_str_t[ 256 ];



int8_t get_audiopcm_event_str( event_str_t event_str, event_t* event );



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_AUDIOPCM_EVENTS_H_ */
