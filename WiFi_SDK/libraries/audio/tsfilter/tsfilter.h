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
 * @file Public API for ISO/IEC 13818-1 Transport Stream Filter
 */

#pragma once

#include <stdint.h>
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TSFILTER_DEFAULT_PACKET_LENGTH (188)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    TSFILTER_AUDIO_TYPE_RESERVED    = 0x00,

    TSFILTER_AUDIO_TYPE_MPEG1       = 0x03, /* ISO/IEC 11172-3, MPEG-1 Part 3, MPEG-1 Audio    */
    TSFILTER_AUDIO_TYPE_MPEG2_BC    = 0x04, /* ISO/IEC 13818-3, MPEG-2 Part 3, MPEG-2 Audio BC */

    TSFILTER_AUDIO_TYPE_AAC_ADTS    = 0x0F, /* ISO/IEC 13818-7, MPEG-2 Part 7, AAC ADTS        */
    TSFILTER_AUDIO_TYPE_MPEG4_LOAS  = 0x11, /* ISO/IEC 14496-3, MPEG-4 Part 3, MPEG-4 Audio    */

    TSFILTER_AUDIO_TYPE_PCM         = 0x80,
    TSFILTER_AUDIO_TYPE_DD          = 0x81,
    TSFILTER_AUDIO_TYPE_DTS         = 0x82,
    TSFILTER_AUDIO_TYPE_DD_TRUEHD   = 0x83,
    TSFILTER_AUDIO_TYPE_DDPLUS_BD   = 0x84,
    TSFILTER_AUDIO_TYPE_DTSHD_HR    = 0x85,
    TSFILTER_AUDIO_TYPE_DTSHD_MA    = 0x86,
    TSFILTER_AUDIO_TYPE_DDPLUS_ATSC = 0x87,

} tsfilter_stream_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct tsfilter_s* tsfilter_ref;

typedef struct
{
    void                   *user_context;
    uint16_t                pid;
    tsfilter_stream_type_t  type;
} tsfilter_stream_type_params_t;

typedef struct
{
   void     *user_context;
   uint16_t  pid;
   uint8_t  *data;
   uint32_t  data_length;
} tsfilter_stream_data_params_t;

typedef wiced_result_t (*tsfilter_stream_type_cb_t)(tsfilter_stream_type_params_t *type_params);
typedef wiced_result_t (*tsfilter_stream_data_cb_t)(tsfilter_stream_data_params_t *data_params);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint8_t                    ts_packet_length;
    void                      *user_context;
    tsfilter_stream_type_cb_t  stream_type_cbf;
    tsfilter_stream_data_cb_t  stream_data_cbf;
} tsfilter_init_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize TSfilter
 *
 * @param[ in] params          : TSfilter input parameters
 * @param[out] filter_ptr      : Storage for TSfilter handle @ref tsfilter_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t tsfilter_init(tsfilter_init_params_t *params, tsfilter_ref *filter_ptr);


/** Clean-up TSfilter
 *
 * @param[in] filter           : TSfilter handle @ref tsfilter_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t tsfilter_deinit(tsfilter_ref filter);


/** Filter/parse TS packets contained in given byte buffer
 *
 * @param[in] filter           : TSfilter handle @ref tsfilter_ref
 * @param[in] buffer           : Pointer to buffer containing TS packets
 * @param[in] buffer_length    : Length of buffer in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t tsfilter_parse(tsfilter_ref filter, uint8_t *buffer, uint32_t buffer_length);


/** Reset state of TSfilter before submitting a new set of TS packets from a new/different stream
 *
 * @param[in] filter           : TSfilter handle @ref tsfilter_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t tsfilter_reset(tsfilter_ref filter);


#ifdef __cplusplus
} /* extern "C" */
#endif
