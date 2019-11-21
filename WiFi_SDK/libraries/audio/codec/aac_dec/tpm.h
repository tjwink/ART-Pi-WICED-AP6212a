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
#ifndef _H_TPM_H_
#define _H_TPM_H_


#ifdef __cplusplus
extern "C" {
#endif

/** The application using this library must define
 * the setup for profiling config (TraceX for WICED)
 * and take care of logging server (or local debug dump)
 * these macros are used only for AAC_DEC internal
 * profiling USER events.
 */

typedef enum
{
    /* MP4FF/M4A */
    TPM_MP4_INPUT_PARSE_PUSH    = 7000,
    TPM_MP4_INPUT_AAC_PUSH,
    TPM_MP4_DSP_PKT_SEEK,
    TPM_MP4_DSP_CHUNK_SEEK,
    TPM_MP4_DSP_FDK_COALESCE,
    TPM_MP4_DSP_FDK_FILL_BEGIN,
    TPM_MP4_DSP_FDK_FILL_END,
    TPM_MP4_DSP_FDK_DECODE_BEGIN,
    TPM_MP4_DSP_FDK_DECODE_END,
    TPM_MP4_DSP_PCM_PUSH,
    /* MP4/ADTS */
    TPM_ADTS_INPUT_PARSE_PUSH,
    TPM_ADTS_INPUT_AAC_PUSH,
    TPM_ADTS_DSP_PKT_SEEK,
    TPM_ADTS_DSP_CHUNK_SEEK,
    TPM_ADTS_DSP_FDK_COALESCE,
    TPM_ADTS_DSP_FDK_FILL_BEGIN,
    TPM_ADTS_DSP_FDK_FILL_END,
    TPM_ADTS_DSP_FDK_DECODE_BEGIN,
    TPM_ADTS_DSP_FDK_DECODE_END,
    TPM_ADTS_DSP_PCM_PUSH,
} tpm_event_t;


typedef enum
{
    TPM_TOOL_UNKNOWN   = 0,
    TPM_TOOL_TRACEX,
    TPM_TOOL_GPROF,
    TPM_TOOL_MAX
} tpm_tool_t;


#if defined  ( LINUX )
    #include "tpm_x86.h"
#endif

#if defined  ( WICED )
    #include "tpm_wiced.h"
#endif



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /*_H_TPM_H_*/
