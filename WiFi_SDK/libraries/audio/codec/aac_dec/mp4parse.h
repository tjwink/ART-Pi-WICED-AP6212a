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
#ifndef _H_MP4PARSE_H_
#define _H_MP4PARSE_H_



#ifdef __cplusplus
extern "C" {
#endif


#include "mp4parse_types.h"
#include "tpm.h"

#define MP4PARSE_DEFAULT_PRIORITY         ( WICED_DEFAULT_LIBRARY_PRIORITY  )
#define MP4PARSE_DEFAULT_STACK_SIZE                              ( 1024 * 8 )
#define MP4PARSE_DEFAULT_LABEL                                     "mp4parse"

typedef struct mp4parse_t* mp4parse_ref;


/* ******************************************************************************* */
/* SYSTEM                                                                          */
/* ******************************************************************************* */

void mp4parse_new( int8_t* err,
                   void**  mp4parse,
                   void*   decoder,
                   uint8_t logging_level );


void mp4parse_start( int8_t* err,
                     void*   mp4parse );


void mp4parse_stop( int8_t* err,
                    void*   mp4parse );

void mp4parse_flush( int8_t* err,
                     void*   mp4parse );

void mp4parse_destroy( int8_t* err,
                       void**  mp4parse );


/* ******************************************************************************* */
/* SETTERS                                                                         */
/* ******************************************************************************* */
void mp4parse_set_output_push_callback( int8_t*             err,
                                        void*               mp4parse,
                                        uint32_t ( *        output_push_callback )(
                                            int8_t*         err,
                                            void*           mp4parse,
                                            void*           decoder,
                                            mp4_pcm_info_t* pcm_info,
                                            int16_t*        buf,
                                            uint32_t        buf_len,
                                            uint32_t*       buf_id ) );

void mp4parse_set_log_verbosity( int8_t* err,
                                 void*   mp4parse,
                                 uint8_t lvl );

void mp4parse_set_tpconfig( int8_t*    err,
                            void*      mp4parse,
                            void*      tpconfig,
                            tpm_tool_t tool_type );

uint32_t mp4parse_set_ExtAudioSpecificConfig( int8_t*  err,
                                              void*    mp4parse,
                                              char*    asc,
                                              uint32_t asc_len );

uint32_t mp4parse_set_ExtSampleSizeTable( int8_t*  err,
                                          void*    mp4parse,
                                          char*    stsz,
                                          uint32_t stsz_len );

uint32_t mp4parse_set_ExtChunkOffsetTable( int8_t*  err,
                                           void*    mp4parse,
                                           char*    stco,
                                           uint32_t stco_len );

uint32_t mp4parse_set_ExtSampleToChunkTable( int8_t*  err,
                                             void*    mp4parse,
                                             char*    sttc,
                                             uint32_t sttc_len );

void mp4parse_set_start_offset( int8_t*  err,
                                void*    mp4parse,
                                uint32_t start_offset_ms );

void mp4parse_set_stream_position( int8_t*  err,
                                   void*    mp4parse,
                                   uint32_t stream_position );

/* ******************************************************************************* */
/* GETTERS                                                                         */
/* ******************************************************************************* */
void mp4parse_get_trackInfo( int8_t*           err,
                             void*             mp4parse,
                             mp4_track_info_t* mp4_track_info );

/* ******************************************************************************* */
/* TOOLS                                                                           */
/* ******************************************************************************* */


/* *********************************************** */
/* DATA I/O: INPUT: PUSH(int) calls                */
/* *********************************************** */

uint32_t mp4parse_input_push_buf( int8_t*  err,
                                  void*    mp4parse,
                                  char*    buf,
                                  uint32_t buf_len,
                                  uint8_t  buf_type );

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_MP4PARSE_H_ */
