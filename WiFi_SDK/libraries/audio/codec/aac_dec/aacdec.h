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

#ifndef _H_AACDEC_H_
#define _H_AACDEC_H_


#ifdef __cplusplus
extern "C" {
#endif

/**
 * AAC decoder wrapper for Fraunhofer AAC SDK (FDK)
 * Thi library is dedicated to the decoding of .m4a audio files
 * with a simplified API to decode an incoming packet stream into
 * PCM samples.
 *
 * this library does take care of "parsing" the MP4 syntax to
 * extract the audio samples, decode them by calling the FDK
 * and return the PCM stream to the upper layer (audio player)
 *
 * NOTE: this is not a fill MP4 demuxer, it is only a lightweigth
 * parser for audio files.
 *
 * since this library is oriented to the STREAMING use case there
 * is currenlty NO support for trick playback. Only START/STOP
 * are supported to allow continuos playback.
 *
 * During the create (new) call the application must specify
 * the requested channel map (same as APOLLO specification) and
 * PCM format targets. The library will internally convert to match
 * the requested format.
 */

#include "aacdec_types.h"

typedef struct aacdec_t* aadec_ref;

/* ******************************************************************************* */
/* SYSTEM                                                                          */
/* ******************************************************************************* */

/**
 * aadec_new:
 * does create the decoder component and returns the handle to it.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         OUT: returns the decoder handle, NULL if there was an error
 * \param player:         IN:  input pointer to the upper (player) application, cannot be NULL
 * \param logging_level:  IN:  verbosity of logging messages, from LOG_NONE to LOG_MAX (logger.h)
 * \param target_pcm_fmt: IN:  requested PCM format needed for optimal audio rendering
 * \param target_chmap:   IN:  target channel map for audio rendering, max 2 channels
 *                        needed to handle playback of multi channels audio streams,
 *                        works in combination with the downmix_f flag
 * \param downmix_f       IN:  when the input stream has more channels than the target_chmap
 *                        will define if audio downmix is requested to the FDK decoder
 *                        or instead "audio channel extraction" is performed by this library.
 */
void aacdec_new( int8_t*            err,
                 void**             aacdec,
                 void*              player,
                 uint8_t            logging_level,
                 aacdec_stream_t    stream_type,
                 aacdec_pcm_info_t* target_pcm_fmt,
                 uint32_t           target_chmap,
                 uint8_t            downmix_f );


/**
 * aadec_start:
 * once the aacdec component is created the aacdec_start call will launch the internal threads
 * for audio parsing and decoding. NOTE: new()/start() calls are separated to allow a separate handling
 * of the delays of memory buffer creation during new().
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN:  handle to the aac decoder component
 */
void aacdec_start( int8_t* err,
                   void*   aacdec );


/**
 * aadec_stop:
 * stop call to the decoder that will immediately drop any input push call.
 * if the flush flag is set the decoder will decode and push (to the player)
 * any remaining AAC sample in the input queue before returning the stop call.
 *
 * NOTE: even a single RTP packet
 * payload can contain quite a few AAC samples and so flushing will introduce
 * a delay on the audio output stop from the end user experience point of view.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN:  handle to the aac decoder component
 * \flush_f:              IN:  force decode/push of the remainig samples.
 */
void aacdec_stop( int8_t* err,
                  void*   aacdec,
                  uint8_t flush_f );


/**
 * aadec_flush:
 * custom flush call to decode/push all the remainig samples of the codec queue
 * to be used only when the decoder is in a stopped state ( to force the flush if needed )
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN:  handle to the aac decoder component
 */
void aacdec_flush( int8_t* err,
                   void*   aacdec );


/**
 * aadec_destroy:
 * close the decoder and destroy the component, if the force_f is NOT set
 * all the remainig samples in the queue are decoded and sent to the player first.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN:  handle to the aac decoder component
 * \param force_f:        IN:  force the immediate close of the decoder
 */
void aacdec_destroy( int8_t* err,
                     void**  aacdec,
                     uint8_t force_f );

/* ******************************************************************************* */
/* SETTERS                                                                         */
/* ******************************************************************************* */

/**
 * aacdec_set_log_verbosity:
 * sets the level of verbosity for DEBUG builds.
 * Note: RELEASE builds will not generate any logging, a specific EVENT api
 * is provided to trigger events for internal state monitoring.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN:  handle to the aac decoder component
 * \param lvl:            IN:  log level from LOG_NONE to LOG_MAX
 */
void aacdec_set_log_verbosity( int8_t* err,
                               void*   aacdec,
                               uint8_t lvl );

/**
 * aacdec_set_output_push_callback:
 * sets the callback to return PCM samples to the upper application
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN: handle to the aac decoder component
 * \param output_push_callback: player call back for PCM tranfer.
 *
 * the callback has the following parameters:
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         OUT: handle to the aac decoder component
 * \param player:         OUT: upper application handle set on aacdec_new()
 * \param pcm_info        OUT: PCM format descriptor pointer (sr, bps, endiannes etc.)
 * \param buf             OUT: raw buffer pointer for PCM data
 * \param buf_len         OUT: length of the PCM buffer
 * \param buf_id          OUT: parameter used for zero_copy OR inline commands (like EOS)
 */
void aacdec_set_output_push_callback( int8_t*                err,
                                      void*                  aacdec,
                                      uint32_t ( *           output_push_callback )(
                                          int8_t*            err,
                                          void*              aacdec,
                                          void*              player,
                                          aacdec_pcm_info_t* pcm_info,
                                          uint8_t*           buf,
                                          uint32_t           buf_len,
                                          uint32_t*          buf_id ) );


/**
 * aacdec_set_parameter:
 *
 * IMPORTANT: most of the parameters must be set BEFORE the decoder starts.
 * a call for a parameter set while the decoder is running will return an error.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN: handle to the aac decoder component
 * \param parameter:      IN: decoder  aacdec_param_t  to be set by the call
 * \param value:          IN: simple value of the above parameter
 * \param data:           IN: raw buffer for the parameter payload
 * \param data_len:       IN: length (in bytes) of the parameter buffer configuration
 */
uint32_t aacdec_set_parameter( int8_t*        err,
                               void*          aacdec,
                               aacdec_param_t parameter,
                               uint32_t       value,
                               void*          data,
                               uint32_t       data_len );


/* ******************************************************************************* */
/* GETTERS                                                                         */
/* ******************************************************************************* */
void aacdec_get_trackInfo( int8_t*              err,
                           void*                aacdec,
                           aacdec_track_info_t* audio_track_info );


/* ******************************************************************************* */
/* TOOLS                                                                           */
/* ******************************************************************************* */


/* *********************************************** */
/* DATA I/O: INPUT: PUSH(int) calls                */
/* *********************************************** */


/**
 * aacdec_input_push_buf:
 * main call to push aac packets into the decoder. When enough data are
 * pushed to decode a frame the decoder will trigger the internal PCM callback
 * to the upper application.
 *
 * \param err:            OUT: for error return, ignored if NULL.
 * \param aacdec:         IN: handle to the aac decoder component
 * \param buf:            IN: byte pointer to the input MP4A payload data
 * \param buf_len:        IN: lenght (num of bytes) of the payload buffer
 * \param buf_type:       IN: type, distinguish commands from data (0 for data)
 */
uint32_t aacdec_input_push_buf( int8_t*  err,
                                void*    aacdec,
                                char*    buf,
                                uint32_t buf_len,
                                uint8_t  buf_type );

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /*_H_AACDEC_H_*/
