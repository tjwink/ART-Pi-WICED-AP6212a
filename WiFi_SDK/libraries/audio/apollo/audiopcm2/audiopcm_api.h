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
 * @file audiopcm_api.h
 *
 * audiopcm library API defintion
 *
 */
#ifndef _H_AUDIOPCM_API_H_
#define _H_AUDIOPCM_API_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "audiopcm_types.h"
#include "events.h"

#include <inttypes.h>


/*
 * The audiopcm library is implementing packet based error recovery (FEC,CPY) and
 * also SLC unshuffling for Apollo/BRCM audio specification as documented in
 *
 * http://confluence.broadcom.com/display/MobileMultimedia/Apollo+Wireless+Audio+Specification
 *
 * The input of the audiopcm component are the RTP packets from the upper
 * application, taken "as-is" from the socket output.
 *
 * The audiopcm will separate audio payload from CPY (redundant packets) data and
 * FEC (forward error corection, XOR) data, will try to recover missing packets based
 * on the available informations, and when there are no extra data to recover missing audio
 * packets audiopcm will trigger the audioplc (sample/packet loss concealment) from the
 * linked audioplc component library.
 *
 * NOTE: the name of the library (audioplc) should not be confused with the actual
 *       concealment technique, either PLC (ordered PCM samples in a single pkt using
 *       WSOLA algorithm) or SLC (sample loss concealment for PCM shuffled samples)
 *
 * NOTE: for multi-channel audio input the AudioPCL is triggered only after
 *       demuxing the required playback channel pcm samples (mono/stereo signal)
 *
 * The output of the audiopcm library is a single/stero channel PCM audio stream
 * based on the setting from the audiopcm_start call.
 *
 * The audiopcm has two input queue methods, WRITE and ZERO_COPY_WRITE
 * The normal write will introduce an extra memcpy, while the zero_copy version
 * is leveraging the internal demux queue as a buffer factory, holding the input buffer
 * (during zc_init) until the buffer is released (durinc zc_complete)
 *
 * NOTE: on wiced only the normal write is implemented, since the audio packets are
 *       passed from the network and not pulled from the audiopcm internal buffer factory.
 *
 * The audiopcm component has two facilities for internal or external usage.
 * - Watchdog: used for internal status check, queue overflow or and status check of inner threads.
 * - Clock: currenlty a "sync beat" only. Will be used to improve data processing when
 * audio_latency_target is set.
 *
 * Both Watchdog and Clock can be disabled on setup
 *
 * DATA_FLOW_MODEL:
 * The audiopcm component can work in PUSH(in)/PUSH(out) or PULL(in)/PULL(out) based
 * on which IN/OUT callbacks are set. Currenlty (and especially for WICED) the model
 * is fixed to be PUSH/PUSH which is the optimal structure for free-clock settings
 * since this will not affect the internal queues other than front/back pressure.
 *
 * The internal structure of audiopcm is composed of four main processing loops,
 * each one mapped on a single thread with in/out queues. These queues are needed to
 * handle the asyncronous data flow during demuxing and dsp (concealment) processing.
 *
 * Note: the input bitrate is usually a lot greather than the output data rate since
 * a maxium of two audio channels can be decoded from the input stream.
 *
 * The inner queues size and indexig has been specifically designed to allow a lock-free
 * data processing, handling queues as arrays instead of linked lists for speed purposes.
 */

/* ******************************************************************************************************* */
/* AudioPCM library data flow chart                                                                         */
/* ******************************************************************************************************** */

/*
 *                   rtp_fifo (multi ch payload)
 *                     +------------+
 * input_push_pkt ---> | | | | | .. |
 * (player app. call)  +------------+
 *                           |
 *                           |
 *                           V                       dsp_fifo (stereo, single ch payload)
 *                    {rtp_loop thread}                +------------+
 *                    does: FEC/CPY pkt recovery  ---> | | | | | ...|
 *                    does: pcm_demux                  +------------+
 *                    does: FEC/CPY/PCM references     | | | | | ...|
 *                                                     +------------+
 *                                                           |
 *                                                           |
 *                                                           V                   output_fifo (stereo)
 *                                                   {dsp_loop_thread}             +------------+
 *                                                   does: SLC unshuffling    ---> | | | | | ...|
 *                                                   does: audioSLC/PLC calls      +------------+
 *                                                                                 | | | | | ...|
 *                                                                                 +------------+
 *                                                                                       |
 *                                                                                       |
 *  {watchdog thread}                                                                    V
 *  does: queue level monitor (backpressure)                                   {output_loop_thread}
 *                                                                           does: audio_output_push callback
 *                                                                                 to the mediaplayer app.
 *  {clock thread}                                                           does: mono-to-stereo conversion
 *  does: heartbeat                                                                if needed (single ch decode)
 *  does: queue martialling when latency is fixed                                        |
 *                                                                                       |
 *                                                                                       V
 *                                                                                audio_output_push
 *                                                                               (player app callback)
 */

/* ******************************************************************************************************** */
/* SYSTEM                                                                                                   */
/* ******************************************************************************************************** */

/** audio_new: constructor
 * @param err                 : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm            : will return the handle to the component once created
 * @param logging_level       : from LOG_LOW to LOG_MAX (see logger.h)
 * @param max_latency_ms      : the maximum overall latency for playback (in ms)
 * @param max_jitter_ms       : the maximum overall jitter (in ms)
 * @param max_rtt_ms          : the maximum RTT time (in ms)
 * @param max_bl              : maximum BURST LENGTH (BL) for the playback session
 * @param max_pcm_info        : pointer to a given pcm_info_t struct with the maximum audio paremters
 * @param dmx_max_channel_cnt : maximum number of channels to demux (max 2)
 * @param max_concealment     : maximum level of concealment to apply (NONE|PLC|SLC)
 * @return value is ZERO on success, NON zero on error.
 */
int8_t audiopcm_new( int8_t*     err,
                     void**      audiopcm,
                     uint8_t     logging_level,
                     uint16_t    max_latency_ms,
                     uint16_t    max_jitter_ms,
                     uint16_t    max_rtt_ms,
                     uint8_t     max_bl,
                     pcm_info_t* max_pcm_info,
                     uint8_t     dmx_max_channel_cnt,
                     uint8_t     max_concealment );

/** audio_start: will start the component by creating the internal processing threads
 * @param err                       : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm                  : handle to a valid audiopcm component
 * @param target_pcm_info           : pointer to the expected pcm_info for the playback session
 * @param target_audio_ch_list      : channel demux list (with descending priority)
 * @param target_audio_ch_list_size : number of entries for the audio_ch_list
 * @param target_audio_latency_ms   : expected overall playback latency
 */
int8_t audiopcm_start( int8_t*     err,
                       void*       audiopcm,
                       pcm_info_t* target_pcm_info,
                       uint32_t*   target_audio_ch_list,
                       uint8_t     target_audio_ch_list_size,
                       uint16_t    target_audio_latency_ms );

/** audio_stop: will stop the component without destroying the internal processing threads and queues
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_stop( int8_t* err, void* audiopcm );

/** audio_destroy: will destroy the component (audiopcm must be stopped first!)
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_destroy( int8_t* err, void** audiopcm );


/* ******************************************************************************************************** */
/* SETTERS                                                                                                  */
/* ******************************************************************************************************** */

/** audiopcm_set_label: will set a component label, used on LOG_MSG (must be set before audiopcm_start)
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_set_label( int8_t* err, void* audiopcm, char* label );

/** audiopcm_set_plc: turn on/off the AudioPLC sub-component (must be set before audiopcm_start)
 * @param err        : pointer to a given return error variable (ignored if NULL)
 * @param enable_plc : boolean
 * @param audiopcm   : handle to a valid audiopcm component
 */
int8_t audiopcm_set_plc( int8_t* err, void* audiopcm, uint8_t enable_plc );

/** audiopcm_set_slc: turn on/off the AudioSLC sub-component (must be set before audiopcm_start)
 * @param err        : pointer to a given return error variable (ignored if NULL)
 * @param enable_slc : boolean
 * @param audiopcm   : handle to a valid audiopcm component
 */
int8_t audiopcm_set_slc( int8_t* err, void* audiopcm, uint8_t enable_slc );

/** audiopcm_set_clock: turn on/off clock support (must be set before audiopcm_start)
 * @param err          : pointer to a given return error variable (ignored if NULL)
 * @param enable_clock : boolean
 * @param audiopcm     : handle to a valid audiopcm component
 */
int8_t audiopcm_set_clock( int8_t* err, void* audiopcm, uint8_t enable_clock );

/** audiopcm_set_watchdog: turn on/off watchdog support (must be set before audiopcm_start)
 * @param err             : pointer to a given return error variable (ignored if NULL)
 * @param enable_watchdog : boolean
 * @param audiopcm        : handle to a valid audiopcm component
 */
int8_t audiopcm_set_watchdog( int8_t* err, void* audiopcm, uint8_t enable_watchdog );

/** audiopcm_set_log_verbosity: set the log level for LOG_MSG verbosity
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param lvl      : verbose level (0->5)
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_set_log_verbosity( int8_t* err, void* audiopcm, uint8_t lvl );

/** audiopcm_set_userid: set the user id of the audiopcm component, useful for debug
 *                       when multiple audiopcm components are created.
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param userid   : a unique numeric ID to identify the audiopcm component
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_set_userid( int8_t* err, void* audiopcm, uint8_t userid );

/** audiopcm_set_chmap_custom: set a user customized channel map via command message
 *
 * @param err : pointer to a given return error variable (ignored if NULL)
 * note: to be implemented for Apollo Command message
 */
int8_t audiopcm_set_chmap_custom( int8_t* err, void* audiopcm, char* buf, uint32_t buf_len );

/** audiopcm_set_label_statistics_filename: set the output stats filename (NOT supported on WICED)
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param filename : full path and name of the statistics file
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_set_statistics_filename( int8_t* err, void* audiopcm, char* filename );

/** audiopcm_set_output_filename: set the output dumps base filename (NOT supported on WICED)
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param filename : full path and name of the output file
 * @param audiopcm : handle to a valid audiopcm component
 */
int8_t audiopcm_set_output_filename( int8_t* err, void* audiopcm, char* file );

/** audiopcm_set_target_match_map_bypass: force audioplayback if target channel does not match channel map.
 * audio will fall back on MONO (FL) channel.
 * @param err          : pointer to a given return error variable (ignored if NULL)
 * @param boolean_flag : value 1 will force audio playback on channel mismatch
 */
int8_t audiopcm_set_target_match_map_bypass( int8_t* err, void* audiopcm, uint8_t boolean_flag );


/* ******************************************************************************************************** */
/* GETTERS                                                                                                  */
/* ******************************************************************************************************** */

/** audiopcm_get_is_valid: check if the audiopcm component is valid
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @return return value is ZERO on success, NON zero on error.
 */
int8_t audiopcm_get_is_valid( int8_t* err, void* audiopcm );

/** audiopcm_get_api_version: get the Apollo Audio API version implemented by the component
 * Api versioning: MJR.MIN with a flag to tell if the component is in debug mode
 * @param err       : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm  : handle to an audiopcm component
 * @param api_major : pointer for MJR api number
 * @param api_minor : pointer for MIN api number
 * @param is_debug  : optional flag to indicate if the component has been built in DEBUG mode
 */
int8_t audiopcm_get_api_version( int8_t* err, void* audiopcm,
                                 uint8_t* api_major, uint8_t* api_minor, uint8_t* is_debug );


/** audiopcm_get_status: get the Apollo audiopcm component status
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @param status   : pointer to audiopcm_cstatus variable
 * @return return value is ZERO on success, NON zero on error.
 */
int8_t audiopcm_get_status( int8_t* err, void* audiopcm, audiopcm_cstatus_t* status );


/** audiopcm_get_version: get the Audiopcm sw version implemented by the component.
 * Note: this is not the API version but the actual version of the component,
 *       useful to track incremental sw revisions when linking.
 * versioning: MJR.MIN.revision
 * this is useful to distinguish components that have the same Apollo API version
 * but have a different sw implementation.
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @param major
 * @param minor
 * @param revision string for revision control
 */
int8_t audiopcm_get_version( int8_t* err, void* audiopcm,
                             uint8_t* major, uint8_t* minor, char* revision );

/** audiopcm_get_userid: get the userid of the provided audiopcm component
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @param userid   : userid value of the audiopcm component
 */
int8_t audiopcm_get_userid( int8_t* err, void* audiopcm, uint8_t* userid );


/** audiopcm_get_userid: get the userid of the provided audiopcm component
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @param stats    : handle to the statistics return for the playback session
 */
int8_t audiopcm_get_stats( int8_t* err, void* audiopcm, audiopcm_stats_t* stats );

/* ******************************************************************************************************** */
/* TOOLS                                                                                                    */
/* ******************************************************************************************************** */

/** chmap_to_chnum: convert a channel map to a specific channel number value as per Apollo audio specification
 * @param err      : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm : handle to an audiopcm component
 * @param chmap    : a valid (per spec) channel map value
 * @return the number of channels specified by the chmap value
 */
int8_t chmap_to_chnum( int8_t* err, int8_t chmap );

/** audiopcm_get_optimal_params
 * based on the input configuration return the optimal values for streaming configuration
 * by setting BL, SL and audio samples per pkt payload
 * @param err            : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm       : handle to an audiopcm component
 * @param bps            : bit per sample of the audio stream
 * @param cbps           : container bit per sample (16, 24, 32) of the given bps
 * @param sr             : sample_rate value in herts (44100, 48000, 96000)
 * @param ch_num         : number of audio channels per payload
 * @param target_latency : the required target latency for the streaming session
 * @param bl             : computed optimal BL value
 * @param sl             : computed optimal SL value
 * @param audio_samples  : computed optimal number of audio samples per channel per payload
 */
int8_t get_optimal_stream_params( int8_t* err, void* audiopcm,
                                  uint8_t bps, uint8_t cbps, uint32_t sr,
                                  uint8_t ch_num, uint8_t target_latency,
                                  uint8_t* bl, uint8_t* sl, uint16_t* audio_samples );


/** get_is_legal_slc_payload_length
 * validate the given set of values based on the Apollo specification and
 * return a valid/invalid indication on the given combination
 * @param err           : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm      : handle to an audiopcm component
 * @param bps           : bit per sample of the audio stream
 * @param cbps          : container bit per sample (16, 24, 32) of the given bps
 * @param sr            : sample_rate value in herts (44100, 48000, 96000)
 * @param ch_num        : number of audio channels per payload
 * @param bl            : burst value
 * @param sl            : shuffle length value
 * @param audio_samples : number of audio samples per channel per payload
 * @param specs_f       : (boolean flag) if "1" it will enforce the specification
 *                        BL/SL configuration matrix or it will compute validity based
 *                        on bl/sl/alignement sanity checks
 * @return true/false if the given configuration is legal as per Apollo specification
 */
int8_t get_is_legal_slc_payload_length( int8_t* err, void* audiopcm,
                                        uint8_t bps, uint8_t cbps, uint32_t sr,
                                        uint8_t ch_num, uint8_t bl, uint8_t sl,
                                        uint16_t audio_samples, uint8_t specs_f );


/* ******************************************************************************************************** */
/* DATA QUEUES SPECIFIC */
/* ******************************************************************************************************** */

/** audiopcm_get_queue_level: retrieve the level and max size of the audiopcm input RTP queue
 * @param err                  : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm             : handle to an audiopcm component
 * @param audiopcm_queue_level : the current RTP queue level
 * @param audiopcm_queue_size  : the maximum RTP queue level
 */
int8_t audiopcm_get_queue_level( int8_t* err, void* audiopcm,
                                 uint32_t* audiopcm_queue_level, uint32_t* audiopcm_queue_size );

/** audiopcm_set_alert_overflow_queue: set the level on the RTP input queue to trigger the alert for OVERFLOW
 * if the alert callback is set by the upper application the component will raise an alert when the level
 * of the queue is above the given threshould
 * @param err                  : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm             : handle to an audiopcm component
 * @param audiopcm_queue_level : alert threshould for overflow
 */
int8_t audiopcm_set_alert_overflow_queue( int8_t*  err,
                                          void*    audiopcm,
                                          uint32_t audiopcm_queue_level );

/** audiopcm_set_alert_underflow_queue: set the level on the RTP input queue to trigger the alert for UNDERFLOW
 * if the alert callback is set by the upper application the component will raise an alert when the level
 * of the queue is below the given threshould
 * @param err                  : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm             : handle to an audiopcm component
 * @param audiopcm_queue_level : alert threshould for underflow
 */
int8_t audiopcm_set_alert_underflow_queue( int8_t*  err,
                                           void**   audiopcm,
                                           uint32_t audiopcm_queue_level );



/* ******************************************************************************************************** */
/* CALLBACKS                                                                                                */
/* ******************************************************************************************************** */

/* *********************************************** */
/* ALERTS, HEARTBEAT, EVENT calls                  */
/* *********************************************** */


/** audiopcm_set_alert
 *  set the callback to raise for an internal system event (like underflow/overflow, missing component)
 * @param err pointer to a given return error variable (ignored if NULL)
 * @param audiopcm           : handle to an audiopcm component
 * @param player_app         : pointer to the upper player application
 * @param sys_alert_callback : pointer to the upper application callback
 */
int8_t audiopcm_set_system_alert_callback( int8_t*      err,
                                           void*        audiopcm,
                                           void*        player_app,
                                           int8_t ( *   sys_alert_callback )(
                                               int8_t*  err,
                                               void*    audiopcm,
                                               void*    player_app,
                                               event_t* event,
                                               void*    event_data,
                                               uint32_t event_data_size ) );

/** public set/get method: set_audio_event
 *  set the callback to raise an event for the AUDIO stream only (like format change, sample_cnt)
 * @param err                  : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm             : handle to an audiopcm component
 * @param player_app           : pointer to the upper player application
 * @param audio_alert_callback : pointer to the upper application callback
 */
int8_t audiopcm_set_audio_alert_callback( int8_t*      err,
                                          void*        audiopcm,
                                          void*        player_app,
                                          int8_t ( *   audio_alert_callback )(
                                              int8_t*  err,
                                              void*    audiopcm,
                                              void*    player_app,
                                              event_t* event,
                                              void*    event_data,
                                              uint32_t event_data_size ) );

/** public set/get method: set_audiopcm_hearbeat_callback
 *  set the callback to request an heartbeat every (x) milliseconds
 *  note: audiopcm clock must be enabled to send heartbeat calls
 * @param err         : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm    : handle to an audiopcm component
 * @param player_app  : pointer to the upper player application
 * @param millisecond : interval for hearbeat callback
 */
int8_t audiopcm_set_heartbeat_callback( int8_t*     err,
                                        void*       audiopcm,
                                        void*       player_app,
                                        uint32_t    milliseconds,
                                        int8_t ( *  heartbeat_callback )(
                                            int8_t* err,
                                            void*   audiopcm,
                                            void*   player_app ) );


/* *********************************************** */
/* DATA I/O: INPUT: PUSH(int) calls                */
/* *********************************************** */

/** audiopcm_input_push_pkt
 *  push an RTP packet to the audiopcm input. Audiopcm will handle the content internally.
 *  if needed the application should release the packet after this call.
 *  Note: for Apollo audio RTP packets the audio_info should be set to NULL,
 *  all the details on format and content will be derived by the Apollo audio header format
 *  of the RTP payload
 * @param err        : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm   : handle to an audiopcm component
 * @param rtp_sn     : sequence number of the RTP pkt header
 * @param buf        : pointer to the rtp packet data
 * @param buf        : rtp packet data length
 * @param audio_info : optional descriptor for the given RTP packet
 */
uint32_t audiopcm_input_push_pkt( int8_t*       err,
                                  void*         audiopcm,
                                  uint16_t      rtp_sn,
                                  char*         buf,
                                  uint32_t      buf_len,
                                  audio_info_t* audio_info );

/** audiopcm_input_push_pkt_zc: same as the above, but the application will request the packet
 *  from the internal audiopcm buffer queue directly, saving one memcpy stage.
 *
 *  Note: NOT implemented yet.
 */
int8_t audiopcm_input_push_zc_begin( int8_t* err, void* audiopcm, rtp_buf_t** audio_rtp_pkt );
int8_t audiopcm_input_push_zc_end  ( int8_t* err, void* audiopcm, rtp_buf_t* audio_rtp_pkt );



/* *********************************************** */
/* DATA I/O: PUSH(out) callbacks                   */
/* *********************************************** */

/** audiopcm_set_output_push_callback
 *  callback for the output stage called once PCM samples are available from the output queue
 *  of the audiopcm component to be sent to the upper application
 * @param err                        : pointer to a given return error variable (ignored if NULL)
 * @param audiopcm handle            : to an audiopcm component
 * @param player_app                 : handle for the upper application
 * @param audio_output_push_callback : application cback
 */
int8_t audiopcm_set_output_push_callback( int8_t*            err,
                                          void*              audiopcm,
                                          void*              player_app,
                                          int8_t ( *         audio_output_push_callback )(
                                              int8_t*        err,
                                              void*          audiopcm,
                                              void*          player_app,
                                              unsigned char* buf,
                                              uint32_t*      buf_len,
                                              uint32_t*      buf_id,
                                              audio_info_t*  audio_info ) );

/** audiopcm_set_output_push_zc
 *  same as the above but the buffer is requested from the buffer pool of the application itself,
 *  skipping one memcpy.
 *
 *  Note: NOT implemented yet.
 */
int8_t audiopcm_set_output_push_zc_begin_callback( int8_t*             err,
                                                   void*               audiopcm,
                                                   void*               player_app,
                                                   int8_t ( *          audio_output_push_zc_begin_callback )(
                                                       int8_t*         err,
                                                       void*           audiopcm,
                                                       void*           player_app,
                                                       unsigned char** buf,
                                                       uint32_t*       buf_len,
                                                       uint32_t*       buf_id,
                                                       audio_info_t*   audio_info ) );

int8_t audiopcm_set_output_push_zc_end_callback( int8_t*            err,
                                                 void*              audiopcm,
                                                 void*              player_app,
                                                 int8_t ( *         audio_push_zc_end_callback )(
                                                     int8_t*        err,
                                                     void*          audiopcm,
                                                     void*          player_app,
                                                     unsigned char* buf,
                                                     uint32_t*      buf_len,
                                                     uint32_t*      buf_id,
                                                     audio_info_t*  audio_info ) );



/* *********************************************** */
/* DATA COMPRESSION PRIMITIVES                     */
/* *********************************************** */

/* to compress means to encode */
int8_t   audiopcm_comp_enc_new( int8_t* err, void** compressor, pcm_info_t* pcm_info, uint32_t frames_num );
uint32_t audiopcm_comp_enc_payload( int8_t* err, void* compressor, uint8_t* dst, uint8_t* src, pcm_info_t* pcm_info, uint32_t frames_num );
int8_t   audiopcm_comp_enc_destroy( int8_t* err, void** compressor );

/* to decompress means to decode */
int8_t   audiopcm_comp_dec_new( int8_t* err, void** compressor );
uint32_t audiopcm_comp_dec_payload( int8_t* err, void* compressor, uint8_t* dst, uint8_t* src );
int8_t   audiopcm_comp_dec_destroy( int8_t* err, void** compressor );



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_AUDIOPCM_H_ */
