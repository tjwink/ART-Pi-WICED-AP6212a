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
 * @file Apollo Audio Streamer
 */

#pragma once

#include "wiced_result.h"
#include "wiced_tcpip.h"
#include "apollo_rtp_params.h"
#include "platform_audio.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_STREAMER_BURST_AUTO_SLC      (0xF0F0F0F0)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_AUDIO_SOURCE_CAPTURE,                           /*!< Audio from WICED audio capture/RS device                                  */
    APOLLO_AUDIO_SOURCE_BT,                                /*!< Bluetooth audio source - A2DP only -                                      */
    APOLLO_AUDIO_SOURCE_EXTERNAL,                          /*!< Audio coming from a source external to the streaming engine               */
} apollo_audio_source_type_t;

typedef enum
{
    APOLLO_STREAMER_EVENT_CONNECTED,
    APOLLO_STREAMER_EVENT_DISCONNECTED,
    APOLLO_STREAMER_EVENT_PLAYBACK_STARTED,
    APOLLO_STREAMER_EVENT_PLAYBACK_STOPPED,
    APOLLO_STREAMER_EVENT_VOLUME_CHANGED,
    APOLLO_STREAMER_EVENT_PLAYBACK_STATUS,
    APOLLO_STREAMER_EVENT_TRACK_METADATA,
    APOLLO_STREAMER_EVENT_BUFFER_RELEASED,
} apollo_streamer_event_t;

typedef enum
{
    APOLLO_STREAMER_COMMAND_PUSH_BUFFER,                   /*!< EXTERNAL audio source only: argument must be of type @ref apollo_streamer_buf_t */
    APOLLO_STREAMER_COMMAND_SET_AUDIO_CONFIG,              /*!< EXTERNAL audio source only: argument must be of type @ref wiced_audio_config_t  */
    APOLLO_STREAMER_COMMAND_START,                         /*!< EXTERNAL audio source only: no argument                                   */
    APOLLO_STREAMER_COMMAND_STOP,                          /*!< EXTERNAL audio source only: no argument                                   */
    APOLLO_STREAMER_COMMAND_SET_VOLUME_ATTENUATION,        /*!< argument must be of type @ref uint8_t from 0 to 100                       */
    APOLLO_STREAMER_COMMAND_SEND_EOS,                      /*!< EXTERNAL audio source only: no argument                                   */
    APOLLO_STREAMER_COMMAND_GET_VOLUME_ATTENUATION,        /*!< argument must be of type @ref uint8_t from 0 to 100                       */
    APOLLO_STREAMER_COMMAND_GET_SOCKET,                    /*!< argument must be of type @ref wiced_udp_socket_t                          */
    APOLLO_STREAMER_COMMAND_GET_STATS,                     /*!< argument must be of type @ref apollo_streamer_stats_t                     */
} apollo_streamer_command_t;

typedef enum
{
    APOLLO_STREAMER_BUF_FLAG_NONE = (0 << 0),
    APOLLO_STREAMER_BUF_FLAG_EOS  = (1 << 0),
} apollo_streamer_buf_flag_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint8_t*        buf;                                   /*!< Pointer to the audio data buffer                                          */
    uint32_t        buflen;                                /*!< Length of audio data buffer                                               */
    uint32_t        offset;                                /*!< Offset in bytes to the start of valid data in the buffer                  */
    uint32_t        curlen;                                /*!< Current length of valid data in the buffer                                */
    uint32_t        flags;                                 /*!< Informational flags; use apollo_streamer_buf_flag_t                       */
    void*           opaque;                                /*!< Opaque pointer - not used by apollo streamer                              */
} apollo_streamer_buf_t;

typedef struct
{
    uint32_t position_msecs;                               /*!< Playback position in milliseconds (relative to track duration below)      */
    uint32_t duration_msecs;                               /*!< Track duration in milliseconds                                            */
} apollo_streamer_track_playback_status_t;

typedef struct
{
    int8_t* artist_utf8_str;                               /*!< NULL-terminated, UTF-8 encoded, track artist string                       */
    int8_t* title_utf8_str;                                /*!< NULL-terminated, UTF-8 encoded, track title string                        */
} apollo_streamer_track_metadata_t;

typedef union
{
    uint8_t                                 volume;        /*!< Audio volume (0 - 100)                                                    */
    apollo_streamer_track_playback_status_t playback;
    apollo_streamer_track_metadata_t        metadata;
    apollo_streamer_buf_t*                  buffer;
} apollo_streamer_event_data_t;

typedef struct
{
    uint64_t     rtp_packets_sent;                         /*!< Number of RTP packets sent                                                */
    uint64_t     total_bytes_sent;                         /*!< Total number of RTP bytes received                                        */
    uint32_t     audio_frames_sent;                        /*!< Number of audio bytes received - excludes RTP header bytes and error correction packets */
    uint16_t     audio_frame_size;                         /*!< Audio frame size in bytes                                                 */
    uint32_t     payload_size;                             /*!< Audio payload size of RTP packets                                         */
    int64_t      media_clock_drift_ppb;                    /*!< Drift in parts per billion of media clock compared to sysclock reference  */
    int64_t      synth_clock_drift_error_current_nsecs;    /*!< Current/instant difference between synthesized timestamps and sysclock    */
    int64_t      synth_clock_drift_error_max_nsecs;        /*!< Maximum recorded difference between synthesized timestamps and sysclock   */
    int64_t      synth_clock_drift_error_min_nsecs;        /*!< Minimum recorded difference between synthesized timestamps and sysclock   */
    uint64_t     audio_frames_played;                      /*!< Audio frames played by audio render                                       */
    uint64_t     audio_frames_dropped;                     /*!< Late audio frames dropped by audio render                                 */
    uint64_t     audio_frames_inserted;                    /*!< Silent audio frames inserted by audio render                              */
    int64_t      audio_max_early;                          /*!< Maximum early time (delta from threshold window) reported by audio render */
    int64_t      audio_max_late;                           /*!< Maximum late time (delta from threshold window) reported by audio render  */
    uint64_t     audio_underruns;                          /*!< Number of audio underruns reported by audio render                        */
} apollo_streamer_stats_t;

typedef struct apollo_streamer_s* apollo_streamer_ref;

typedef int (*apollo_streamer_event_callback)(apollo_streamer_ref handle, void* user_context, apollo_streamer_event_t event, apollo_streamer_event_data_t* event_data);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct apollo_streamer_params_s
{
    apollo_streamer_event_callback event_cb;               /*!< Application event callback                                                */
    void*                          user_context;           /*!< User context for event callback                                           */
    apollo_audio_source_type_t     source_type;            /*!< Audio source type @ref apollo_audio_source_type_t                         */
    wiced_interface_t              iface;                  /*!< network interface @ref wiced_interface_t                                  */
    wiced_ip_address_t             clientaddr;             /*!< IPv4 destination address (can be multicast) @ref wiced_ip_address_t       */
    uint16_t                       port;                   /*!< RTP/UDP port number, enter 0 to use default                               */
    int                            num_pool_packets;       /*!< Total number of packets to be allocated for streamer buffering            */
    int                            num_packets;            /*!< Number of packets streamer may use for audio buffering                    */
    int                            max_payload_size;       /*!< RTP maximum payload size (audio data size)                                */
    int                            burst_length;           /*!< Burst length of protected audio packets                                   */
    int                            shuffle_length;         /*!< SLC shuffle length                                                        */
    int                            slc_send_duplicates;    /*!< Send duplicate packets when SLC is enabled                                */
    int                            audio_compression_type; /*!< audio_compression type                                                    */
    platform_audio_device_id_t     audio_device_rx;        /*!< Input  / capture  device ID @ref platform_audio_device_id_t               */
    platform_audio_device_id_t     audio_device_tx;        /*!< Output / playback device ID @ref platform_audio_device_id_t               */

    /* Audio render playback parameters */

    int                            volume;                 /*!< Audio volume (0 - 100)                                                    */
    uint32_t                       buffer_nodes;           /*!< Number of buffer nodes for audio render to allocate                       */
    uint32_t                       buffer_ms;              /*!< Buffering (pre-roll) time that audio render should use                    */
    uint32_t                       threshold_ns;           /*!< Threshold in nanosecs for adding silence/dropping audio frames            */
    int                            clock_enable;           /*!< 0 = disable (blind push), 1 = enable                                      */

    /* Audio PLL tuning control parameters */

    int                            pll_tuning_enable;      /*!< 0 = disable audio PLL tuning, 1 = enable                                  */
    int                            pll_tuning_ppm_max;     /*!< maximum PPM amount that can be applied to the PLL                         */
    int                            pll_tuning_ppm_min;     /*!< minimum PPM amount that can be applied to the PLL                         */

    /* Input device audio format */

    platform_audio_sample_sizes_t  input_sample_size;      /*!< Input sample size for audio_device_rx                                     */
    platform_audio_sample_rates_t  input_sample_rate;      /*!< Input sample rate for audio_device_rx                                     */
    uint8_t                        input_channel_count;    /*!< Input channel count for audio_device_rx                                   */

} apollo_streamer_params_t;

/*****************************************************************************/
/**
 *
 *  @defgroup multimedia     WICED Multimedia
 *  @ingroup  multimedia
 *
 *  @addtogroup apollostreamer   WICED Apollo Streaming API
 *  @ingroup multimedia
 *
 * This library implements Apollo Streaming Service APIs
 *
 *  @{
 */
/*****************************************************************************/
/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Perform the main initialization and initiate streaming
 *
 * @param[in]  params       : Streaming engine parameters       @ref apollo_streamer_params_t
 * @param[out] streamer_ptr : Pointer to Apollo streamer handle @ref apollo_streamer_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_streamer_init( apollo_streamer_params_t* params, apollo_streamer_ref* streamer_ptr );


/**
 * Destroy and clean-up the audio streamer.
 *
 * @param[in]  streamer     : Apollo streamer handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_streamer_deinit( apollo_streamer_ref streamer );


/** Set the volume for the apollo streamer library.
 *
 * @param[in]  streamer     : Apollo streamer handle
 * @param[in]  volume       : New volume level (0-100).
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_streamer_set_volume(apollo_streamer_ref streamer, int volume);


/** Send an audio configuration, start, stop command with an EXTERNAL audio source
 *  (can also be used with INTERNAL audio sources for a limited number of commands)
 *
 * @param[in]  streamer     : Apollo streamer handle
 * @param[in]  command      : Command @ref apollo_streamer_command_t
 * @param[in]  arg          : Argument pointer to command @ref apollo_streamer_command_t
 * @param[in]  arg_length   : Length of the data associated with the argument
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_streamer_send_command(apollo_streamer_ref streamer, apollo_streamer_command_t command, void* arg, uint32_t arg_length);

#ifdef __cplusplus
} /* extern "C" */
#endif
