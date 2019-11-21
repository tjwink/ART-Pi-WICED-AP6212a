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

#ifdef __cplusplus
extern "C" {
#endif

#include "apollocore.h"
#include "platform_audio.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_PLAYER_EVENT_PLAYBACK_STARTED = 0,
    APOLLO_PLAYER_EVENT_PLAYBACK_STOPPED,
    APOLLO_PLAYER_EVENT_SEQ_ERROR,
    APOLLO_PLAYER_EVENT_RTP_TIMING_FULL,
} APOLLO_PLAYER_EVENT_T;

typedef enum
{
    APOLLO_PLAYER_IOCTL_GET_STATS = 0,      /* Arg is pointer to apollo_player_stats_t                          */
    APOLLO_PLAYER_IOCTL_GET_SOCKET,         /* Arg is pointer to wiced_udp_socket_t                             */
    APOLLO_PLAYER_IOCTL_RTP_TIMING_INIT,    /* Arg is pointer to uint32_t - number of log entries to allocate   */
    APOLLO_PLAYER_IOCTL_RTP_TIMING_START,   /* Arg is NULL                                                      */
    APOLLO_PLAYER_IOCTL_RTP_TIMING_STOP,    /* Arg is NULL                                                      */
    APOLLO_PLAYER_IOCTL_RTP_TIMING_RESET,   /* Arg is NULL                                                      */
    APOLLO_PLAYER_IOCTL_RTP_TIMING_GET,     /* Arg is pointer to apollo_player_timing_stats_t                   */

    APOLLO_PLAYER_IOCTL_MAX
} APOLLO_PLAYER_IOCTL_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct apollo_player_s *apollo_player_ref;

typedef int (*apollo_player_event)(apollo_player_ref handle, void* userdata, APOLLO_PLAYER_EVENT_T event, void* arg);

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * Audio format structure passed with APOLLO_PLAYER_EVENT_PLAYBACK_STARTED event.
 */

typedef struct apollo_audio_format_s
{
    uint16_t num_channels;
    uint16_t bits_per_sample;
    uint32_t sample_rate;
} apollo_audio_format_t;

/**
 * Sequence error structure passed with APOLLO_PLAYER_EVENT_SEQ_ERROR event.
 */

typedef struct apollo_seq_err_s
{
    uint16_t last_valid_seq;    /*!< Last valid RTP sequence number received */
    uint16_t cur_seq;           /*!< Current RTP sequence number             */
} apollo_seq_err_t;


/**
 * Stats structure passed with APOLLO_PLAYER_EVENT_PLAYBACK_STOPPED event.
 * Also used with APOLLO_PLAYER_IOCTL_GET_STATS ioctl call.
 */

typedef struct
{
    uint32_t     rtp_packets_received;             /* Number of RTP packets received                                                           */
    uint32_t     rtp_packets_dropped;              /* Number of RTP packets dropped by the network                                             */
    uint64_t     total_bytes_received;             /* Total number of RTP bytes received                                                       */
    uint64_t     audio_bytes_received;             /* Number of audio bytes received - excludes RTP header bytes and error correction packets  */
    uint32_t     payload_size;                     /* Audio payload size of RTP packets                                                        */
    uint64_t     audio_frames_played;              /* Audio frames played by audio render                                                      */
    uint64_t     audio_frames_dropped;             /* Late audio frames dropped by audio render                                                */
    uint64_t     audio_frames_inserted;            /* Silent audio frames inserted by audio render                                             */
    int64_t      audio_max_early;                  /* Maximum early time (delta from threshold window) reported by audio render                */
    int64_t      audio_max_late;                   /* Maximum late time (delta from threshold window) reported by audio render                 */
    uint64_t     audio_underruns;                  /* Number of audio underruns reported by audio render                                       */
    uint64_t     audio_concealment_slcplc_cnt;     /* SLC/PLC concealment event counts count                                                   */
    uint64_t     audio_concealment_fec_cnt;        /* FEC     concealment event counts count                                                   */

} apollo_player_stats_t;


/**
 * RTP packet arrival time debug logging structures.
 * Timing stats information retrieved with APOLLO_PLAYER_IOCTL_RTP_TIMING_GET ioctl call.
 */

typedef struct
{
    uint16_t sequence_number;
    uint64_t timestamp;
} rtp_arrival_time_t;


typedef struct
{
    uint32_t            number_entries;
    rtp_arrival_time_t* rtp_entries;
} apollo_player_timing_stats_t;


typedef struct apollo_player_params_s
{

    apollo_player_event     event_cb;       /* Application event callback                               */
    void*                   userdata;       /* Opaque userdata passed back in event callback            */

    wiced_interface_t       interface;      /* Interface to use for RTP socket                          */
    int                     rtp_port;       /* RTP port for socket - 0 to use default port              */
    APOLLO_CHANNEL_MAP_T    channel;        /* Audio channel to process                                 */
    int                     volume;         /* Audio volume (0 - 100)                                   */

    /* Audio render playback parameters */

    platform_audio_device_id_t  device_id;      /* Audio device ID for audio playback                       */
    uint32_t                    buffer_nodes;   /* Number of buffer nodes for audio render to allocate      */
    uint32_t                    buffer_ms;      /* Buffering (pre-roll) time that audio render should use   */
    uint32_t                    threshold_ns;   /* Threshold in ns for adding silence/dropping audio frames */
    int                         clock_enable;   /* 0 = disable (blind push), 1 = enable                     */

    /* Audio PLL tuning control parameters */

    int                         pll_tuning_enable;  /* 0 = disable audio PLL tuning, 1 = enable             */

} apollo_player_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/
/*****************************************************************************/
/**
 *
 *  @defgroup multimedia     WICED Multimedia
 *  @ingroup  multimedia
 *
 *  @addtogroup apolloplayer   WICED Apollo Player API
 *  @ingroup multimedia
 *
 * This library implements Apollo Player Service APIs
 *
 *  @{
 */
/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize the apollo player library.
 *
 * @param[in] params      : Pointer to the configuration parameters.
 *
 * @return Pointer to the apollo player instance or NULL
 */
apollo_player_ref apollo_player_init(apollo_player_params_t* params);

/** Deinitialize the apollo player library.
 *
 * @param[in] apollo_player : Pointer to the apollo player instance.
 *
 * @return    Status of the operation.
 */
wiced_result_t apollo_player_deinit(apollo_player_ref apollo_player);

/** Set the volume for the apollo player library.
 *
 * @param[in] apollo_player : Pointer to the apollo player instance.
 * @param[in] volume        : New volume level (0-100).
 *
 * @return    Status of the operation.
 */
wiced_result_t apollo_player_set_volume(apollo_player_ref apollo_player, int volume);

/** Send an IOCTL to the apollo player session.
 *
 * @param[in] apollo_player : Pointer to the apollo player instance.
 * @param[in] cmd           : IOCTL command to process.
 * @param[inout] arg        : Pointer to argument for IOTCL.
 *
 * @return    Status of the operation.
 */
wiced_result_t apollo_player_ioctl(apollo_player_ref apollo_player, APOLLO_PLAYER_IOCTL_T cmd, void* arg);

#ifdef __cplusplus
} /* extern "C" */
#endif
