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

#include "wiced.h"
#include "wiced_log.h"
#include "button_manager.h"
#include "apollo_config.h"
#include "apollo_player.h"
#include "apollo_streamer.h"
#include "apollocore.h"
#include "apollo_cmd.h"
#include "apollo_cmd_sender.h"

#ifdef USE_UPNPAV
#include "upnp_av_render.h"
#include "bufmgr.h"
#endif

#if defined(OTA2_SUPPORT)
#include "apollo_ota2_context.h"
#endif

#if defined(USE_AMBILIGHT)
#include "amblt_source.h"
#include "amblt_sink.h"
#include "amblt_back.h"
#include "amblt_front.h"
#include "amblt_bus.h"
#include "amblt_drv.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_TAG_VALID                      (0xCA11AB1E)
#define APOLLO_TAG_INVALID                    (0xDEADBEEF)


#if defined(USE_AMBILIGHT)
#define APOLLO_CONSOLE_COMMAND_MAX_LENGTH     (255)
#define APOLLO_CONSOLE_COMMAND_MAX_PARAMS     (96)
#else
#define APOLLO_CONSOLE_COMMAND_MAX_LENGTH     (85)
#define APOLLO_CONSOLE_COMMAND_MAX_PARAMS     (32)
#endif

#define APOLLO_CONSOLE_COMMAND_HISTORY_LENGTH (10)

#ifdef USE_UPNPAV
#define APOLLO_TX_PACKET_BUFFER_COUNT         (48)
#else  /* !USE_UPNPAV */
#define APOLLO_TX_PACKET_BUFFER_COUNT         (128)
#endif /* USE_UPNPAV */
#define APOLLO_TX_AUDIO_RENDER_BUFFER_NODES   (200)
#define APOLLO_RX_AUDIO_RENDER_BUFFER_NODES   (128)

#define BUTTON_WORKER_STACK_SIZE              ( 4096 )
#define BUTTON_WORKER_QUEUE_SIZE              ( 4 )

#define NETWORK_INIT_TIMER_PERIOD_MSECS       (15000)

#define PLAYBACK_TIMER_PERIOD_MSECS           (1000)
#define PLAYBACK_TIMER_TIMEOUT_MSECS          (4 * 1000)

#define SECONDS_PER_MINUTE                    (60)
#define SECONDS_PER_HOUR                      (SECONDS_PER_MINUTE * 60)
#define SECONDS_PER_DAY                       (SECONDS_PER_HOUR * 24)   //86400

#define UPNPAV_FRIENDLY_NAME_STR_LENGTH       (36)
#define UPNPAV_UUID_STR_LENGTH                (36)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_EVENT_NONE                 = 0,
    APOLLO_EVENT_SHUTDOWN             = (1 << 0),
    APOLLO_EVENT_START                = (1 << 1),
    APOLLO_EVENT_STOP                 = (1 << 2),
    APOLLO_EVENT_AUTOSTOP             = (1 << 3),
    APOLLO_EVENT_CONFIG_GATT          = (1 << 4),
    APOLLO_EVENT_RTP_TIMING           = (1 << 5),
    APOLLO_EVENT_TIMER                = (1 << 6),
    APOLLO_EVENT_NETWORK_TIMER        = (1 << 7),

    APOLLO_EVENT_RELOAD_DCT_WIFI      = (1 << 8),
    APOLLO_EVENT_RELOAD_DCT_NETWORK   = (1 << 9),
    APOLLO_EVENT_RELOAD_DCT_BLUETOOTH = (1 << 10),

    APOLLO_EVENT_RMC_DOWN             = (1 << 11),
    APOLLO_EVENT_RMC_UP               = (1 << 12),

    APOLLO_EVENT_RX_CSA               = (1 << 13),

    APOLLO_EVENT_VOLUME_UP            = (1 << 14),
    APOLLO_EVENT_VOLUME_DOWN          = (1 << 15),

    APOLLO_EVENT_PTP_PACKET           = (1 << 16),

#if defined(OTA2_SUPPORT)
    APOLLO_EVENT_GET_UPDATE           = (1 << 20),
    APOLLO_EVENT_START_TIMED_UPDATE   = (1 << 21),
    APOLLO_EVENT_STOP_TIMED_UPDATE    = (1 << 22),
    APOLLO_EVENT_OTA2_STATUS          = (1 << 23),
    APOLLO_EVENT_OTA2_UPDATE          = (1 << 24),
#endif  /* defined(OTA2_SUPPORT) */

#if defined(USE_AMBILIGHT)
    APOLLO_EVENT_AMBILIGHT            = (1 << 25),
#endif

} APOLLO_EVENTS_T;

#define APOLLO_ALL_EVENTS       ((uint32_t)-1)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t                    tag;
    int                         stop_received;
    wiced_bool_t                initializing;
    wiced_bool_t                playback_active;
    wiced_bool_t                network_logging_active;

    wiced_time_t                playback_ended;

    wiced_event_flags_t         events;
    wiced_timer_t               timer;

    apollo_dct_collection_t     dct_tables;
    apollo_streamer_params_t    streamer_params;

    wiced_ip_address_t          mrtp_ip_address;

    wiced_bool_t                rmc_network_up;
    wiced_bool_t                rmc_network_coming_up;      /* TRUE while network is coming up */
    wiced_timer_t               network_timer;
    wiced_bool_t                network_timer_ignore;       /* if WICED_TRUE, ignore timer events - timer will continue */
    wiced_interface_t           interface;
    wiced_interface_t           upstream_interface;
    wiced_bool_t                upstream_interface_valid;
    wiced_udp_socket_t          report_socket;
    wiced_udp_socket_t*         report_socket_ptr;
    wiced_mac_t                 mac_address;                        /* Our mac address */

    apollo_player_ref           player_handle;
    apollo_streamer_ref         streamer_handle;
    void*                       cmd_handle;
    void*                       cmd_sender_handle;

    button_manager_t            button_manager;
    wiced_worker_thread_t       button_worker_thread;
    wiced_bool_t                button_gatt_launch_was_pressed;

    APOLLO_CMD_RTP_TIMING_T     rtp_timing_cmd;
    uint32_t                    rtp_timing_entries;

    uint64_t                    total_rtp_packets_received;
    uint64_t                    total_rtp_packets_dropped;
    uint64_t                    total_audio_frames_played;
    uint64_t                    total_audio_frames_dropped;
    uint64_t                    total_audio_frames_inserted;
    uint64_t                    total_audio_underruns;
    uint64_t                    total_audio_concealment_slcplc_cnt;
    uint64_t                    total_audio_concealment_fec_cnt;

    wiced_udp_socket_t          csa_socket;
    wiced_udp_socket_t*         csa_socket_ptr;

    /* needed to serialize display & ambilight bus access */
    wiced_mutex_t               i2c_bus_mtx;

#ifdef USE_AUDIO_DISPLAY
    wiced_thread_t display_thread;
    apollo_player_stats_t player_stats;
    char display_name[32];
    char display_info[32]; /* includes channel info */
#endif

#ifdef USE_UPNPAV
    upnpavrender_service_ref    upnpavrender_handle;
    objhandle_t                 upnpavrender_buf_pool;
    wiced_bool_t                upnpavrender_mute_enabled;
    uint8_t                     upnpavrender_volume;
    wiced_bool_t                upnpavrender_paused;
    char                        upnpavrender_friendly_name[UPNPAV_FRIENDLY_NAME_STR_LENGTH + 1];
    char                        upnpavrender_uuid[UPNPAV_UUID_STR_LENGTH + 1];
#endif

#if defined(OTA2_SUPPORT)
    apollo_ota2_service_info_t  ota2_info;
#endif

#if defined(USE_AMBILIGHT)
    /* hw params */
    amblt_drv_psoc_param_t      amblt_psoc_hw;
    amblt_bus_i2c_param_t       amblt_i2c_hw;

    /* sw components : source */
    amblt_front_handle_t        amblt_front;
    amblt_source_handle_t       amblt_source;

    /* sw components : sink */
    amblt_sink_handle_t         amblt_sink;
    AMBLT_BACK_HANDLE           amblt_back;
#endif

} apollo_app_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/*
 * That's only an extern variable because of the button and console managers
 * not allowing API users to provide their own context variable
 */
extern apollo_app_t *g_apollo;

/******************************************************
 *               Function Declarations
 ******************************************************/

int apollo_log_output_handler(WICED_LOG_LEVEL_T level, char *logmsg);

wiced_result_t apollo_console_start(apollo_app_t *apollo);
wiced_result_t apollo_console_stop(apollo_app_t *apollo);

#ifdef __cplusplus
} /* extern "C" */
#endif
