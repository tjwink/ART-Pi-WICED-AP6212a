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

#include "protocols/HTTP/http_stream.h"

#include "audio_render.h"

/* FLAC library */
#include <stream_encoder.h>
#include <stream_decoder.h>

#include "flac_app_dct.h"

#include "wiced_flac_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/
/* log levels for the FLAC Library interface */
typedef enum {
    FLAC_LOG_ERROR      = 0,
    FLAC_LOG_INFO       = 1,
    FLAC_LOG_DEBUG      = 2,
} WICED_SSDP_LOG_LEVEL_T;

#define FLAC_APP_PRINT(level, arg)   if ((g_player != NULL) && (g_player->log_level >= level))   WPRINT_LIB_INFO(arg);

/******************************************************
 *                    Constants
 ******************************************************/

#define FLAC_THREAD_STACK_SIZE  8000

#define PLAYER_TAG_VALID                0x51EDBA15
#define PLAYER_TAG_INVALID              0xDEADBEEF

#define FLAC_VOLUME_MIN                   0
#define FLAC_VOLUME_DEFAULT              50
#define FLAC_VOLUME_MAX                 100

/* flac gives us 4096 samples max, 32 bit values, 2 channel, allocate for maximum # bits per channel  */
#define FLAC_APP_AUDIO_RENDER_BITS_PER_SAMPLE   32
#define FLAC_APP_AUDIO_BUFFER_SIZE              (4096 * (FLAC_APP_AUDIO_RENDER_BITS_PER_SAMPLE/8) * 2)
#define FLAC_APP_AUDIO_BUFFER_NODES             20
#define FLAC_APP_AUDIO_BUFFER_MS                50
#define FLAC_APP_AUDIO_THRESH_MS                40
#define FLAC_APP_AUDIO_CLOCK_ENABLE              0

#define FLAC_APP_URI_MAX                1024
#define FLAC_HTTP_QUERY_SIZE            1024

#define HTTP_PORT                       80
#define FLAC_HTTP_THREAD_PRIORITY       (WICED_APPLICATION_PRIORITY)
#define FLAC_HTTP_STACK_SIZE            (6*1024)

/* name of file on server that has the flac play list */
#define FLAC_PLAYLIST_NAME          "/flac_playlist.txt"
#define FLAC_PLAYLIST_ENTRY_MAX     128
/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum {
    FLAC_DECODE_RESOURCE = 0,
    FLAC_DECODE_HTTP
} flac_decoder_stream_source_t;

typedef enum {
    PLAYER_EVENT_SHUTDOWN           = (1 <<  0),

    PLAYER_EVENT_CONNECT            = (1 <<  1),
    PLAYER_EVENT_DISCONNECT         = (1 <<  2),

    PLAYER_EVENT_PLAY               = (1 <<  3),
    PLAYER_EVENT_SKIP               = (1 <<  4),
    PLAYER_EVENT_STOP               = (1 <<  5),
    PLAYER_EVENT_AUTOSTOP           = (1 <<  6),

    PLAYER_EVENT_LIST               = (1 <<  7),
    PLAYER_EVENT_INFO               = (1 <<  8),

    PLAYER_EVENT_LOG_LEVEL_OFF      = (1 << 10),
    PLAYER_EVENT_LOG_LEVEL_INFO     = (1 << 11),
    PLAYER_EVENT_LOG_LEVEL_DEBUG    = (1 << 12),

    PLAYER_EVENT_RELOAD_DCT_WIFI    = (1 << 13),
    PLAYER_EVENT_RELOAD_DCT_NETWORK = (1 << 14),

    PLAYER_EVENT_HTTP_THREAD_DONE   = (1 << 20),

} PLAYER_EVENTS_T;

#define PLAYER_ALL_EVENTS       (-1)

typedef enum {
    PLAYBACK_TYPE_NONE          = 0,
    PLAYBACK_TYPE_ALL           = (1 <<  1),    /* play all the files */
    PLAYBACK_TYPE_LOOP          = (1 <<  0),    /* loop one file - if "all", loop over all the files */

} PLAYBACK_TYPE_T;


/* todo: individual file playback state machine */
typedef enum {
    FLAC_PLAY_STATE_INIT        = 0,
    FLAC_PLAY_STATE_CONNECTING,
    FLAC_PLAY_STATE_CONNECTED,
    FLAC_PLAY_STATE_WAIT_FOR_FLAC_HEADER,
    FLAC_PLAY_STATE_GOT_FLAC_HEADER,
    FLAC_PLAY_STATE_USER_STOP,
    FLAC_PLAY_STATE_DATA_STOP,

} FLAC_PLAY_STATE_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/* audio source informatiuon */
typedef struct flac_packet_info_s {
        wiced_packet_t*                 packet;
        uint32_t                        pkt_offset;     /* currently used offset inside packet */
        uint64_t                        pts;            /* pts at start of this packet (0 if unknown - figure out at render stage by # samples already played out) */
}flac_packet_info_t;

typedef struct flac_source_resource_s {
        const resource_hnd_t*           resource;
        uint32_t                        size;
}flac_source_resource_t;

#define MAX_HTTP_OBJECT_PATH        128
typedef struct flac_source_http_s {
        char*                           uri;
        uint32_t                        size;
        http_stream_t                   http_stream;
}flac_source_http_t;


typedef struct flac_source_info_s {

    flac_decoder_stream_source_t    source_type;
    union {
            flac_source_resource_t      res;
            flac_source_http_t          http;
    }src;
}flac_source_info_t;


/* audio render buffers */
typedef struct audio_buff_list_t_s {
    struct audio_buff_list_t_s*   next;
    uint8_t                     data_buf[FLAC_APP_AUDIO_BUFFER_SIZE];     /* raw LPCM data to send to renderer */
    uint32_t                    data_buf_size;
} audio_buff_list_t;

/* list of songs on server */
typedef struct flac_play_list_s {
    uint8_t                     uri[FLAC_PLAYLIST_ENTRY_MAX + 1];   /* path on server (after http://xxxxx/) */
} flac_play_list_t;

typedef struct flac_player_s {
    uint32_t tag;

    int                             log_level;
    volatile wiced_bool_t           skip_received;      /* skip current song, continue if all or loop */
    volatile wiced_bool_t           stop_received;      /* stop playback */
    volatile wiced_bool_t           peer_closed_socket; /* peer closed socket while streaming */
    volatile wiced_bool_t           finished_playing_out;

    wiced_event_flags_t             events;

    platform_dct_network_config_t*  dct_network;
    platform_dct_wifi_config_t*     dct_wifi;
    flac_app_dct_t*                 dct_app;

    wiced_thread_t                  http_thread;
    volatile wiced_thread_t*        http_thread_ptr;

    wiced_ip_address_t              ip_address;

    /* source info for http streaming */
    flac_decoder_stream_source_t    source_type;
    flac_source_info_t              source;
    char*                           uri_desc;
    char                            uri_to_stream[FLAC_APP_URI_MAX];
    wiced_tcp_socket_t              tcp_socket;
    wiced_bool_t                    tcp_socket_created;
    wiced_bool_t                    tcp_packet_pool_created;

    char                            http_query[FLAC_HTTP_QUERY_SIZE];       /* for building the http query */
    char                            server_uri[FLAC_APP_URI_MAX];
    wiced_bool_t                    connect_state;      /* WICED_TRUE when connected to a server */
    char                            last_connected_host_name[MAX_HTTP_HOST_NAME_SIZE + 1];
    uint16_t                        last_connected_port;
    uint16_t                        server_playlist_count;
    flac_play_list_t*               server_playlist;

    PLAYBACK_TYPE_T                 playback_type;
    uint16_t                        current_play_list_index;

    /*
     * Audio LPCM Buffer management.
     */
    audio_render_ref                audio_render;
    wiced_audio_config_t            curr_audio_config;
    wiced_mutex_t                   audio_buffer_mutex;
    uint16_t                        num_audio_buffer_nodes;
    uint16_t                        num_audio_buffer_used;
    audio_buff_list_t*              audio_buffer_list;
    audio_buff_list_t*              audio_buffer_free_list;

    /* debugging */
    wiced_time_t                    start_time;         /* when the app started */
    wiced_time_t                    play_start_time;    /* when the last play started */



    /* flac internal */
    void*                           internal;

} flac_player_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

extern flac_player_t *g_player;

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
