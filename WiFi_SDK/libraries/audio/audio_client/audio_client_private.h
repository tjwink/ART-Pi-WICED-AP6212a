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
 * @file
 *
 * File Audio Client Internal Definitions
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "wiced_time.h"
#include "audio_client.h"

#include "audio_render.h"
#include "bufmgr.h"
#include "linked_list.h"
#include "audio_client_hls_types.h"

/******************************************************
 *                     Macros
 ******************************************************/

#define CHECK_FOR_THRESHOLD_HIGH_EVENT(client)                                          \
    do                                                                                  \
    {                                                                                   \
        if (!client->threshold_high_sent && client->params.data_threshold_high > 0)     \
        {                                                                               \
            int in_use;                                                                 \
                                                                                        \
            in_use = client->data_buf_widx - client->data_buf_ridx;                     \
            if (in_use < 0)                                                             \
            {                                                                           \
                in_use += client->params.data_buffer_num;                               \
            }                                                                           \
            if (in_use >= client->params.data_threshold_high)                           \
            {                                                                           \
                client->threshold_high_sent = WICED_TRUE;                               \
                client->threshold_low_sent  = WICED_FALSE;                              \
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH, NULL); \
            }                                                                           \
        }                                                                               \
    } while (0)

#define CHECK_FOR_THRESHOLD_LOW_EVENT(client)                                           \
    do                                                                                  \
    {                                                                                   \
        if (!client->threshold_low_sent && client->params.data_threshold_high > 0)      \
        {                                                                               \
            int in_use;                                                                 \
                                                                                        \
            in_use = client->data_buf_widx - client->data_buf_ridx;                     \
            if (in_use < 0)                                                             \
            {                                                                           \
                in_use += client->params.data_buffer_num;                               \
            }                                                                           \
            if (in_use <= client->params.data_threshold_low && client->http_params.http_content_read < client->http_params.http_content_length) \
            {                                                                           \
                client->threshold_high_sent = WICED_FALSE;                              \
                client->threshold_low_sent  = WICED_TRUE;                               \
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW, NULL); \
                if (client->params.high_threshold_read_inhibit)                         \
                {                                                                       \
                    wiced_rtos_set_event_flags(&client->http_params.http_events, HTTP_EVENT_TCP_DATA); \
                }                                                                       \
            }                                                                           \
        }                                                                               \
    } while (0)

#define RELEASE_DBUF(client, dbuf)                                                                  \
    do                                                                                              \
    {                                                                                               \
        if (dbuf->inuse)                                                                            \
        {                                                                                           \
            dbuf->buflen  = 0;                                                                      \
            dbuf->bufused = 0;                                                                      \
            dbuf->inuse   = 0;                                                                      \
            client->data_buf_ridx = (client->data_buf_ridx + 1) % client->params.data_buffer_num;   \
        }                                                                                           \
    } while (0)


/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_CLIENT_TAG_VALID              0x05EAf00D
#define AUDIO_CLIENT_TAG_INVALID            0xDEADBEEF

#define AUDIO_CLIENT_THREAD_PRIORITY        5
#define AUDIO_CLIENT_THREAD_STACK_SIZE      (2 * 4096)

#define AUDIO_CLIENT_DEFAULT_HTTP_PORT      (80)
#define AUDIO_CLIENT_DEFAULT_HTTPS_PORT     (443)

#define AUDIO_CLIENT_HOSTNAME_LEN           (80)
#define AUDIO_CLIENT_PATH_LEN               (768)

#define AUDIO_CLIENT_HTTP_BUF_SIZE          (2048)

#define AUDIO_CLIENT_DATA_BUF_SIZE          (1500)

#define AUDIO_CLIENT_CONTENT_TYPE_SIZE      (128)

#define AUDIO_CLIENT_AUDIO_BUF_TIMEOUT      (1)

#define AUDIO_CLIENT_DEFAULT_LOAD_FILE_SIZE (4096)

#define AUDIO_CLIENT_HLS_LINE_BUFFER        (AUDIO_CLIENT_DATA_BUF_SIZE)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_CLIENT_STATE_IDLE = 0,
    AUDIO_CLIENT_STATE_PLAY,
    AUDIO_CLIENT_STATE_PAUSE,

} AUDIO_CLIENT_STATE_T;

typedef enum
{
    AUDIO_CLIENT_EVENT_SHUTDOWN             = (1 <<  0),
    AUDIO_CLIENT_EVENT_PLAY                 = (1 <<  1),
    AUDIO_CLIENT_EVENT_STOP                 = (1 <<  2),
    AUDIO_CLIENT_EVENT_PAUSE                = (1 <<  3),
    AUDIO_CLIENT_EVENT_RESUME               = (1 <<  4),
    AUDIO_CLIENT_EVENT_VOLUME               = (1 <<  5),
    AUDIO_CLIENT_EVENT_SEEK                 = (1 <<  6),
    AUDIO_CLIENT_EVENT_EFFECT               = (1 <<  7),
    AUDIO_CLIENT_SUSPEND                    = (1 <<  8),

    AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE = (1 << 15),

    AUDIO_CLIENT_EVENT_HTTP_ERROR           = (1 << 19),
    AUDIO_CLIENT_EVENT_HTTP_THREAD_DONE     = (1 << 20),
    AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE  = (1 << 21),
    AUDIO_CLIENT_EVENT_PLAYBACK_COMPLETE    = (1 << 22),

    AUDIO_CLIENT_EVENT_SUSPEND_COMPLETE     = (1 << 31)
} AUDIO_CLIENT_EVENTS_T;

#define AUDIO_CLIENT_ALL_EVENTS       (~(AUDIO_CLIENT_EVENT_SUSPEND_COMPLETE))

typedef enum
{
    HTTP_EVENT_TCP_DATA             = (1 <<  0),
    HTTP_EVENT_SEEK                 = (1 <<  1),
    HTTP_EVENT_CONNECT              = (1 <<  2),
    HTTP_EVENT_DISCONNECT           = (1 <<  3),
    HTTP_EVENT_TIMER                = (1 <<  4),
} HTTP_EVENTS_T;

#define HTTP_ALL_EVENTS       (-1)

typedef enum
{
    DECODER_EVENT_AUDIO_DATA        = (1 <<  0),
    DECODER_EVENT_FLUSH             = (1 <<  1),

    DECODER_EVENT_FLUSH_COMPLETE    = (1 <<  31),
} DECODER_EVENTS_T;

#define DECODER_ALL_EVENTS       (~(DECODER_EVENT_FLUSH_COMPLETE))

typedef enum
{
    DECODER_IOCTL_INFO = 0,             /* Arg is pointer to audio_client_stream_info_t                     */
    DECODER_IOCTL_GET_SEEK_POSITION,    /* Arg in is pointer to uint32_t (time in ms), out is byte position */
    DECODER_IOCTL_SET_POSITION,         /* Arg is new stream position in bytes                              */
    DECODER_IOCTL_FLUSH,                /* Arg is NULL                                                      */

    DECODER_IOCTL_MAX
} DECODER_IOCTL_T;

typedef enum
{
    AUDIO_CLIENT_CONTAINER_NULL,
    AUDIO_CLIENT_CONTAINER_TS,

    AUDIO_CLIENT_CONTAINER_MAX
} AUDIO_CLIENT_CONTAINER_T;

typedef enum
{
    FILTER_IOCTL_PROCESS_DATA = 0,      /* Arg is pointer to audio_client_filter_buffer_t                   */
    FILTER_IOCTL_RESET_STATE,           /* No arg                                                           */
} FILTER_IOCTL_T;

typedef enum
{
    STREAM_URI_STORAGE_TYPE_PLAY_REQUEST = 0,
    STREAM_URI_STORAGE_TYPE_HLS_NODE,
} STREAM_URI_STORAGE_TYPE_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    char* name;
    uint32_t value;
} lookup_t;

typedef struct audio_client_play_request_s
{
    struct audio_client_play_request_s* next;
    char*                               stream_uri;
    uint32_t                            start_offset_ms;
    wiced_bool_t                        load_file;
    wiced_bool_t                        contiguous;
    AUDIO_CLIENT_CODEC_T                push_audio_codec;
    char                                data[0];
} audio_client_play_request_t;


typedef struct
{
    uint64_t        stream_current_sample;  /* last played sample                   */
    uint64_t        stream_total_samples;   /* total samples in stream (if known)   */
    uint32_t        stream_sample_rate;     /* sample rate                          */
    uint8_t         stream_channels;        /* number of channels in source stream  */
    uint8_t         stream_bps;             /* bits per sample in source stream     */
    uint32_t        bit_rate;               /* bitrate of source stream             */
} audio_client_stream_info_t;


typedef struct
{
    volatile uint8_t     inuse;
    volatile uint16_t    buflen;
    uint16_t    bufused;
    uint8_t     buf[AUDIO_CLIENT_DATA_BUF_SIZE];
} data_buf_t;


struct audio_client_s;

typedef wiced_result_t (*audio_client_decoder_start_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_decoder_stop_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_decoder_ioctl_t)(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg);

typedef struct
{
    audio_client_decoder_start_t            decoder_start;
    audio_client_decoder_stop_t             decoder_stop;
    audio_client_decoder_ioctl_t            decoder_ioctl;
} audio_client_decoder_api_t;

typedef wiced_result_t (*audio_client_filter_create_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_filter_destroy_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_filter_ioctl_t)(struct audio_client_s* client, FILTER_IOCTL_T ioctl, void* arg);

typedef struct
{
    uint8_t* input_buffer;
    uint32_t input_buffer_length;
    uint8_t* output_buffer;
    uint32_t output_buffer_length;
} audio_client_filter_buffer_t;

typedef struct
{
    audio_client_filter_create_t            filter_create;
    audio_client_filter_destroy_t           filter_destroy;
    audio_client_filter_ioctl_t             filter_ioctl;
} audio_client_filter_api_t;

struct audio_client_s;

typedef struct
{
    struct audio_client_s*      client;

    wiced_tcp_socket_t          socket;
    wiced_tcp_socket_t*         socket_ptr;

    char*                       stream_uri;
    STREAM_URI_STORAGE_TYPE_T   stream_uri_storage;

    char                        hostname[AUDIO_CLIENT_HOSTNAME_LEN];
    char                        path[AUDIO_CLIENT_PATH_LEN];
    int                         port;
    char                        http_buf[AUDIO_CLIENT_HTTP_BUF_SIZE];           /* HTTP temporary buffer */
    int                         http_buf_idx;
    char                        http_content_type[AUDIO_CLIENT_CONTENT_TYPE_SIZE];

    wiced_thread_t              http_thread;
    wiced_thread_t*             http_thread_ptr;
    wiced_event_flags_t         http_events;
    wiced_timer_t               http_timer;
    wiced_time_t                http_last_packet_time;

    wiced_bool_t                http_done;
    wiced_bool_t                http_error;
    wiced_bool_t                http_redirect;
    wiced_bool_t                http_need_header;
    wiced_bool_t                http_need_extra_header_check;
    wiced_bool_t                http_in_header;
    wiced_bool_t                http_icy_header;
    uint8_t                     http_header_idx;
    uint8_t                     http_body_idx;
    wiced_bool_t                http_transfer_encoding_chunked;                 /* Server using chunked transfer encoding   */
    wiced_bool_t                http_range_requests;                            /* Server supports range requests           */
    uint32_t                    http_total_content_length ;                     /* The total length of media file           */
    uint32_t                    http_content_length;                            /* The content length of the http           */
    uint32_t                    http_content_read;
    uint32_t                    http_chunk_size;
    uint32_t                    http_chunk_bytes;
    uint32_t                    http_chunk_skip;
    uint32_t                    packets_read;
    uint32_t                    initial_buffer_count;

    wiced_bool_t                http_load_file;
    uint8_t*                    http_file_data;
    uint32_t                    http_file_idx;

} audio_client_http_params_t;

typedef struct audio_client_s
{
    uint32_t                    tag;
    AUDIO_CLIENT_STATE_T        state;
    wiced_bool_t                stop_in_progress;

    wiced_thread_t              client_thread;
    wiced_thread_t*             client_thread_ptr;

    audio_client_params_t       params;
    int                         new_volume;

    wiced_event_flags_t         events;

    audio_client_play_request_t* play_queue;
    audio_client_play_request_t* current_play;
    audio_client_play_request_t* post_decode;
    wiced_mutex_t               play_mutex;

    wiced_bool_t                accept_push_buffer;
    wiced_bool_t                accept_raw_buffer;
    wiced_bool_t                raw_mono_to_stereo;

    AUDIO_CLIENT_CODEC_T        audio_codec;        /* which codec are we playing? */
    wiced_time_t                play_start_time;    /* when the last play started */
    uint32_t                    start_offset_ms;

    wiced_bool_t                seek_in_progress;
    uint32_t                    seek_request_ms;
    uint32_t                    seek_position;

    uint32_t                    effect_mode;

    /*
     * HTTP reader thread context
     */

    audio_client_http_params_t  http_params;

    /*
     * HLS playlist reload thread context
     */

    audio_client_http_params_t  hls_params;

    /*
     * Received data buffers.
     */

    data_buf_t                  *data_bufs;
    int                         data_buf_widx;
    int                         data_buf_ridx;

    wiced_bool_t                threshold_high_sent;                            /* Threshold high event sent    */
    wiced_bool_t                threshold_low_sent;                             /* Threshold low event sent     */

    /*
     * Decoder information.
     */

    audio_client_decoder_api_t  decoders[AUDIO_CLIENT_CODEC_MAX];
    wiced_bool_t                decoder_done;
    wiced_event_flags_t         decoder_events;

    wiced_thread_t              decoder_thread;
    wiced_thread_t*             decoder_thread_ptr;

    /*
     * Audio render information.
     */

    audio_render_ref            audio;
    objhandle_t                 buf_pool;
    wiced_bool_t                audio_configured;
    wiced_bool_t                audio_pushed;

    /*
     * Handle for decoder specific information.
     */

    void*                       decoder_handle;
    wiced_bool_t                decoder_first_kick;

    /*
     * Handle for media filters/demuxers/parsers
     */

    AUDIO_CLIENT_CONTAINER_T    filter_type;
    audio_client_filter_api_t   filters[AUDIO_CLIENT_CONTAINER_MAX];
    char                        filter_content_type[AUDIO_CLIENT_CONTENT_TYPE_SIZE];
    void*                       filter_handle;

    /*
     * HLS handling
     */

    wiced_bool_t                     hls_playlist_active;
    wiced_bool_t                     hls_playlist_parsing_complete;
    wiced_bool_t                     hls_playlist_is_live;
    wiced_bool_t                     hls_playlist_last_entry;
    wiced_bool_t                     hls_playlist_is_reloading;
    wiced_bool_t                     hls_playlist_done;
    wiced_bool_t                     hls_playlist_needs_reload;
    wiced_bool_t                     hls_do_not_store_node;
    linked_list_t                    hls_list_playlist;
    linked_list_t                    hls_list_media;
    linked_list_t                    hls_list_media_reload;
    uint32_t                         hls_base_uri_length;
    uint32_t                         hls_entry_number;
    uint32_t                         hls_playlist_count;
    uint32_t                         hls_segment_count;
    uint32_t                         hls_entry_number_reload;
    linked_list_node_t*              hls_node_current;
    linked_list_node_t*              hls_node_previous;
    uint32_t                         hls_start_offset_ms;
    uint32_t                         hls_target_duration_ms;
    wiced_time_t                     hls_playlist_load_ts;
    wiced_event_flags_t              hls_wait_flag;
    char*                            hls_playlist_stream_uri;
    uint8_t                          hls_line_buffer[AUDIO_CLIENT_HLS_LINE_BUFFER];
    uint32_t                         hls_line_buffer_index;
    audio_client_hls_playlist_info_t hls_playlist_node_info;
    AUDIO_CLIENT_HLS_NODE_TYPE_T     hls_node_type;

    /*
     * Suspend information
     */

    audio_client_suspend_t*     suspend;

    /*
     * Audio client thread stack.
     */

    uint8_t                     client_thread_stack_buffer[AUDIO_CLIENT_THREAD_STACK_SIZE];

} audio_client_t;


/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

extern lookup_t mime_types[];

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t audio_client_audio_config(audio_client_t* client, wiced_audio_config_t* config);
wiced_result_t audio_client_buffer_get(audio_client_t* client, audio_client_buf_t* buf, uint32_t timeout_ms);
wiced_result_t audio_client_buffer_release(audio_client_t* client, audio_client_buf_t* buf);
wiced_bool_t audio_client_check_start_offset_seek(audio_client_t* client);

wiced_result_t audio_client_http_reader_stop(audio_client_t* client, audio_client_http_params_t* params);
wiced_result_t audio_client_http_reader_start(audio_client_t* client, audio_client_http_params_t* params);

void           write_data_buf(audio_client_t* client, audio_client_http_params_t* params, data_buf_t* dbuf, uint8_t* data, uint16_t data_length);

#ifdef __cplusplus
} /*extern "C" */
#endif
