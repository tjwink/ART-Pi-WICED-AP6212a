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

/** @file
 *
 * Audio Player Application
 *
 */

#include <ctype.h>
#include <unistd.h>
#include <malloc.h>

#include "wiced.h"
#include "command_console.h"
#include "wifi/command_console_wifi.h"
#include "dct/command_console_dct.h"

#include "dns.h"

#include "bufmgr.h"
#include "wiced_log.h"
#include "wiced_audio.h"
#include "audio_player_config.h"
#include "audio_player_tracex.h"

#include "audio_client.h"
#include "audio_render.h"
#include "audio_client_private.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define PRINT_IP(addr)    (int)((addr.ip.v4 >> 24) & 0xFF), (int)((addr.ip.v4 >> 16) & 0xFF), (int)((addr.ip.v4 >> 8) & 0xFF), (int)(addr.ip.v4 & 0xFF)

#define AUDIO_PLAYER_CONSOLE_COMMANDS \
    { (char*) "exit",           audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Exit application" }, \
    { (char*) "config",         audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Display / change config values" }, \
    { (char*) "log",            audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Set the current log level" }, \
    { (char*) "play",           audio_player_console_command,    1, NULL, NULL, (char *)"",                     (char *)"Play a track" }, \
    { (char*) "stop",           audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Stop playing a track" }, \
    { (char*) "pause",          audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Pause playing a track" }, \
    { (char*) "resume",         audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Resume playing a track" }, \
    { (char*) "volume",         audio_player_console_command,    1, NULL, NULL, (char *)"<volume>",             (char *)"Change the playback volume" }, \
    { (char*) "load_playlist",  audio_player_console_command,    1, NULL, NULL, (char *)"<ipaddr>",             (char *)"Load an audio playlist from a server" }, \
    { (char*) "playlist",       audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Display the current audio playlist" }, \
    { (char*) "clear",          audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Clear the current audio playlist" }, \
    { (char*) "stats",          audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Display playback stats" }, \
    { (char*) "seek",           audio_player_console_command,    1, NULL, NULL, (char *)"<ms>",                 (char *)"Seek to the requested time (ms)" }, \
    { (char*) "memory",         audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Display the current system memory info" }, \
    { (char*) "effect",         audio_player_console_command,    1, NULL, NULL, (char *)"<index>",              (char *)"Set audio effect" }, \
    { (char*) "play_offset",    audio_player_console_command,    2, NULL, NULL, (char *)"<track> <offset_ms>",  (char *)"Play playlist track at a specified offset" }, \
    { (char*) "playc",          audio_player_console_command,    1, NULL, NULL, (char *)"<track>",              (char *)"Play a contiguous track" }, \
    { (char*) "suspend",        audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Suspend playback" }, \
    { (char*) "suspend_resume", audio_player_console_command,    0, NULL, NULL, (char *)"",                     (char *)"Resume suspended playback" }, \
    { (char*) "play_raw",       audio_player_console_command,    2, NULL, NULL, (char *)"<sample rate> <ch>",   (char *)"Play a raw file" }, \


/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_PLAYER_CONSOLE_COMMAND_MAX_LENGTH      (120)
#define AUDIO_PLAYER_CONSOLE_COMMAND_HISTORY_LENGTH  (10)

#define AUDIO_PLAYER_TAG_VALID              0x51EDBA15
#define AUDIO_PLAYER_TAG_INVALID            0xDEADBEEF

#define SERVER_URI_MAX_LEN                  (128)
#define WORK_BUFFER_LEN                     (1024)

#define AUDIO_PLAYER_DEFAULT_HTTP_PORT      (80)
#define AUDIO_PLAYER_DEFAULT_HTTPS_PORT     (443)

#define AUDIO_PLAYER_PLAYLIST_NAME          "/audio_playlist.txt"

#define AUDIO_EFFECT_MODE_NONE              (0)

#ifndef AUDIO_EFFECT_MODE_MAX
#define AUDIO_EFFECT_MODE_MAX               AUDIO_EFFECT_MODE_NONE
#endif

#define TCP_DEFAULT_LISTEN_PORT     (19703)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_PLAYER_CONSOLE_CMD_EXIT = 0,
    AUDIO_PLAYER_CONSOLE_CMD_CONFIG,
    AUDIO_PLAYER_CONSOLE_CMD_LOG,
    AUDIO_PLAYER_CONSOLE_CMD_PLAY,
    AUDIO_PLAYER_CONSOLE_CMD_STOP,
    AUDIO_PLAYER_CONSOLE_CMD_PAUSE,
    AUDIO_PLAYER_CONSOLE_CMD_RESUME,
    AUDIO_PLAYER_CONSOLE_CMD_VOLUME,
    AUDIO_PLAYER_CONSOLE_CMD_LOAD_PLAYLIST,
    AUDIO_PLAYER_CONSOLE_CMD_PLAYLIST,
    AUDIO_PLAYER_CONSOLE_CMD_CLEAR,
    AUDIO_PLAYER_CONSOLE_CMD_STATS,
    AUDIO_PLAYER_CONSOLE_CMD_SEEK,
    AUDIO_PLAYER_CONSOLE_CMD_MEMORY,
    AUDIO_PLAYER_CONSOLE_CMD_EFFECT,
    AUDIO_PLAYER_CONSOLE_CMD_PLAY_OFFSET,
    AUDIO_PLAYER_CONSOLE_CMD_PLAY_CONTIGUOUS,
    AUDIO_PLAYER_CONSOLE_CMD_SUSPEND,
    AUDIO_PLAYER_CONSOLE_CMD_SUSPEND_RESUME,
    AUDIO_PLAYER_CONSOLE_CMD_PLAY_RAW,

    AUDIO_PLAYER_CONSOLE_CMD_MAX,
} AUDIO_PLAYER_CONSOLE_CMDS_T;

typedef enum
{
    PLAYER_EVENT_SHUTDOWN               = (1 <<  0),
    PLAYER_EVENT_LOAD_PLAYLIST          = (1 <<  1),
    PLAYER_EVENT_PLAYLIST_LOADED        = (1 <<  2),
    PLAYER_EVENT_OUTPUT_PLAYLIST        = (1 <<  3),
    PLAYER_EVENT_PLAY                   = (1 <<  4),
    PLAYER_EVENT_PLAYBACK_EOS           = (1 <<  5),
    PLAYER_EVENT_EFFECT                 = (1 <<  6),
    PLAYER_EVENT_PLAY_RAW               = (1 <<  7),
    PLAYER_EVENT_CONNECT                = (1 <<  8),
    PLAYER_EVENT_DISCONNECT             = (1 <<  9),
    PLAYER_EVENT_PACKET                 = (1 << 10),

    PLAYER_EVENT_RELOAD_DCT_WIFI        = (1 << 30),
    PLAYER_EVENT_RELOAD_DCT_NETWORK     = (1 << 31),
} PLAYER_EVENTS_T;

#define PLAYER_ALL_EVENTS       (-1)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    char *cmd;
    uint32_t event;
} cmd_lookup_t;


typedef struct
{
    int buflen;
    int maxlen;
    objhandle_t buf_handle;
    uint8_t buf[0];
} audio_buf_t;


typedef struct audio_player_s
{
    uint32_t                        tag;

    wiced_event_flags_t             player_events;

    audio_player_dct_collection_t   dct_tables;
    int                             current_volume;

    audio_client_ref                audio_client;
    audio_client_suspend_t*         suspend;

    char                            work_buffer[WORK_BUFFER_LEN];

    char                            server_uri[SERVER_URI_MAX_LEN];
    char                            hostname[SERVER_URI_MAX_LEN];
    int                             port;

    int                             play_index_start;
    int                             play_index_end;
    int                             play_index_current;
    wiced_bool_t                    play_repeat;
    wiced_bool_t                    play_contiguous;

    uint32_t                        starting_offset_ms;

    int                             num_playlist_entries;
    char**                          playlist_entries;
    char*                           playlist_data;

    /*
     * Decoded audio buffer pool.
     */

    objhandle_t                     buf_pool;

    audio_render_ref                audio;

    uint32_t                        effect_mode;

    /*
     * Play raw testing.
     */

    wiced_tcp_socket_t              socket;
    wiced_tcp_socket_t*             socket_ptr;
    wiced_bool_t                    play_raw_active;
    wiced_bool_t                    disconnect_received;

} audio_player_t;


/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static int audio_player_console_command(int argc, char *argv[]);
static void audio_player_set_effect(audio_player_t* player);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static char audio_player_command_buffer[AUDIO_PLAYER_CONSOLE_COMMAND_MAX_LENGTH];
static char audio_player_command_history_buffer[AUDIO_PLAYER_CONSOLE_COMMAND_MAX_LENGTH * AUDIO_PLAYER_CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t audio_command_table[] =
{
#ifdef AUDIO_PLAYER_ENABLE_WIFI_CMDS
    WIFI_COMMANDS
#endif
    DCT_CONSOLE_COMMANDS
    AUDIO_PLAYER_CONSOLE_COMMANDS
    AUDIO_PLAYER_TRACEX_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[AUDIO_PLAYER_CONSOLE_CMD_MAX] =
{
    { "exit",           PLAYER_EVENT_SHUTDOWN           },
    { "config",         0                               },
    { "log",            0                               },
    { "play",           0                               },
    { "stop",           AUDIO_CLIENT_IOCTL_STOP         },
    { "pause",          AUDIO_CLIENT_IOCTL_PAUSE        },
    { "resume",         AUDIO_CLIENT_IOCTL_RESUME       },
    { "volume",         AUDIO_CLIENT_IOCTL_SET_VOLUME   },
    { "load_playlist",  0                               },
    { "playlist",       PLAYER_EVENT_OUTPUT_PLAYLIST    },
    { "clear",          0                               },
    { "stats",          0                               },
    { "seek",           0                               },
    { "memory",         0                               },
    { "effect",         0                               },
    { "play_offset",    0                               },
    { "playc",          0                               },
    { "suspend",        0                               },
    { "suspend_resume", 0                               },
    { "play_raw",       0                               },
};

static audio_player_t* g_player;


/******************************************************
 *               Function Definitions
 ******************************************************/

static int audio_player_log_output_handler(WICED_LOG_LEVEL_T level, char *logmsg)
{
    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}


static wiced_result_t client_connected_callback(wiced_tcp_socket_t* socket, void* arg)
{
    audio_player_t* player = (audio_player_t*)arg;

    if (player != NULL && player->tag == AUDIO_PLAYER_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_CONNECT);
    }

    return WICED_SUCCESS;
}


static wiced_result_t client_disconnected_callback(wiced_tcp_socket_t* socket, void* arg)
{
    audio_player_t* player = (audio_player_t*)arg;

    if (player != NULL && player->tag == AUDIO_PLAYER_TAG_VALID)
    {
        player->disconnect_received = WICED_TRUE;
        wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_DISCONNECT);
    }

    return WICED_SUCCESS;
}


static wiced_result_t received_data_callback(wiced_tcp_socket_t* socket, void* arg)
{
    audio_player_t* player = (audio_player_t*)arg;

    if (player != NULL && player->tag == AUDIO_PLAYER_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_PACKET);
    }

    return WICED_SUCCESS;
}


static void audio_player_delete_raw_socket(audio_player_t* player)
{
    if (player->socket_ptr)
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "Deleting socket\n");
        wiced_tcp_disconnect_with_timeout(player->socket_ptr, WICED_NO_WAIT);
        wiced_tcp_delete_socket(player->socket_ptr);
        player->socket_ptr = NULL;
    }

    player->disconnect_received = WICED_FALSE;
}


/****************************************************************
 *  Console command Function Declarations
 ****************************************************************/

static int audio_player_console_command(int argc, char *argv[])
{
    audio_client_track_info_t info;
    wiced_audio_config_t config;
    wiced_result_t result;
    uint32_t event = 0;
    uint32_t current_ms;
    uint32_t total_ms;
    char* ptr;
    int log_level;
    int volume;
    int index;
    int i;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Received command: %s\n", argv[0]);

    if (g_player == NULL || g_player->tag != AUDIO_PLAYER_TAG_VALID)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_player_console_command() Bad player structure\n");
        return ERR_CMD_OK;
    }

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < AUDIO_PLAYER_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= AUDIO_PLAYER_CONSOLE_CMD_MAX)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized command: %s\n", argv[0]);
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case AUDIO_PLAYER_CONSOLE_CMD_EXIT:
        case AUDIO_PLAYER_CONSOLE_CMD_PLAYLIST:
            event = command_lookup[i].event;
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_CONFIG:
            audio_player_set_config(&g_player->dct_tables, argc, argv);
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_LOG:
            log_level = atoi(argv[1]);
            wiced_log_printf("Setting new log level to %d (0 - off, %d - max debug)\n", log_level, WICED_LOG_DEBUG4);
            wiced_log_set_all_levels(log_level);
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_PLAY:
        case AUDIO_PLAYER_CONSOLE_CMD_PLAY_CONTIGUOUS:
            g_player->starting_offset_ms = 0;
            g_player->play_index_start   = -1;
            g_player->play_index_end     = -1;
            g_player->play_index_current = -1;
            g_player->play_repeat        = WICED_FALSE;
            g_player->play_contiguous    = (i == AUDIO_PLAYER_CONSOLE_CMD_PLAY_CONTIGUOUS ? WICED_TRUE : WICED_FALSE);
            if (g_player->playlist_entries != NULL && g_player->num_playlist_entries > 0)
            {
                /*
                 * There's a playlist loaded. Are we playing everything?
                 */

                if (strncasecmp(argv[1], "all", 3) == 0)
                {
                    g_player->play_index_start = 0;
                    g_player->play_index_end   = g_player->num_playlist_entries - 1;
                }
                else
                {
                    index = atoi(argv[1]);
                    if (index >= 0 && index < g_player->num_playlist_entries)
                    {
                        g_player->play_index_start = index;
                    }
                    else
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid playlist index %d\n", index);
                    }
                    ptr = strchr(argv[1], '-');
                    if (ptr != NULL)
                    {
                        /*
                         * We found a '-' in the argument. We assume that we're being given a range.
                         */

                        if (ptr[1] == '\0')
                        {
                            g_player->play_index_end = g_player->num_playlist_entries - 1;
                        }
                        else
                        {
                            index = atoi(&ptr[1]);
                            if (index >= 0 && index < g_player->num_playlist_entries)
                            {
                                g_player->play_index_end = index;
                            }
                            else
                            {
                                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid playlist index %d\n", index);
                            }
                        }
                    }
                }

                if (argc > 2 && strncasecmp(argv[2], "repeat", 6) == 0)
                {
                    g_player->play_repeat = WICED_TRUE;
                }

                if (g_player->play_index_start >= 0)
                {
                    g_player->play_index_current = g_player->play_index_start;
                    event = PLAYER_EVENT_PLAY;
                }
            }
            else
            {
                /*
                 * No playlist loaded. Just pass the URI directly to the audio client.
                 */

                if (g_player->audio_client)
                {
                    audio_client_ioctl(g_player->audio_client,
                                       (i == AUDIO_PLAYER_CONSOLE_CMD_PLAY_CONTIGUOUS ? AUDIO_CLIENT_IOCTL_PLAY_CONTIGUOUS : AUDIO_CLIENT_IOCTL_PLAY), argv[1]);
                }
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_PLAY_OFFSET:
            if (g_player->playlist_entries == NULL && g_player->num_playlist_entries == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No playlist loaded\n");
                break;
            }

            g_player->starting_offset_ms = 0;
            g_player->play_index_start   = -1;
            g_player->play_index_end     = -1;
            g_player->play_index_current = -1;
            g_player->play_repeat        = WICED_FALSE;
            g_player->play_contiguous    = WICED_FALSE;
            index = atoi(argv[1]);
            if (index >= 0 && index < g_player->num_playlist_entries)
            {
                g_player->play_index_start = index;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid playlist index %d\n", index);
                break;
            }

            g_player->starting_offset_ms = atoi(argv[2]);
            g_player->play_index_current = g_player->play_index_start;
            event = PLAYER_EVENT_PLAY;
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_STOP:
        case AUDIO_PLAYER_CONSOLE_CMD_PAUSE:
        case AUDIO_PLAYER_CONSOLE_CMD_RESUME:
            if (g_player->audio_client)
            {
                audio_client_ioctl(g_player->audio_client, command_lookup[i].event, NULL);
                if (i == AUDIO_PLAYER_CONSOLE_CMD_STOP && g_player->play_raw_active)
                {
                    audio_player_delete_raw_socket(g_player);

                    /*
                     * Pretend we're a disconnect event so we ensure that we shut down cleanly.
                     */

                    event = PLAYER_EVENT_DISCONNECT;
                }
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_VOLUME:
            volume = atoi(argv[1]);
            volume = MIN(volume, AUDIO_PLAYER_VOLUME_MAX);
            volume = MAX(volume, AUDIO_PLAYER_VOLUME_MIN);
            if (g_player->audio_client)
            {
                g_player->current_volume = volume;
                audio_client_ioctl(g_player->audio_client, command_lookup[i].event, (void*)volume);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_EFFECT:
            if (AUDIO_EFFECT_MODE_MAX == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio effects not supported\n");
            }
            else
            {
                int16_t index = atoi(argv[1]);
                if (index >= AUDIO_EFFECT_MODE_NONE && index < AUDIO_EFFECT_MODE_MAX)
                {
                    g_player->effect_mode = index;
                    event = PLAYER_EVENT_EFFECT;
                }
                else
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid audio effect mode index\n");
                }
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_LOAD_PLAYLIST:
            strlcpy(g_player->server_uri, argv[1], sizeof(g_player->server_uri));
            event = PLAYER_EVENT_LOAD_PLAYLIST;
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_CLEAR:
            g_player->play_index_start   = -1;
            g_player->play_index_end     = -1;
            g_player->play_index_current = -1;
            g_player->play_repeat        = WICED_FALSE;
            if (g_player->playlist_entries)
            {
                free(g_player->playlist_entries);
                g_player->playlist_entries = NULL;
                g_player->num_playlist_entries = 0;
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_STATS:
            if (g_player->audio_client)
            {
                audio_client_ioctl(g_player->audio_client, AUDIO_CLIENT_IOCTL_TRACK_INFO, &info);
                current_ms = (uint32_t)(1000.0 / ((double)info.sample_rate / (double)info.current_sample));

                if ( info.total_samples != 0 )
                {
                    total_ms = ( uint32_t )( 1000.0 / ( (double) info.sample_rate / (double) info.total_samples ) );
                }
                else if ( info.total_samples == 0 && info.bitrate != 0 )
                {
                    total_ms = ( g_player->audio_client->http_params.http_total_content_length ) / ( info.bitrate / 8 ) * 1000.0;
                }
                else
                {
                    total_ms = 0;
                }

                wiced_log_printf("Playback: %lu out of %lu samples   %3lu.%03lu of %3lu.%03lu\n",
                                 (uint32_t)info.current_sample, (uint32_t)info.total_samples, current_ms / 1000, current_ms % 1000, total_ms / 1000, total_ms % 1000);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_SEEK:
            current_ms = atoi(argv[1]);
            if (g_player->audio_client)
            {
                audio_client_ioctl(g_player->audio_client, AUDIO_CLIENT_IOCTL_SEEK, (void*)current_ms);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_MEMORY:
            {
                extern unsigned char _heap[];
                extern unsigned char _eheap[];
                extern unsigned char *sbrk_heap_top;
                volatile struct mallinfo mi = mallinfo();

                wiced_log_printf("sbrk heap size:    %7lu\n", (uint32_t)_eheap - (uint32_t)_heap);
                wiced_log_printf("sbrk current free: %7lu \n", (uint32_t)_eheap - (uint32_t)sbrk_heap_top);

                wiced_log_printf("malloc allocated:  %7d\n", mi.uordblks);
                wiced_log_printf("malloc free:       %7d\n", mi.fordblks);

                wiced_log_printf("\ntotal free memory: %7lu\n", mi.fordblks + (uint32_t)_eheap - (uint32_t)sbrk_heap_top);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_SUSPEND:
            if (g_player->audio_client != NULL)
            {
                if (g_player->suspend)
                {
                    free(g_player->suspend);
                    g_player->suspend = NULL;
                }
                result = audio_client_ioctl(g_player->audio_client, AUDIO_CLIENT_IOCTL_SUSPEND, (void*)&g_player->suspend);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio client suspend returns %d\n", result);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_SUSPEND_RESUME:
            if (g_player->suspend)
            {
                result = audio_client_ioctl(g_player->audio_client, AUDIO_CLIENT_IOCTL_SUSPEND_RESUME, (void*)g_player->suspend);
                g_player->suspend = NULL;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio client suspend resume returns %d\n", result);
            }
            break;

        case AUDIO_PLAYER_CONSOLE_CMD_PLAY_RAW:
            if (g_player->play_raw_active)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Play raw is currently active\n");
                break;
            }
            memset(&config, 0, sizeof(wiced_audio_config_t));
            config.sample_rate     = atoi(argv[1]);
            config.channels        = atoi(argv[2]);
            config.bits_per_sample = 16;
            config.frame_size      = 4;
            config.volume          = g_player->current_volume;

            result = audio_client_ioctl(g_player->audio_client, AUDIO_CLIENT_IOCTL_SET_RAW_CONFIG, (void*)&config);
            if (result == WICED_SUCCESS)
            {
                g_player->play_raw_active = WICED_TRUE;
                event = PLAYER_EVENT_PLAY_RAW;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to configure audio client for raw playback\n");
            }
            break;
    }

    if (event)
    {
        /*
         * Send off the event to the main audio loop.
         */

        wiced_rtos_set_event_flags(&g_player->player_events, event);
    }

    return ERR_CMD_OK;
}


static wiced_result_t audio_client_buf_get_callback(audio_client_ref handle, void* userdata, audio_client_buf_t* ac_buf, uint32_t timeout_ms)
{
    audio_player_t* player = (audio_player_t*)userdata;
    objhandle_t buf_handle;
    audio_buf_t* buf;
    uint32_t buf_size;
    (void)handle;

    if (player == NULL || player->tag != AUDIO_PLAYER_TAG_VALID || player->buf_pool == NULL)
    {
        return WICED_ERROR;
    }

    if (bufmgr_pool_alloc_buf(player->buf_pool, &buf_handle, (char **)&buf, &buf_size, timeout_ms) != BUFMGR_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Unable to allocate audio packet\n");
        return WICED_ERROR;
    }

    memset(buf, 0, sizeof(audio_buf_t));

    ac_buf->opaque = buf_handle;
    ac_buf->buf    = buf->buf;
    ac_buf->buflen = player->dct_tables.dct_app->audio_buffer_size;
    ac_buf->curlen = 0;
    ac_buf->offset = 0;
    ac_buf->flags  = 0;

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_buf_release_callback(audio_client_ref handle, void* userdata, audio_client_buf_t* buf)
{
    audio_player_t* player = (audio_player_t*)userdata;
    audio_render_buf_t render_buf;
    wiced_result_t result = WICED_SUCCESS;
    (void)handle;

    if (player == NULL || player->tag != AUDIO_PLAYER_TAG_VALID || buf == NULL)
    {
        return WICED_ERROR;
    }

    if (player->audio)
    {
        /*
         * Pass this buffer on to audio render.
         */

        if (buf->buf && buf->curlen > 0)
        {
            render_buf.pts         = 0;
            render_buf.data_buf    = buf->buf;
            render_buf.data_offset = buf->offset;
            render_buf.data_length = buf->curlen;
            render_buf.opaque      = buf->opaque;

            result = audio_render_push(player->audio, &render_buf);

            if (result != WICED_SUCCESS)
            {
                /*
                 * Audio render didn't accept the buffer, make sure that we don't leak it.
                 */

                if (player->buf_pool && buf->opaque)
                {
                    bufmgr_pool_free_buf(player->buf_pool, (objhandle_t)buf->opaque);
                    buf->opaque = NULL;
                }
            }
        }
        else if (player->buf_pool && buf->opaque)
        {
            /*
             * The decoder is releasing this buffer without writing any data to it.
             */

            bufmgr_pool_free_buf(player->buf_pool, (objhandle_t)buf->opaque);
            buf->opaque = NULL;
        }

        if (buf->flags & AUDIO_CLIENT_BUF_FLAG_EOS)
        {
            /*
             * Tell audio render that there's no more audio coming for this stream.
             */

            result = audio_render_push(player->audio, NULL);
        }
    }
    else
    {
        /*
         * No audio render instance. Just release the buffer.
         */

        if (player->buf_pool != NULL && buf->opaque != NULL)
        {
            bufmgr_pool_free_buf(player->buf_pool, (objhandle_t)buf->opaque);
        }
    }

    return result;
}


static wiced_result_t audio_client_callback(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg)
{
    audio_player_t* player = (audio_player_t*)userdata;
    wiced_audio_config_t* config;
    wiced_result_t result = WICED_SUCCESS;
    (void)handle;

    if (player == NULL || player->tag != AUDIO_PLAYER_TAG_VALID)
    {
        return WICED_BADARG;
    }

    switch (event)
    {
        case AUDIO_CLIENT_EVENT_ERROR:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error from audio_client: %d\n", (int)arg);
            break;

        case AUDIO_CLIENT_EVENT_AUDIO_FORMAT:
            config = (wiced_audio_config_t*)arg;
            if (config == NULL)
            {
                return WICED_BADARG;
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Audio format:\n");
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "    num channels: %u\n", config->channels);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, " bits per sample: %u\n", config->bits_per_sample);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "     sample rate: %lu\n", config->sample_rate);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "      frame size: %u\n", config->frame_size);

            if (player->audio != NULL)
            {
                result = audio_render_configure(player->audio, config);
                if (result != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from render configure: %d\n", result);
                }
                else
                {
                    if (player->effect_mode != AUDIO_EFFECT_MODE_NONE)
                        audio_player_set_effect(player);
                }
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
            wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_PLAYBACK_EOS);
            break;

        case AUDIO_CLIENT_EVENT_DECODE_COMPLETE:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Decode complete\n");
            break;

        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
            //wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Threshold high\n");
            break;

        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
            //wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Threshold low\n");
            break;

        case AUDIO_CLIENT_EVENT_HTTP_COMPLETE:
            if (player->playlist_data)
            {
                free(player->playlist_data);
            }
            player->playlist_data = arg;

            if (player->playlist_entries)
            {
                free(player->playlist_entries);
                player->playlist_entries = NULL;
            }
            player->num_playlist_entries = 0;
            wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_PLAYLIST_LOADED);
            break;

        case AUDIO_CLIENT_EVENT_HTTP_REDIRECT:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP redirect to %s\n", (char*)arg);
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio client event: %d\n", event);
            break;
    }

    return result;
}


wiced_result_t audio_render_buf_release(audio_render_buf_t* buf, void* userdata)
{
    audio_player_t* player = (audio_player_t*)userdata;

    /*
     * If we are in the process of shutting down, the player structure might be
     * tagged as invalid. So don't error out on that condition.
     */

    if (player == NULL)
    {
        return WICED_BADARG;
    }

    if (player->buf_pool != NULL && buf != NULL)
    {
        bufmgr_pool_free_buf(player->buf_pool, (objhandle_t)buf->opaque);
        buf->opaque = NULL;
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_render_event(audio_render_ref handle, void* userdata, AUDIO_RENDER_EVENT_T event, void* arg)
{
    audio_player_t* player = (audio_player_t*)userdata;
    (void)handle;
    (void)arg;

    if (player == NULL || player->tag != AUDIO_PLAYER_TAG_VALID)
    {
        return WICED_BADARG;
    }

    switch (event)
    {
        case AUDIO_RENDER_EVENT_EOS:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Audio render EOS event\n");
            wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_PLAYBACK_EOS);
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized audio render event: %d\n", event);
            break;
    }

    return WICED_SUCCESS;
}


static void audio_player_console_dct_callback(console_dct_struct_type_t struct_changed, void* app_data)
{
    audio_player_t* player = (audio_player_t*)app_data;

    /* sanity check */
    if (player == NULL)
    {
        return;
    }

    switch (struct_changed)
    {
        case CONSOLE_DCT_STRUCT_TYPE_WIFI:
            /* Get WiFi configuration */
            wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_RELOAD_DCT_WIFI);
            break;

        case CONSOLE_DCT_STRUCT_TYPE_NETWORK:
            /* Get network configuration */
            wiced_rtos_set_event_flags(&player->player_events, PLAYER_EVENT_RELOAD_DCT_NETWORK);
            break;

        default:
            break;
    }
}


static wiced_result_t audio_player_get_host_and_port(audio_player_t* player)
{
    const char* ptr;
    wiced_bool_t https;
    int copy_len;
    int len;

    player->port = 0;
    https = WICED_FALSE;

    /*
     * Skip over http:// or https://
     */

    ptr = player->server_uri;
    if ((ptr[0] == 'h' || ptr[0] == 'H') && (ptr[1] == 't' || ptr[1] == 'T') && (ptr[2] == 't' || ptr[2] == 'T') && (ptr[3] == 'p' || ptr[3] == 'P'))
    {
        ptr += 4;
        if (ptr[0] == 's' || ptr[0] == 'S')
        {
            https = WICED_TRUE;
            ptr++;
        }
        if (ptr[0] != ':' || ptr[1] != '/' || ptr[2] != '/')
        {
            return WICED_BADARG;
        }
        ptr += 3;
    }

    /*
     * Isolate the host part of the URI.
     */

    for (len = 0; ptr[len] != ':' && ptr[len] != '/' && ptr[len] != '\0'; )
    {
        len++;
    }

    if (ptr[len] == ':')
    {
        player->port = atoi(&ptr[len + 1]);
    }
    else
    {
        if (https == WICED_TRUE)
        {
            player->port = AUDIO_PLAYER_DEFAULT_HTTPS_PORT;
        }
        else
        {
            player->port = AUDIO_PLAYER_DEFAULT_HTTP_PORT;
        }
    }

    /*
     * And copy it over.
     */

    copy_len = len;
    if (copy_len > sizeof(player->hostname) - 1)
    {
        copy_len = sizeof(player->hostname) - 1;
    }
    memcpy(player->hostname, ptr, copy_len);
    player->hostname[copy_len] = '\0';

    return WICED_SUCCESS;
}


static void audio_player_load_playlist(audio_player_t* player)
{
    wiced_result_t      result;

    /*
     * Get and store the hostname and the port. We'll need that information later.
     */

    result = audio_player_get_host_and_port(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to parse server URI %s\n", player->server_uri);
        return;
    }

    if (player->port == AUDIO_PLAYER_DEFAULT_HTTP_PORT)
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "http://%s%s", player->hostname, AUDIO_PLAYER_PLAYLIST_NAME);
    }
    else if (player->port == AUDIO_PLAYER_DEFAULT_HTTPS_PORT)
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "https://%s%s", player->hostname, AUDIO_PLAYER_PLAYLIST_NAME);
    }
    else
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "%s:%d%s", player->hostname, player->port, AUDIO_PLAYER_PLAYLIST_NAME);
    }

    result = audio_client_ioctl(player->audio_client, AUDIO_CLIENT_IOCTL_LOAD_FILE, player->work_buffer);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from load file\n");
    }
}


static void audio_player_output_playlist(audio_player_t* player)
{
    int i;

    if (player->num_playlist_entries == 0 || player->playlist_entries == NULL)
    {
        wiced_log_printf("No playlist loaded\n");
    }
    else
    {
        wiced_log_printf("Playlist from %s:%d\n", player->hostname, player->port);
        for (i = 0; i < player->num_playlist_entries; i++)
        {
            wiced_log_printf("%3lu %s\n", i, player->playlist_entries[i]);
        }
    }
}


static void audio_player_parse_playlist(audio_player_t* player)
{
    char* ptr;
    int count;
    int inspace;

    /*
     * Do we have data to parse?
     */

    if (player->playlist_data == NULL)
    {
        return;
    }

    /*
     * Count how many playlist entries we have converting all whitespace to nuls.
     */

    count   = 0;
    inspace = 1;
    for (ptr = player->playlist_data; *ptr != '\0'; ptr++)
    {
        if (isspace((int)*ptr))
        {
            *ptr = '\0';
            inspace = 1;
        }
        else if (inspace)
        {
            inspace = 0;
            count++;
        }
    }

    /*
     * Allocate the entries list.
     */

    if ((player->playlist_entries = calloc(count, sizeof(char*))) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate playlist entries\n");
        free(player->playlist_data);
        player->playlist_data = NULL;
        return;
    }
    player->num_playlist_entries = count;

    /*
     * And set up the individual entries.
     */

    count   = 0;
    inspace = 1;
    for (ptr = player->playlist_data; count < player->num_playlist_entries; ptr++)
    {
        if (*ptr == '\0')
        {
            inspace = 1;
        }
        else if (inspace)
        {
            inspace = 0;
            player->playlist_entries[count++] = ptr;
        }
    }

    /*
     * Print out the playlist.
     */

    audio_player_output_playlist(player);
}


static void audio_player_play_playlist_entry(audio_player_t* player)
{
    wiced_result_t result;

    /*
     * Valid playlist index?
     */

    if (player->play_index_current < 0 || player->play_index_current >= player->num_playlist_entries)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid playlist index %d\n", player->play_index_current);
        return;
    }

    /*
     * Construct the complete track URI.
     */

    if (player->port == AUDIO_PLAYER_DEFAULT_HTTP_PORT)
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "http://%s%s", player->hostname, player->playlist_entries[player->play_index_current]);
    }
    else if (player->port == AUDIO_PLAYER_DEFAULT_HTTPS_PORT)
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "https://%s%s", player->hostname, player->playlist_entries[player->play_index_current]);
    }
    else
    {
        snprintf(player->work_buffer, sizeof(player->work_buffer), "%s:%d%s", player->hostname, player->port, player->playlist_entries[player->play_index_current]);
    }
    player->work_buffer[sizeof(player->work_buffer) - 1] = '\0';

    /*
     * And ask the audio client to play it for us.
     */

    if (player->starting_offset_ms)
    {
        audio_client_play_offset_t play_offset;

        play_offset.uri       = player->work_buffer;
        play_offset.offset_ms = player->starting_offset_ms;

        result = audio_client_ioctl(player->audio_client, AUDIO_CLIENT_IOCTL_PLAY_OFFSET, &play_offset);
    }
    else
    {
        result = audio_client_ioctl(player->audio_client, (player->play_contiguous ? AUDIO_CLIENT_IOCTL_PLAY_CONTIGUOUS : AUDIO_CLIENT_IOCTL_PLAY), player->work_buffer);
    }
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to start playback (%d)\n", result);
    }
}


static void audio_player_set_effect(audio_player_t* player)
{
    wiced_result_t      result;

    if (player->effect_mode >= AUDIO_EFFECT_MODE_MAX)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid parameter\n");
        return;
    }

    result = audio_client_ioctl(player->audio_client, AUDIO_CLIENT_IOCTL_EFFECT, (void*)((uint32_t)player->effect_mode));
    if(result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set audio effect mode (%d)\n", result);
    }
}


static void audio_player_start_play_raw(audio_player_t* player)
{
    wiced_result_t result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Opening socket on port %d (%p)\n", TCP_DEFAULT_LISTEN_PORT, player->socket_ptr);

    result = wiced_tcp_create_socket(&player->socket, WICED_STA_INTERFACE);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create socket\n");
        goto _error_out;
    }

    player->socket_ptr = &player->socket;
    result = wiced_tcp_register_callbacks(player->socket_ptr, client_connected_callback,
                                          received_data_callback, client_disconnected_callback, player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error setting socket callbacks\n");
        goto _error_out;
    }

    result = wiced_tcp_listen(player->socket_ptr, TCP_DEFAULT_LISTEN_PORT);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error setting socket in listen mode\n");
        goto _error_out;
    }

    return;

  _error_out:
    audio_player_delete_raw_socket(player);
    player->play_raw_active = WICED_FALSE;
}


static void audio_player_end_play_raw(audio_player_t* player)
{
    audio_client_push_buf_t buf;

    memset(&buf, 0, sizeof(audio_client_push_buf_t));
    audio_client_ioctl(player->audio_client, AUDIO_CLIENT_IOCTL_PUSH_RAW, (void*)&buf);

    audio_player_delete_raw_socket(player);
    player->play_raw_active = WICED_FALSE;
}


static void audio_player_handle_packet(audio_player_t* player)
{
    wiced_packet_t*         packet;
    wiced_result_t          result;
    uint8_t*                data;
    uint16_t                data_length;
    uint16_t                available_data_length;
    audio_client_push_buf_t ac_buf;

    while (player->socket_ptr && player->play_raw_active)
    {
        /*
         * Get the next packet.
         */

        result = wiced_tcp_receive(player->socket_ptr, &packet, WICED_NO_WAIT);
        if (result != WICED_SUCCESS)
        {
            if (result != WICED_TCPIP_TIMEOUT && !player->disconnect_received)
            {
                wiced_log_msg(WLF_DEF, WICED_LOG_ERR, "Error receiving packet (%d)\n", result);
            }
            return;
        }

        wiced_packet_get_data(packet, 0, &data, &data_length, &available_data_length);

        ac_buf.buf       = data;
        ac_buf.buflen    = data_length;
        ac_buf.pushedlen = 0;

        do
        {
            result = audio_client_ioctl(player->audio_client, AUDIO_CLIENT_IOCTL_PUSH_RAW, &ac_buf);
            if (result != WICED_SUCCESS)
            {
                if (result == WICED_PENDING)
                {
                    wiced_rtos_delay_milliseconds(2);
                }
                else
                {
                    wiced_log_msg(WLF_DEF, WICED_LOG_ERR, "audio_client: push raw failed with %d\n", result);
                    break;
                }
            }
        } while (result != WICED_SUCCESS);

        wiced_packet_delete(packet);
    }
}


static void audio_player_mainloop(audio_player_t* player)
{
    wiced_result_t      result;
    uint32_t            events;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Begin audio player mainloop\n");

    /*
     * If auto play is set then start off by sending ourselves a play event.
     */

    while (player->tag == AUDIO_PLAYER_TAG_VALID)
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&player->player_events, PLAYER_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & PLAYER_EVENT_SHUTDOWN)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "mainloop received EVENT_SHUTDOWN\n");
            break;
        }

        if (events & PLAYER_EVENT_LOAD_PLAYLIST)
        {
            audio_player_load_playlist(player);
        }

        if (events & PLAYER_EVENT_PLAYLIST_LOADED)
        {
            audio_player_parse_playlist(player);
        }

        if (events & PLAYER_EVENT_OUTPUT_PLAYLIST)
        {
            audio_player_output_playlist(player);
        }

        if (events & PLAYER_EVENT_PLAY)
        {
            audio_player_play_playlist_entry(player);
        }

        if (events & PLAYER_EVENT_PLAYBACK_EOS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Playback EOS\n");
            if (player->num_playlist_entries > 0)
            {
                if (++player->play_index_current > player->play_index_end)
                {
                    if (player->play_repeat && player->play_index_start >= 0)
                    {
                        player->play_index_current = player->play_index_start;
                    }
                    else
                    {
                        player->play_index_current = -1;
                    }
                }

                if (player->play_index_current >= 0)
                {
                    audio_player_play_playlist_entry(player);
                }
            }
        }

        if (events & PLAYER_EVENT_EFFECT)
        {
            audio_player_set_effect(player);
        }

        if (events & PLAYER_EVENT_PLAY_RAW)
        {
            audio_player_start_play_raw(player);
        }

        if (events & PLAYER_EVENT_CONNECT)
        {
            result = wiced_tcp_accept(player->socket_ptr);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_DEF, WICED_LOG_ERR, "Error accepting connection (%d)\n", result);
            }
        }

        if (events & PLAYER_EVENT_DISCONNECT)
        {
            audio_player_end_play_raw(player);
        }

        if (events & PLAYER_EVENT_PACKET)
        {
            audio_player_handle_packet(player);
        }

        if (events & PLAYER_EVENT_RELOAD_DCT_WIFI)
        {
            audio_player_config_reload_dct_wifi(&player->dct_tables);
        }

        if (events & PLAYER_EVENT_RELOAD_DCT_NETWORK)
        {
            audio_player_config_reload_dct_network(&player->dct_tables);
        }
    }   /* while */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "End audio player mainloop\n");
}


static void audio_player_shutdown(audio_player_t* player)
{
    /*
     * Shutdown the console.
     */

    command_console_deinit();

    wiced_rtos_deinit_event_flags(&player->player_events);

    player->tag = AUDIO_PLAYER_TAG_INVALID;

    if (player->audio_client)
    {
        audio_client_deinit(player->audio_client);
        player->audio_client = NULL;
    }

    if (player->audio)
    {
        audio_render_deinit(player->audio);
        player->audio = NULL;
    }

    if (player->playlist_data)
    {
        free(player->playlist_data);
        player->playlist_data = NULL;
    }

    if (player->playlist_entries)
    {
        free(player->playlist_entries);
        player->playlist_entries = NULL;
    }

    audio_player_config_deinit(&player->dct_tables);

    if (player->buf_pool != NULL)
    {
        bufmgr_pool_destroy(player->buf_pool);
        player->buf_pool = NULL;
    }

    free(player);
}


static audio_player_t* audio_player_init(void)
{
    audio_player_t*         player;
    audio_client_params_t   params;
    audio_render_params_t   render_params;
    wiced_result_t          result;
    uint32_t                tag;

    tag = AUDIO_PLAYER_TAG_VALID;

    /* Initialize the device */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return NULL;
    }

     /* initialize audio */
    platform_init_audio();

    /*
     * Initialize the logging subsystem.
     */

    wiced_log_init(WICED_LOG_ERR, audio_player_log_output_handler, NULL);

    /*
     * Allocate the main player structure.
     */

    player = calloc_named("audio_player", 1, sizeof(audio_player_t));
    if (player == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate player structure\n");
        return NULL;
    }

    /*
     * Create the command console.
     */

    printf("Start the command console\n");
    result = command_console_init(STDIO_UART, sizeof(audio_player_command_buffer), audio_player_command_buffer,
                                  AUDIO_PLAYER_CONSOLE_COMMAND_HISTORY_LENGTH, audio_player_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error starting the command console\n");
        free(player);
        return NULL;
    }
    console_add_cmd_table(audio_command_table);
    console_dct_register_callback(audio_player_console_dct_callback, player);

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&player->player_events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing player event flags\n");
        tag = AUDIO_PLAYER_TAG_INVALID;
    }

    /* read in our configurations */
    audio_player_config_init(&player->dct_tables);
    player->current_volume = player->dct_tables.dct_app->volume;

    /* print out our current configuration */
    audio_player_config_print_info(&player->dct_tables);

    /* Bring up the network interface */
    result = wiced_network_up_default(&player->dct_tables.dct_network->interface, NULL);
    if (result != WICED_SUCCESS)
    {
        /*
         * The network didn't initialize but we don't want to consider that a fatal error.
         * This allows the user to use the command console to update the network settings.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bringing up network interface failed\n");
    }

    /*
     * Fire off the audio client service.
     */

    memset(&params, 0, sizeof(audio_client_params_t));

    params.event_cb                    = audio_client_callback;
    params.userdata                    = player;
    params.interface                   = player->dct_tables.dct_network->interface;
    params.load_playlist_files         = WICED_TRUE;
    params.disable_hls_streaming       = WICED_FALSE; /* keep HLS (HTTP Live Streaming) support enabled */
    params.hls_max_entry_count         = 0;           /* let audio_client pick the minimum number of cached HLS playlist entries */

    params.data_buffer_num             = player->dct_tables.dct_app->http_buffer_num;
    params.audio_buffer_num            = player->dct_tables.dct_app->audio_buffer_num;
    params.audio_buffer_size           = player->dct_tables.dct_app->audio_buffer_size;
    params.audio_period_size           = player->dct_tables.dct_app->audio_period_size;
    params.data_buffer_preroll         = player->dct_tables.dct_app->http_buffer_preroll;
    params.no_length_disable_preroll   = player->dct_tables.dct_app->disable_preroll ? WICED_TRUE : WICED_FALSE;

    params.data_threshold_high         = player->dct_tables.dct_app->http_threshold_high;
    params.data_threshold_low          = player->dct_tables.dct_app->http_threshold_low;
    params.high_threshold_read_inhibit = player->dct_tables.dct_app->http_read_inhibit ? WICED_TRUE: WICED_FALSE;

    params.device_id                   = player->dct_tables.dct_app->audio_device_tx;
    params.volume                      = player->dct_tables.dct_app->volume;
    params.enable_playback             = player->dct_tables.dct_app->app_playback ? WICED_FALSE : WICED_TRUE;

    params.buffer_get                  = audio_client_buf_get_callback;
    params.buffer_release              = audio_client_buf_release_callback;

    if ((player->audio_client = audio_client_init(&params)) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize audio_client\n");
        tag = AUDIO_PLAYER_TAG_INVALID;
    }

    if (player->dct_tables.dct_app->app_playback)
    {
        /*
         * Create the audio buffer pool.
         */

        if (bufmgr_pool_create(&player->buf_pool, "audio_player_bufs", sizeof(audio_buf_t) + player->dct_tables.dct_app->audio_buffer_size,
            player->dct_tables.dct_app->audio_buffer_num, player->dct_tables.dct_app->audio_buffer_num, 0, sizeof(uint32_t)) != BUFMGR_OK)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bufmgr_pool_create failed\n");
            tag = AUDIO_PLAYER_TAG_INVALID;
        }

        /*
         * Fire off the audio render component.
         */

        memset(&render_params, 0, sizeof(audio_render_params_t));
        render_params.buffer_nodes   = player->dct_tables.dct_app->audio_buffer_num;
        render_params.device_id      = player->dct_tables.dct_app->audio_device_tx;
        render_params.period_size    = player->dct_tables.dct_app->audio_period_size;
        render_params.userdata       = player;
        render_params.buf_release_cb = audio_render_buf_release;
        render_params.event_cb       = audio_render_event;

        player->audio = audio_render_init(&render_params);
        if (player->audio == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize audio_render\n");
            tag = AUDIO_PLAYER_TAG_INVALID;
        }
    }

    /* set our valid tag */
    player->tag = tag;

    return player;
}


void application_start(void)
{
    audio_player_t* player;

    /*
     * Main initialization.
     */

    if ((player = audio_player_init()) == NULL)
    {
        return;
    }
    g_player = player;

    if (player->tag != AUDIO_PLAYER_TAG_VALID)
    {
        /*
         * We didn't initialize successfully. Bail out here so that the console
         * will remain active. This lets the user correct any invalid configuration parameters.
         * Mark the structure as valid so the console command processing will work and
         * note that we are intentionally leaking the player structure memory here.
         */

        player->tag = AUDIO_PLAYER_TAG_VALID;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Main application thread exiting...command console still active\n");
        return;
    }

    /*
     * Drop into our main loop.
     */

    audio_player_mainloop(player);

    /*
     * Cleanup and exit.
     */

    g_player = NULL;
    audio_player_shutdown(player);
    player = NULL;
}
