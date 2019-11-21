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
 * FLAC audio playback Application
 *
 * Free
 * Lossless
 * Audio
 * Compression
 *
 * This application snippet demonstrates how to use the WICED
 * interface to the FLAC decoder
 *
 * Application Instructions
 * 1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the wifi_config_dct.h header file to match your Wi-Fi access point
 * 2. Connect a PC terminal to the serial port of the WICED Eval board,
 *    then build and download the application as described in the WICED
 *    Quick Start Guide
 *
 * Create / Know where FLAC files can be found
 * 1. Use a server program like mini-httpd on Linux
 * 2. Create a file called "flac_playlist.txt" on your server
 *    Each line in the file is a uri to a FLAC file on the same server
 *    Do not use spaces in the file names
 *    file example:
     "/flac/track_01.flac\n"
     "/flac/track_02.flac\n"
     "/flac/track_03.flac\n"
     "/flac/track_04.flac\n"
     "/flac/track_05.flac\n"
     "/flac/track_06.flac\n"
 *
 *
 * After the download completes, it connects to the Wi-Fi AP specified in apps/snip/flac/wifi_config_dct.h
 * If the connection to the AP fails, you can still play the resource file to test FLAC decoding (see below)
 *      You can try to join the AP using the "join" console command:
 *  join <ssid> <open|wep|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [ip netmask gateway]
 *      --Encapsulate SSID in quotes in order to include spaces
 *      - Join an AP. DHCP assumed if no IP address provided
 *
 * "play"
 *          Play the FLAC file in the WICED resources.
 *          This does not require a server or HTTP connection.
 *
 * Issue these commands to play a FLAC file from your server:
 *
 * "connect <your server uri>"
 *          ex: "connect http://192.165.100.37"
 *          This will connect to your server and get the file "flac_playlist.txt"
 *          and print out the list to the console.
 *
 * "list"
 *          Print out the list to the console.
 *
 * "play x <loop>"
 *          ex: "play 3"
 *          This plays the 3rd file in the list from the server.
 *          Adding "loop" will loop this file after it is finished playing.
 *
 * "play all <loop>"
 *          This will play through all of the FLAC file sin the list from the server
 *          Adding "loop" will loop after it is finished playing the whole list.
 *
 */

#include "ctype.h"
#include "wiced.h"
#include "wiced_tcpip.h"
#include "platform_audio.h"
#include "command_console.h"
#include "console_wl.h"
#include "wifi/command_console_wifi.h"
#include "dct/command_console_dct.h"
#include "audio_render.h"
#include "resources.h"
#include "internal/wwd_sdpcm.h"

#include "flac_test.h"
#include "flac_config.h"

#include "wiced_flac_interface.h"

#ifdef WWD_TEST_NVRAM_OVERRIDE
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return WWD_BUFFER_ALLOC_FAIL; }
#define CHECK_RETURN( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }

#define Mod32_GT( A, B )        ( (int32_t)( ( (uint32_t)( A ) ) - ( (uint32_t)( B ) ) ) >   0 )

#define FLAC_CONSOLE_COMMANDS \
    { (char*) "exit",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Exit application" }, \
    { (char*) "connect",        flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Connect to flac server" }, \
    { (char*) "disconnect",     flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Disconnect from flac server" }, \
    { (char*) "play",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Start playing from resource" }, \
    { (char*) "info",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Info on current song" }, \
    { (char*) "list",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"List song URLs" }, \
    { (char*) "skip",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Skip current song" }, \
    { (char*) "stop",           flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Stop playing" }, \
    { (char*) "log_off",        flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Set log off" }, \
    { (char*) "log_info",       flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Set log info" }, \
    { (char*) "log_debug",      flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Set log debug" }, \
    { (char*) "config",         flac_console_command,    0, NULL, NULL, (char *)"", (char *)"Display / change config values" }, \


/******************************************************
 *                    Constants
 ******************************************************/

#define MY_DEVICE_NAME                      "flac_test"
#define MY_DEVICE_MODEL                     "1.0"
#define MAX_FLAC_COMMAND_LENGTH              (85)
#define FLAC_CONSOLE_COMMAND_HISTORY_LENGTH  (10)

#define FIRMWARE_VERSION                    "wiced-1.0"

#define NUM_FLAC_HEADERS    10

#define WICED_FLAC_BUFFER_NODE_COUNT         (256)

#define FLAC_BAR_GRAPH_LENGTH    62

#define MILLISECONDS_PER_SECOND         (uint64_t)(1000)
#define SECONDS_PER_MINUTE              (uint64_t)(60)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    FLAC_CONSOLE_CMD_EXIT = 0,

    FLAC_CONSOLE_CMD_CONNECT,
    FLAC_CONSOLE_CMD_DISCONNECT,

    FLAC_CONSOLE_CMD_PLAY,
    FLAC_CONSOLE_CMD_SKIP,
    FLAC_CONSOLE_CMD_STOP,

    FLAC_CONSOLE_CMD_LIST,
    FLAC_CONSOLE_CMD_INFO,

    FLAC_CONSOLE_CMD_CONFIG,

    FLAC_CONSOLE_CMD_LOG_LEVEL_OFF,
    FLAC_CONSOLE_CMD_LOG_LEVEL_INFO,
    FLAC_CONSOLE_CMD_LOG_LEVEL_DEBUG,


    FLAC_CONSOLE_CMD_MAX,
} FLAC_CONSOLE_CMDS_T;

#define NUM_NSECONDS_IN_SECOND                      (1000000000LL)
#define NUM_USECONDS_IN_SECOND                      (1000000)
#define NUM_NSECONDS_IN_MSECOND                     (1000000)
#define NUM_NSECONDS_IN_USECOND                     (1000)
#define NUM_USECONDS_IN_MSECOND                     (1000)
#define NUM_MSECONDS_IN_SECOND                      (1000)

/******************************************************
 *                 Type Definitions
 ******************************************************/


/******************************************************
 *                    Structures
 ******************************************************/

typedef struct cmd_lookup_s {
        char *cmd;
        uint32_t event;
} cmd_lookup_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

int flac_console_command(int argc, char *argv[]);

/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef PLATFORM_L1_CACHE_BYTES
#define NUM_BUFFERS_POOL_SIZE(x)       ((WICED_LINK_MTU_ALIGNED + sizeof(wiced_packet_t) + 1) * (x))
#define APP_RX_BUFFER_POOL_SIZE        NUM_BUFFERS_POOL_SIZE(WICED_FLAC_BUFFER_NODE_COUNT)
#endif

#ifdef PLATFORM_L1_CACHE_BYTES
uint8_t                          flac_rx_packets[APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES]        __attribute__ ((section (".external_ram")));
#else
uint8_t                          flac_rx_packets[WICED_NETWORK_MTU_SIZE * WICED_FLAC_BUFFER_NODE_COUNT]     __attribute__ ((section (".external_ram")));
#endif

static char flac_command_buffer[MAX_FLAC_COMMAND_LENGTH];
static char flac_command_history_buffer[MAX_FLAC_COMMAND_LENGTH * FLAC_CONSOLE_COMMAND_HISTORY_LENGTH];

uint8_t flac_thread_stack_buffer[FLAC_THREAD_STACK_SIZE]                               __attribute__ ((section (".bss.ccm")));

const command_t flac_command_table[] = {
    FLAC_CONSOLE_COMMANDS
    WL_COMMANDS
    WIFI_COMMANDS
    DCT_CONSOLE_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[FLAC_CONSOLE_CMD_MAX] = {
        { "exit",           PLAYER_EVENT_SHUTDOWN           },
        { "connect",        PLAYER_EVENT_CONNECT            },
        { "disconnect",     PLAYER_EVENT_DISCONNECT         },
        { "play",           PLAYER_EVENT_PLAY               },
        { "info",           PLAYER_EVENT_INFO               },
        { "list",           PLAYER_EVENT_LIST               },
        { "skip",           PLAYER_EVENT_SKIP               },
        { "stop",           PLAYER_EVENT_STOP               },
        { "config",         0                               },
        { "log_off",        PLAYER_EVENT_LOG_LEVEL_OFF      },
        { "log_info",       PLAYER_EVENT_LOG_LEVEL_INFO     },
        { "log_debug",      PLAYER_EVENT_LOG_LEVEL_DEBUG    },
};


/* template for HTTP GET */
char flac_get_request_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s%s \r\n"
    "\r\n"
};

const char* firmware_version = FIRMWARE_VERSION;

flac_player_t *g_player;

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t flac_app_audio_render_init(flac_player_t* player, uint32_t sample_rate, uint16_t channels, uint16_t bits_per_sample, uint8_t frame_size);


/******************************************************
 *               Function Definitions
 ******************************************************/

/****************************************************************
 *  Audio Render and Audio LPCM Buffer Function Declarations
 ****************************************************************/
wiced_result_t flac_app_audio_buffer_list_init(flac_player_t * player)
{
    audio_buff_list_t*  buf_ptr;
    int                 i;

    /* sanity check */
    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_audio_buffer_list_init() bad args %p\n", player));
        return WICED_BADARG;
    }

    wiced_rtos_init_mutex( &player->audio_buffer_mutex );

    /* allocate a list & buffers to go with it */
    player->num_audio_buffer_nodes = FLAC_APP_AUDIO_BUFFER_NODES;
    player->num_audio_buffer_used  = 0;
    player->audio_buffer_list      = (audio_buff_list_t *)calloc_named("audio_buf_list", player->num_audio_buffer_nodes, sizeof(audio_buff_list_t));
    if (player->audio_buffer_list == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_INFO, ("Unable to create audio buffer list\r\n"));
        return WICED_OUT_OF_HEAP_SPACE;
    }

    wiced_rtos_lock_mutex( &player->audio_buffer_mutex );

    buf_ptr = player->audio_buffer_list;
    for (i = 0; i < player->num_audio_buffer_nodes - 1; ++i)
    {
        buf_ptr[i].next          = &buf_ptr[i + 1];
        buf_ptr[i].data_buf_size = FLAC_APP_AUDIO_BUFFER_SIZE;
    }

    wiced_rtos_unlock_mutex( &player->audio_buffer_mutex );

    /* we keep the original buffer_list pointer to free it later, use the audio_buffer_free_list to manipulate */
    player->audio_buffer_free_list = player->audio_buffer_list;

    return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_buffer_list_deinit(flac_player_t* player)
{
    if (player == NULL) return WICED_ERROR;

    player->audio_buffer_free_list = NULL;

    wiced_rtos_lock_mutex( &player->audio_buffer_mutex );

    if (player->audio_buffer_list != NULL)
    {
        player->num_audio_buffer_nodes = 0;
        player->num_audio_buffer_used  = 0;
        free(player->audio_buffer_list);
    }
    player->audio_buffer_list = NULL;

    wiced_rtos_deinit_mutex( &player->audio_buffer_mutex );
return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_render_buffer_release(audio_render_buf_t *rend_buf, void* userdata)
{
    audio_buff_list_t*  buff_ptr;
    flac_player_t*      player = (flac_player_t *)userdata;

    if (player == NULL || player->tag != PLAYER_TAG_VALID)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("buffer release with bad player handle %p\n", player));
        return WICED_ERROR;
    }

    /* add the buffer back to the list */

    buff_ptr = (audio_buff_list_t*)rend_buf->opaque;
    if (buff_ptr != NULL)
    {
        wiced_rtos_lock_mutex( &player->audio_buffer_mutex );

        buff_ptr->next = player->audio_buffer_free_list;
        player->audio_buffer_free_list = buff_ptr;
        player->num_audio_buffer_used--;

        wiced_rtos_unlock_mutex( &player->audio_buffer_mutex );
    }
    else
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_app_audio_render_buffer_release() Bad Ptr! %p! \n", rend_buf));
    }
    return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_render_buffer_push(wiced_flac_buffer_info_t *buff_info)
{
    audio_render_buf_t ar_buf;
    flac_player_t * player = g_player;

    /* sanity check */
    if ((player == NULL) || (player->tag != PLAYER_TAG_VALID) || (buff_info == NULL))
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_audio_render_buffer_push() BAD ARG! %p %p\n", player, buff_info));
        return WICED_BADARG;
    }

    /* do we need to init the renderer? */
    if (flac_app_audio_render_init(player, buff_info->source_sample_rate, buff_info->filled_channels, buff_info->source_bps, buff_info->frame_size) != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("check_audio_config() failed!\r\n"));
        return WICED_ERROR;
    }

    /* play the sample */
    ar_buf.data_buf     = buff_info->data_buf;
    ar_buf.data_offset  = buff_info->filled_data_offset;
    ar_buf.data_length  = buff_info->filled_data_length;
    ar_buf.pts          = buff_info->filled_pts;
    ar_buf.opaque       = buff_info->opaque;

    if ((buff_info->source_total_samples > 0) &&
        ((buff_info->current_sample + buff_info->filled_samples) >= buff_info->source_total_samples) )
    {
        player->finished_playing_out = WICED_TRUE;
    }

    if (audio_render_push(player->audio_render, &ar_buf) != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("wiced_audio_render_push() failed! \r\n"));
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_render_buffer_get(wiced_flac_buffer_info_t *buff_info)
{
    audio_buff_list_t*  buf_ptr = NULL;
    flac_player_t*      player = g_player;
    uint16_t            num_channels, i;
    uint32_t            channel_mask;

    /* sanity check */
    if ((player == NULL) || (player->tag != PLAYER_TAG_VALID) || (buff_info == NULL))
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_get_free_audio_buffer() BAD args! %p %p\n", player, buff_info));
        return WICED_BADARG;
    }

    if (player->audio_buffer_free_list == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_get_free_audio_buffer() No buffers! \n"));
        return WICED_ERROR;
    }

    num_channels = 0;
    channel_mask = player->dct_app->channel;
    for(i = 0; i < (sizeof(channel_mask)*8); i++)
    {
        if (channel_mask & 0x01) num_channels++;
        channel_mask >>= 1;
    }
    if (num_channels == 0)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Unable to create new FLAC Audio Renderer - no valid channel mask!\r\n"));
        return WICED_ERROR;
    }

    /*
     * Grab a node off of the free list.
     */

    wiced_rtos_lock_mutex( &player->audio_buffer_mutex );

    buf_ptr = player->audio_buffer_free_list;
    player->audio_buffer_free_list = player->audio_buffer_free_list->next;
    player->num_audio_buffer_used++;

    buf_ptr->next = NULL;

    wiced_rtos_unlock_mutex( &player->audio_buffer_mutex );

    num_channels = 2;               /* todo: audio render always wants stereo  */
    /* fill in buff_info struct */
    memset(buff_info, 0, sizeof(wiced_flac_buffer_info_t));
    buff_info->data_buf      = buf_ptr->data_buf;
    buff_info->data_buf_size = buf_ptr->data_buf_size;
    buff_info->data_bps      = FLAC_APP_AUDIO_RENDER_BITS_PER_SAMPLE;
    buff_info->channel_mask  = player->dct_app->channel;
    buff_info->num_channels  = num_channels;
    /* so we can know which one it was later */
    buff_info->opaque        = buf_ptr;

    return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_render_init(flac_player_t* player, uint32_t sample_rate, uint16_t channels, uint16_t bits_per_sample, uint8_t frame_size)
{
    audio_render_params_t params;

    if (player->audio_render == NULL)
    {
        wiced_audio_config_t audio_config;

        memset(&params, 0, sizeof(audio_render_params_t));
        params.buffer_nodes   = FLAC_APP_AUDIO_BUFFER_NODES;
        params.buffer_ms      = FLAC_APP_AUDIO_BUFFER_MS;
        params.threshold_ms   = FLAC_APP_AUDIO_THRESH_MS;
        params.clock_enable   = FLAC_APP_AUDIO_CLOCK_ENABLE;
        params.device_id      = player->dct_app->audio_device_tx;
        params.userdata       = player;
        params.buf_release_cb = flac_app_audio_render_buffer_release;

        player->audio_render  = audio_render_init( &params );
        if (player->audio_render == NULL)
        {
            FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Unable to create new FLAC Audio Render\r\n"));
            return WICED_ERROR;
        }

        audio_config.bits_per_sample = bits_per_sample;
        audio_config.channels        = channels;
        audio_config.sample_rate     = sample_rate;
        audio_config.frame_size      = frame_size;
        audio_config.volume          = player->dct_app->volume;

        if (audio_render_configure(player->audio_render, &audio_config)!= WICED_SUCCESS)
        {
            FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("wiced_audio_render_configure() failed\r\n"));
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t flac_app_audio_render_deinit(flac_player_t* player)
{
    if (player == NULL) return WICED_ERROR;

    if (player->audio_render != NULL)
    {
        audio_render_deinit(player->audio_render);
        player->audio_render = NULL;
    }

    return WICED_SUCCESS;
}


/******************************************************
 *     Fake packet generation Function Declarations
 ******************************************************/

/* this is to simulate an incoming packet - we just use the packet as a buffer holder */
wiced_result_t flac_app_fake_send_packets(flac_player_t* player, flac_source_info_t *source)
{
    resource_result_t   res_result;
    wiced_result_t      result;
    wiced_packet_t*     out_packet = NULL;
    uint8_t*            out_data;
    uint16_t            out_data_len;

    uint32_t            total_data_to_send;
    uint32_t            chunk_size;
    uint32_t            size_read;
    uint32_t            data_offset;

    wiced_assert("flac_app_fake_send_packets() player == NULL!", (player != NULL) && (source != NULL));

    /* determine size of stream */
    total_data_to_send = source->src.res.size;
    data_offset = 0;
    while ((total_data_to_send > 0) && (player->stop_received == WICED_FALSE) && (player->skip_received == WICED_FALSE))

    {
        /* create a packet, figure out packet data size to send */
        chunk_size = total_data_to_send;

        if (wiced_packet_create( chunk_size, &out_packet, &out_data, &out_data_len) != WICED_SUCCESS )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_app_fake_send_packets() wiced_packet_create() error\r\n"));
            return WICED_ERROR;
        }
        if ( chunk_size > out_data_len)
        {
            /* too much data for the packet, send less */
            chunk_size = out_data_len;
        }

        res_result = resource_read ( source->src.res.resource,
                                     data_offset, chunk_size, &size_read, out_data );
        if ((size_read != 0) && (size_read < chunk_size))
        {
            chunk_size = size_read;
        }
        if (res_result != RESOURCE_SUCCESS)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, (" flac_app_fake_send_packets()  Failed reading the resource! offset: %ld res_size: %ld to_copy: %ld size_read: %ld\r\n",
                            data_offset, source->src.res.size,
                            chunk_size, size_read));
        }

        /* set end of data in packet */
        if ( wiced_packet_set_data_end( out_packet, (uint8_t *)&out_data[chunk_size] ) != WICED_SUCCESS)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_app_fake_send_packets() wiced_packet_set_data_end() error\r\n"));
            wiced_packet_delete( out_packet );
            return WICED_ERROR;
        }

        /* send packet to the flac decoder */
        result = wiced_flac_decoder_submit_packet(player->internal, out_packet);
        if (result != WICED_SUCCESS)
        {
            FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_app_fake_send_packets() wiced_flac_submit_packet() failed!\r\n"));
            wiced_packet_delete( out_packet );
            return WICED_ERROR;
        }

        data_offset += chunk_size;
        total_data_to_send -= chunk_size;

        FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_app_fake_send_packets() data_offset: %ld size: %ld !\r\n", data_offset, total_data_to_send));

        /* check for end of input stream */
        if (data_offset >= source->src.res.size)
        {
            break;
        }
    }

    return WICED_SUCCESS;
}

/****************************************************************
 *  Console command Function Declarations
 ****************************************************************/
int flac_console_command(int argc, char *argv[])
{
    uint32_t event = 0;
    int i;

    FLAC_APP_PRINT(FLAC_LOG_INFO, ("Received command: %s\n", argv[0]));

    if (g_player == NULL || g_player->tag != PLAYER_TAG_VALID)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_console_command() Bad player structure\r\n"));
        return ERR_CMD_OK;
    }

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < FLAC_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= FLAC_CONSOLE_CMD_MAX)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Unrecognized command: %s\n", argv[0]));
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case FLAC_CONSOLE_CMD_PLAY:
            g_player->source_type = FLAC_DECODE_RESOURCE;
            g_player->playback_type = PLAYBACK_TYPE_NONE;    /* no special flags (all, loop)( */
            memset(g_player->uri_to_stream, 0, sizeof(g_player->uri_to_stream));
            if ((argc > 1) && (strcasecmp(argv[1], "all") == 0) )
            {
                printf("argv[1] = %s  list:%p count:%d\n", argv[1], g_player->server_playlist, g_player->server_playlist_count);
                if ( g_player->server_playlist != NULL )
                {
                    g_player->playback_type |= PLAYBACK_TYPE_ALL;
                    g_player->current_play_list_index = 0;
                    g_player->source_type = FLAC_DECODE_HTTP;
                    strncpy(g_player->uri_to_stream,
                            (char*)g_player->server_playlist[g_player->current_play_list_index].uri,
                            sizeof(g_player->uri_to_stream));
                }
            }
            else if (argc > 1)
            {
                /* play a single file here - either an index, or a specified file **/
                int16_t index = atoi(argv[1]);
                if ( (g_player->server_playlist != NULL) && (index >= 0) && (index < g_player->server_playlist_count) )
                {
                    g_player->source_type = FLAC_DECODE_HTTP;
                    strncpy(g_player->uri_to_stream, (char*)g_player->server_playlist[index].uri, sizeof(g_player->uri_to_stream));
                }
                else
                {
                    g_player->source_type = FLAC_DECODE_HTTP;
                    strncpy(g_player->uri_to_stream, argv[1], (sizeof(g_player->uri_to_stream) - 1) );
                    printf("g_player->uri_to_stream(user):%s\n", g_player->uri_to_stream);
                }
            }

            if ((argc > 2) && (strcasecmp(argv[2], "loop") == 0))
            {
                /* set loop mode */
                g_player->playback_type |= PLAYBACK_TYPE_LOOP;
            }

            event = command_lookup[i].event;
            break;
        case FLAC_CONSOLE_CMD_CONNECT:
            g_player->playback_type = PLAYBACK_TYPE_NONE;
            memset(g_player->server_uri, 0, sizeof(g_player->server_uri));
            if (argc > 1)
            {
                    strncpy(g_player->server_uri, argv[1], (sizeof(g_player->server_uri) - 1) );
            }
            event = command_lookup[i].event;
            break;
        case FLAC_CONSOLE_CMD_DISCONNECT:
        case FLAC_CONSOLE_CMD_EXIT:
        case FLAC_CONSOLE_CMD_LIST:
        case FLAC_CONSOLE_CMD_INFO:
        case FLAC_CONSOLE_CMD_SKIP:
        case FLAC_CONSOLE_CMD_STOP:
        case FLAC_CONSOLE_CMD_LOG_LEVEL_OFF:
        case FLAC_CONSOLE_CMD_LOG_LEVEL_INFO:
        case FLAC_CONSOLE_CMD_LOG_LEVEL_DEBUG:
            event = command_lookup[i].event;
            break;

        case FLAC_CONSOLE_CMD_CONFIG:
            flac_set_config(g_player, argc, argv);
            break;


    }

    if (event)
    {
        /*
         * Send off the event to the main audio loop.
         */

        wiced_rtos_set_event_flags(&g_player->events, event);
    }

    return ERR_CMD_OK;
}

/****************************************************************
 *  HTTP URI connect / disconnect Function Declarations
 ****************************************************************/
wiced_result_t flac_uri_split(const char* uri, char* host_buff, uint16_t host_buff_len, char* path_buff, uint16_t path_buff_len, uint16_t* port)
{
   const char *uri_start, *host_start, *host_end;
   const char *path_start;
   uint16_t host_len, path_len;

  if ((uri == NULL) || (host_buff == NULL) || (path_buff == NULL) || (port == NULL))
  {
      return WICED_ERROR;
  }

  *port = 0;

  /* drop http:// or htts://"  */
  uri_start = strstr(uri, "http");
  if (uri_start == NULL)
  {
      uri_start = uri;
  }
  if (strncasecmp(uri_start, "http://", 7) == 0)
  {
      uri_start += 7;
  }
  else if (strncasecmp(uri_start, "https://", 8) == 0)
  {
      uri_start += 8;
  }

  memset(host_buff, 0, host_buff_len);

  host_start = uri_start;
  host_len = strlen(host_start);
  host_end = strchr(host_start, ':');
  if (host_end != NULL)
  {
      *port = atoi(host_end + 1);
  }
  else
  {
      host_end = strchr(host_start, '/');
  }

  if (host_end != NULL)
  {
      host_len = host_end - host_start;
  }
  if( host_len > (host_buff_len - 1))
  {
      host_len = host_buff_len - 1;
  }
  memcpy(host_buff, host_start, host_len);

  memset(path_buff, 0, path_buff_len);
  path_start = strchr(host_start, '/');
  if( path_start != NULL)
  {
      path_len = strlen(path_start);
      if( path_len > (path_buff_len - 1))
      {
          path_len = path_buff_len - 1;
      }
      memcpy(path_buff, path_start, path_len);
  }


  return WICED_SUCCESS;
}
wiced_result_t flac_check_socket_created(flac_player_t* player)
{
    wiced_result_t result;
    if (player->tcp_socket_created == WICED_TRUE)
    {
        return WICED_TRUE;
    }

    result = wiced_tcp_create_socket( &player->tcp_socket, WICED_STA_INTERFACE );
    if ( result != WICED_SUCCESS )
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("wiced_tcp_create_socket() failed!\r\n"));
        return result;
    }

    player->tcp_socket_created = WICED_TRUE;
    return result;
}

wiced_result_t flac_setup_stream_buffers(flac_player_t *player)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Guard against multiple creations of the packet pool */
    if (player->tcp_packet_pool_created == WICED_FALSE)
    {
        /* Create extra rx packet pool on the first megabyte of the external memory, this packet pool will be used for an audio queue */
        result = wiced_network_create_packet_pool(flac_rx_packets, (uint32_t)sizeof(flac_rx_packets), WICED_NETWORK_PACKET_RX);
        if (result != WICED_SUCCESS)
        {
            return result;
        }
        player->tcp_packet_pool_created = WICED_TRUE;
    }

    return WICED_SUCCESS;
}

/* get the file list */
wiced_result_t flac_get_server_file_list(flac_player_t* player)
{
    wiced_result_t result;
    char            port_name[16] = "\0";
    wiced_packet_t*     reply_packet;

    sprintf(port_name, ":%d", player->last_connected_port);
    sprintf(player->http_query, flac_get_request_template, FLAC_PLAYLIST_NAME, player->last_connected_host_name, port_name);
    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Sending query for FLAC file list...\r\n"));
    FLAC_APP_PRINT(FLAC_LOG_INFO, ("Sending query: [%s]\r\n", player->http_query));

    flac_check_socket_created(player);

    result = wiced_tcp_send_buffer( &player->tcp_socket, player->http_query, (uint16_t)strlen(player->http_query) );
    if ( result != WICED_SUCCESS )
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_get_server_file_list() wiced_tcp_send_buffer() failed!\r\n"));
        return result;
    }

    /* get and parse flac list file */
    if (result == WICED_SUCCESS )
    {
        uint16_t            file_count, i, len;
        char*               data_end;
        char*               data_walker;    /* walk through the list of files */
        char*               line_end;
        flac_play_list_t*   new_playlist;

        uint8_t*        in_data;
        uint16_t        avail_data_length;
        uint16_t        total_data_length;
        uint16_t        wait_loop_count;
        uint32_t        data_received;

        uint8_t*        body;
        uint32_t        body_length;
        uint32_t        content_length;

        char*           incoming_data;
        uint32_t        incoming_data_length;

        http_header_t   headers[1];
        headers[0].name = "Content-Length";
        headers[0].value = NULL;

        content_length = 1; /* so we continue to loop until we have a real content length */
        data_received = 0;
        incoming_data = NULL;
        incoming_data_length = 0;
        wait_loop_count = 0;
        do
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("calling wiced_tcp_receive(%s -- %s, timeout:%d ms)\n", player->last_connected_host_name, FLAC_PLAYLIST_NAME, 500 ));

            result = wiced_tcp_receive( &player->tcp_socket, &reply_packet, 500 ); /* short timeout */
            if (result == WICED_TIMEOUT)
            {
                FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_get_server_file_list() wiced_tcp_receive() %d timeout - retry!\r\n", result));
                result = WICED_SUCCESS; /* so we stay in our loop */
                wait_loop_count++;
                continue;
            }

            if (reply_packet != NULL)
            {
                body = NULL;
                body_length = 0;


                /* does this have headers? what are they ? */
                result = http_extract_headers( reply_packet, headers, 1 );
                if (result == WICED_SUCCESS)
                {
                    content_length = atol(headers[0].value);
                }

                result = http_get_body( reply_packet, &body, &body_length );
                if ((result != WICED_SUCCESS) || (body == NULL))
                {
                    /* no body defined - just look for the data in the packet */
                    result = wiced_packet_get_data( reply_packet, 0, &in_data, &avail_data_length, &total_data_length);
                    if (avail_data_length < total_data_length)
                    {
                        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Fragmented packets not supported!\r\n"));
                    }
                    else if (result == WICED_SUCCESS)
                    {
                        if ( (data_received == 0) && (strncmp( (char *)in_data, "HTTP", 4) == 0))
                        {
                            /* this is just a header message, ignore it */
                            continue;
                        }
                        body = in_data;
                        body_length = avail_data_length;
                    }
                }

                /* if we got data, copy it over */
                if ((body != NULL) && (body_length > 0))
                {
                    if (incoming_data == NULL)
                    {
                        incoming_data = malloc(body_length + 2);
                        if (incoming_data != NULL)
                        {
                            memcpy(incoming_data, body, body_length);
                            incoming_data_length = body_length;
                            data_received = body_length;
                        }
                        else
                        {
                            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_get_server_file_list() alloc() %d failed!\r\n", result));
                            return WICED_ERROR;
                        }
                    }
                    else
                    {
                        incoming_data = realloc(incoming_data, incoming_data_length + body_length + 2);
                        if (incoming_data != NULL)
                        {
                            memcpy(&incoming_data[incoming_data_length], body, body_length);
                            incoming_data_length += body_length;
                            data_received += body_length;
                        }
                        else
                        {
                            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_get_server_file_list() realloc() %d failed!\r\n", result));
                            return WICED_ERROR;
                        }
                    }
                }

                /* free the packet */
                wiced_packet_delete( reply_packet );
            }
        } while ((result == WICED_SUCCESS)&& (wait_loop_count < 100) && (data_received < content_length));

        /* got it - parse it */

        if ( (incoming_data != NULL) && (incoming_data_length > 0) )
        {
            file_count = 0;
            data_walker = incoming_data;
            data_end = &incoming_data[incoming_data_length];
            /* count # files in the list */
            while( (data_walker != NULL) && (data_walker < data_end) && (*data_walker != '\0'))
            {
                line_end = strchr((char*)data_walker, '\r');
                if (line_end == NULL)
                {
                    line_end = strchr((char*)data_walker, '\n');
                }
                if (line_end == NULL)
                {
                    break;
                }

                file_count++;

                /* skip to next line */
                data_walker = line_end;
                while( (data_walker != NULL) && (*data_walker != '\0') &&
                       ( (*data_walker == '\r') || (*data_walker == '\n') ) )
                {
                    data_walker++;
                }
            }

            if (file_count <= 0)
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_get_server_file_list() No files in flac_playlist.txt!\r\n"));
                if (incoming_data != NULL)
                {
                    free(incoming_data);
                }
                return WICED_SUCCESS;
            }

            /* malloc play list */
            new_playlist = malloc(file_count * sizeof(flac_play_list_t));
            if (new_playlist == NULL)
            {
                return WICED_OUT_OF_HEAP_SPACE;
            }
            memset(new_playlist, 0x00, (file_count * sizeof(flac_play_list_t)));

            /* copy data from incoming data to play ;list */
            i = 0;
            data_walker = incoming_data;
            while( (data_walker != NULL) && (data_walker < data_end) && (*data_walker != '\0') && ( i < file_count))
            {
                line_end = strchr((char*)data_walker, '\r');
                if (line_end == NULL)
                {
                    line_end = strchr((char*)data_walker, '\n');
                }

                if ((line_end != NULL) && (line_end > data_walker && (line_end < data_end)))
                {
                    len = line_end - data_walker;
                    if( len > FLAC_PLAYLIST_ENTRY_MAX)
                    {
                        len = FLAC_PLAYLIST_ENTRY_MAX;
                    }
                    strncpy((char*)new_playlist[i].uri, data_walker, len);
                    i++;
                }
                /* move to next entry */
                data_walker = line_end;
                while( (data_walker != NULL) && (*data_walker != '\0') &&
                       ( (*data_walker == '\r') || (*data_walker == '\n') ) )
                {
                    data_walker++;
                }
            }

            /* delete the old list, if there is one */
            if (player->server_playlist != NULL)
            {
                free(player->server_playlist);
                player->server_playlist = NULL;
                player->server_playlist_count = 0;
            }
            player->server_playlist = new_playlist;
            player->server_playlist_count = file_count;

            if (file_count > 0)
            {
                result = WICED_SUCCESS;
            }
        }

        /* free our temp data */
        if (incoming_data != NULL)
        {
            free(incoming_data);
        }

    } /* if we were successful in sending the request */

    /* print out the list */
    if ( (player->server_playlist != NULL) && (player->server_playlist_count > 0) )
    {
        uint16_t idx;

        for(idx = 0; idx < player->server_playlist_count; idx++)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("%d: %s\r\n", idx, player->server_playlist[idx].uri));
        }
    }

    return result;
}

wiced_result_t flac_connect(flac_player_t* player)
{
    wiced_result_t      result;
    wiced_ip_address_t  ip_address;
    uint16_t            port = 0;
    uint16_t            connect_tries;

    char                host_name[MAX_HTTP_HOST_NAME_SIZE];
    char                object_path[MAX_HTTP_OBJECT_PATH];

    flac_check_socket_created(player);

    result = flac_uri_split(player->server_uri, host_name, sizeof(host_name), object_path, sizeof(object_path), &port);
    if (result != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() flac_uri_split() failed %d!\r\n", result));
        return result;
    }

    /* check that we have a port # */
    if (port == 0)
    {
        port = HTTP_PORT;
    }


    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Connect to: host:%s path:%s port:%d\r\n", host_name, object_path, port) );


    if (isdigit((unsigned char)host_name[0]) && isdigit((unsigned char)host_name[1]) && isdigit((unsigned char)host_name[2])
            && host_name[3] == '.')
    {
        int         ip[4];
        char*       numeral;

        ip_address.version = WICED_IPV4;
        numeral = host_name;
        ip[0] = atoi(numeral);
        numeral = strchr(numeral, '.');
        if( numeral == NULL )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() parsing URL numerically failed 1!\r\n"));
            return result;

        }
        numeral++;
        ip[1] = atoi(numeral);
        numeral = strchr(numeral, '.');
        if( numeral == NULL )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() parsing URL numerically failed 2!\r\n"));
            return result;

        }
        numeral++;
        ip[2] = atoi(numeral);
        numeral = strchr(numeral, '.');
        if( numeral == NULL )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() parsing URL numerically failed 3!\r\n"));
            return result;

        }
        numeral++;
        ip[3] = atoi(numeral);
        numeral = strchr(numeral, '.');

        SET_IPV4_ADDRESS( ip_address, MAKE_IPV4_ADDRESS(ip[0], ip[1], ip[2], ip[3]));

        FLAC_APP_PRINT(FLAC_LOG_INFO, ("Using (%ld.%ld.%ld.%ld)\r\n",
                ((ip_address.ip.v4 >> 24) & 0xff), ((ip_address.ip.v4 >> 16) & 0x0ff),
                ((ip_address.ip.v4 >>  8) & 0xff), ((ip_address.ip.v4 >>  0) & 0x0ff)) );

    }
    else
    {
        FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_connect() wiced_hostname_lookup(%s)!\r\n", host_name));
        result =  wiced_hostname_lookup( host_name, &ip_address, 10000, WICED_STA_INTERFACE );
        if (result!= WICED_SUCCESS)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() wiced_hostname_lookup(%s) failed!\r\n", host_name));
            return result;
        }
    }

    connect_tries = 0;
    result = WICED_ERROR;
    do {
        connect_tries++;
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Try %d Connecting to %s:%d  (%ld.%ld.%ld.%ld) !\r\n",
                 connect_tries, host_name, port,
                ((ip_address.ip.v4 >> 24) & 0xff), ((ip_address.ip.v4 >> 16) & 0x0ff),
                ((ip_address.ip.v4 >>  8) & 0xff), ((ip_address.ip.v4 >>  0) & 0x0ff)) );
        result = wiced_tcp_connect( &player->tcp_socket, &ip_address, port, 2000 );
        if ( result != WICED_SUCCESS )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_connect() wiced_tcp_connect() failed! %d\r\n", result));
        }
    } while ((connect_tries < 3) && (result != WICED_SUCCESS) );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Connected to %ld.%ld.%ld.%ld : %d !\r\n",
            ((ip_address.ip.v4 >> 24) & 0xff), ((ip_address.ip.v4 >> 16) & 0x0ff),
            ((ip_address.ip.v4 >>  8) & 0xff), ((ip_address.ip.v4 >>  0) & 0x0ff), port) );


    strcpy(player->last_connected_host_name, host_name);
    player->last_connected_port = port;
    player->connect_state = WICED_TRUE;

    return result;
}

wiced_result_t flac_disconnect(flac_player_t* player)
{
    wiced_result_t result = WICED_SUCCESS;

    player->connect_state = WICED_FALSE;

    if (player->tcp_socket_created == WICED_TRUE)
    {
        result = wiced_tcp_disconnect( &player->tcp_socket );
        FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_disconnect() wiced_tcp_disconnect() - %d\r\n", result));
    }

    return result;
}

/****************************************************************
 *  Play back thread for an HTTP source file
 ****************************************************************/

void flac_http_play_thread(uint32_t arg)
{
    flac_player_t*      player = (flac_player_t*)arg;
    wiced_packet_t*     reply_packet;
    wiced_result_t      result;
    uint16_t            port = 0;
    wiced_bool_t        seen_fLaC_header;

    char                port_name[8] = "\0";
    char                host_name[MAX_HTTP_HOST_NAME_SIZE + 1];
    char                object_path[MAX_HTTP_OBJECT_PATH + 1];

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() Bad ARG\r\n"));
        return;
    }

    flac_uri_split(player->uri_to_stream, host_name, sizeof(host_name), object_path, sizeof(object_path), &port);
    if (port == 0)
    {
        port = player->last_connected_port;
        if (port == 0)
        {
            port = HTTP_PORT;
        }
    }

    if (strlen(host_name) < 4) {
        if (strlen(player->last_connected_host_name) >= 4)
        {
            strcpy(host_name, player->last_connected_host_name);
            strcpy(player->server_uri, player->last_connected_host_name);
        }
    }
    else
    {
        strcpy(player->server_uri, host_name);
    }

    result = flac_connect(player);
    if (result != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() flac_connect() failed %d\r\n", result));
        return;
    }

    sprintf(port_name, ":%d", port);
    sprintf(player->http_query, flac_get_request_template, player->uri_to_stream, host_name, port_name);
    FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Sending query: [%s]\r\n", player->http_query));

    result = wiced_tcp_send_buffer( &player->tcp_socket, player->http_query, (uint16_t)strlen(player->http_query) );
    if ( result != WICED_SUCCESS )
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() wiced_tcp_send_buffer() failed!\r\n"));
        goto _exit_http_thread;

    }

    seen_fLaC_header = WICED_FALSE;
    player->peer_closed_socket = WICED_FALSE;
    player->finished_playing_out = WICED_FALSE;
    while ( (result == WICED_SUCCESS) && (player->stop_received == WICED_FALSE) && (player->skip_received == WICED_FALSE))
    {
        uint8_t*        in_data;
        uint16_t        avail_data_length;
        uint16_t        total_data_length;
        uint8_t*        body;
        uint32_t        body_length;

        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("waiting for HTTP reply for %s  %s [%s]\n", host_name, object_path, player->http_query ));

        result = wiced_tcp_receive( &player->tcp_socket, &reply_packet, 100 ); /* short timeout so we respond to user stop faster */
        if ( result != WICED_SUCCESS )
        {
            if (result == WICED_TIMEOUT)
            {
                FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_http_play_thread() wiced_tcp_receive() %d timed out!\r\n", result));
                /* re- try */
                result = WICED_SUCCESS;
                continue;
            }
            if (result == WICED_TCPIP_SOCKET_CLOSED)
            {
                player->peer_closed_socket = WICED_TRUE;
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() wiced_tcp_receive() peer closed socket !\r\n"));
            }
            else
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() wiced_tcp_receive() %d failed!\r\n", result));
            }
            break;
        }

        result = http_get_body( reply_packet, &body, &body_length );
        if ((result != WICED_SUCCESS) || (body == NULL))
        {
            /* no body defined - just look for the data in the packet */
            result = wiced_packet_get_data( reply_packet, 0, &in_data, &avail_data_length, &total_data_length);
            if (avail_data_length < total_data_length)
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Fragmented packets not supported !\n"));
            } else if (result == WICED_SUCCESS)
            {
                body = in_data;
                body_length = avail_data_length;
            }
        }

        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_http_play_thread() body:%p body_length:%ld !\n", body, body_length));

        /* see what info we can get */
        if (seen_fLaC_header == WICED_FALSE)
        {
            http_header_t   headers[NUM_FLAC_HEADERS];
            uint16_t        num_headers;

            num_headers = NUM_FLAC_HEADERS;
            result = http_process_headers_in_place( reply_packet, headers, &num_headers);
            if (result == WICED_SUCCESS)
            {
                int i;
                for( i = 0; i < num_headers; i++)
                {
                    if (strcasecmp(headers[i].name, "Content-Length") == 0)
                    {
                        player->source.src.http.size = atol(headers[i].value);
                        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() Content-length: %ld\n", player->source.src.http.size) );
                    }
                }
            }

            /* this may be an fLaC header */
            if ((body[0] == 'f') && (body[1] == 'L')  && (body[2] == 'a') && (body[3] == 'C'))
            {
                /* this is the first "fLaC" header we've seen! */
                seen_fLaC_header = WICED_TRUE;
                FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_http_play_thread() Saw fLaC header!\n"));

                /* start up the decoder after we get the header info */
                wiced_flac_decoder_start(player->internal);

                wiced_time_get_time( &player->play_start_time );

            }
        }

        /* NOT an else here - if this packet has the "fLaC" header, we want to use it */
        if (seen_fLaC_header == WICED_TRUE)
        {
            /* send packet to the flac decoder */
            result = wiced_flac_decoder_submit_packet(player->internal, reply_packet);
            if (result != WICED_SUCCESS)
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_play_thread() wiced_flac_submit_packet() %d failed!\r\n", result));
                goto _exit_http_thread;
            }
        }
        else
        {
            /* header info, probably */
            FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_http_play_thread() Data: [[%s]]\r\n", body));
            wiced_packet_delete(reply_packet);
            reply_packet = NULL;
        }
    } /* while still streaming data */

    /* if source is done sending, we wait until the audio is played out */
    while ((player->stop_received == WICED_FALSE) &&
        (player->peer_closed_socket == WICED_TRUE) &&
        (player->finished_playing_out == WICED_FALSE))
    {
        wiced_rtos_delay_milliseconds(100);
    }

    /* wait a bit to let music play out */
    wiced_rtos_delay_milliseconds(500);

_exit_http_thread:
    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("http_thread() - shutting down\r\n"));

    flac_disconnect(player);

    if (reply_packet != NULL)
    {
        wiced_packet_delete(reply_packet);
    }

    wiced_flac_decoder_stop(player->internal);
    audio_render_command(player->audio_render, AUDIO_RENDER_CMD_STOP, NULL);
    flac_app_audio_render_deinit(player);

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("http_thread() - exiting\r\n"));

    player->http_thread_ptr = NULL;
    wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_HTTP_THREAD_DONE );

}


wiced_result_t flac_http_stream_start(flac_player_t* player)
{
    wiced_result_t result;

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_stream_start()\r\n"));

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_stream_start() Bad ARG\r\n"));
        return WICED_BADARG;
    }

    /* Start http thread */
    result = wiced_rtos_create_thread( &player->http_thread, FLAC_HTTP_THREAD_PRIORITY, "FLAC HTTP",
            flac_http_play_thread, FLAC_HTTP_STACK_SIZE, player);
    if (result != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_http_stream_start() wiced_rtos_create_thread(http) failed %d!\r\n", result));
        return WICED_ERROR;
    }
    else
    {
        player->http_thread_ptr = &player->http_thread;
    }

    FLAC_APP_PRINT(FLAC_LOG_INFO, ("flac_http_stream_start() - exiting\r\n"));

    return result;
}

wiced_result_t flac_skip_playback(flac_player_t* player)
{

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_skip_playback()\r\n"));

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_skip_playback() Bad ARG\r\n"));
        return WICED_BADARG;
    }

    player->skip_received = WICED_TRUE;
    if (player->http_thread_ptr != NULL )
    {
        wiced_rtos_thread_force_awake( &player->http_thread );

        /* Wait for HTTP thread to stop */
        while (player->http_thread_ptr != NULL )
        {
            wiced_rtos_delay_milliseconds(500);
        }
        wiced_rtos_thread_join( &player->http_thread );
    }

    return WICED_SUCCESS;
}

wiced_result_t flac_stop_playback(flac_player_t* player)
{

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_stop_playback()\r\n"));

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_stop_playback() Bad ARG\r\n"));
        return WICED_BADARG;
    }

    player->stop_received = WICED_TRUE;
    if (player->http_thread_ptr != NULL )
    {
        wiced_rtos_thread_force_awake( &player->http_thread );

        /* Wait for HTTP thread to stop */
        while (player->http_thread_ptr != NULL )
        {
            wiced_rtos_delay_milliseconds(500);
        }
        wiced_rtos_thread_join( &player->http_thread );
    }

    return WICED_SUCCESS;
}

void host_get_firmware_version(const char **firmware_string)
{
    *firmware_string = firmware_version;
}

void flac_app_print_sys_info(flac_player_t* player)
{
    wiced_time_t    curr_time;

    wiced_time_get_time( &curr_time );

    uint32_t    total_ms;
    uint32_t    min, sec, ms;

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("\r\nFLAC test Application :: firmware version: %s\r\n", FIRMWARE_VERSION ));

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("      play type: %s %s\r\n",
            ((player->playback_type & PLAYBACK_TYPE_ALL) != 0) ? "ALL" : "ONE",
            ((player->playback_type & PLAYBACK_TYPE_LOOP) != 0) ? "LOOPING" : "ONCE"
            ));

    total_ms = (uint32_t)(curr_time - player->start_time);
    ms = total_ms % (uint32_t)MILLISECONDS_PER_SECOND;
    sec = total_ms / (uint32_t)MILLISECONDS_PER_SECOND;
    min = sec / (uint32_t)SECONDS_PER_MINUTE;
    sec -= (min * (uint32_t)SECONDS_PER_MINUTE);
    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("    uptime    :: %ld:%ld.%ld\r\n", min, sec, ms));

    total_ms = (uint32_t)(curr_time - player->play_start_time);
    ms = total_ms % (uint32_t)MILLISECONDS_PER_SECOND;
    sec = total_ms / (uint32_t)MILLISECONDS_PER_SECOND;
    min = sec / (uint32_t)SECONDS_PER_MINUTE;
    sec -= (min * (uint32_t)SECONDS_PER_MINUTE);

    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("   play time :: %ld:%ld.%ld\r\n", min, sec, ms));
}

wiced_result_t flac_test_print_server_song_list(flac_player_t* player)
{
    int i;

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_test_print_server_song_list() Bad ARG\r\n"));
        return WICED_BADARG;
    }

    flac_app_print_sys_info(player);

    /* print out the songs list */
    if (player->server_playlist_count > 0)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("\r\n"));
        for( i = 0; i < player->server_playlist_count; i++ )
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("%d %s\r\n", i, player->server_playlist[i].uri ));

        }
    }

    return WICED_SUCCESS;
}

void flac_app_print_song_info(flac_player_t* player)
{
    wiced_flac_source_info_t source_info = {0};

    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("flac_app_print_song_info() Bad ARG\r\n"));
        return;
    }

    flac_app_print_sys_info(player);

    if (wiced_flac_get_source_info(player->internal, &source_info) == WICED_SUCCESS)
    {
        char            bar_graph[FLAC_BAR_GRAPH_LENGTH] = {0};
        char            partial_time[16] = {0};
        char            total_time[16] = {0};
        uint16_t        i;
        uint64_t        offset, total;
        uint32_t        total_ms;
        uint32_t        min, sec, ms;
        wiced_bool_t    print_bar_graph = WICED_FALSE;
        wiced_bool_t    print_time = WICED_FALSE;

        if (source_info.source_total_samples != 0)
        {
            total = source_info.source_total_samples;
        }
        else
        {
            total = player->source.src.http.size;
        }
        if (total != 0)
        {
            offset = (source_info.source_current_sample * (FLAC_BAR_GRAPH_LENGTH -2)) / total;
            print_bar_graph = WICED_TRUE;

            bar_graph[0] = '|';
            bar_graph[FLAC_BAR_GRAPH_LENGTH - 2] = '|';
            for (i = 1; i < (FLAC_BAR_GRAPH_LENGTH - 2); i++)
            {
                if (i == offset)
                {
                    bar_graph[i] = '|';
                }
                else
                {
                    bar_graph[i] = '-';
                }
            }
            bar_graph[FLAC_BAR_GRAPH_LENGTH - 1] = '\0';

            total_ms = (uint32_t)((source_info.source_current_sample * (uint64_t)MILLISECONDS_PER_SECOND) /
                                  (uint64_t)source_info.source_sample_rate);
            ms = total_ms % (uint32_t)MILLISECONDS_PER_SECOND;
            sec = total_ms / (uint32_t)MILLISECONDS_PER_SECOND;
            min = sec / (uint32_t)SECONDS_PER_MINUTE;
            sec  -= (min * (uint32_t)SECONDS_PER_MINUTE);
            snprintf(partial_time, sizeof(partial_time), "%ld:%ld.%ld", min, sec, ms);

            total_ms = (uint32_t)((source_info.source_total_samples *(uint64_t)MILLISECONDS_PER_SECOND) /
                                  (uint64_t)source_info.source_sample_rate);
            ms = total_ms % (uint32_t)MILLISECONDS_PER_SECOND;
            sec = total_ms / (uint32_t)MILLISECONDS_PER_SECOND;
            min = sec / (uint32_t)SECONDS_PER_MINUTE;
            sec -= (min * (uint32_t)SECONDS_PER_MINUTE);
            snprintf(total_time, sizeof(partial_time), "%ld:%ld.%ld", min, sec, ms);
            print_time = WICED_TRUE;

        }

        /* print out the song info */
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("\r\n"));
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("playback info for %s\r\n", player->uri_to_stream));
        if (print_bar_graph == WICED_TRUE)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("%s\r\n", bar_graph));
        }
        if (print_time == WICED_TRUE)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("            time: %s of %s\r\n", partial_time, total_time));
        }
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("     last played: %9ld\r\n", (uint32_t)source_info.source_current_sample));
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("   total samples: %9ld\r\n", (uint32_t)total));
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("            rate: %ld\r\n", source_info.source_sample_rate));
        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("        channels: %d\r\n", source_info.source_channels));
        FLAC_APP_PRINT(FLAC_LOG_ERROR, (" bits per sample: %d\r\n", source_info.source_bps));
    }
}

/****************************************************************
 *  Application Main loop Function
 ****************************************************************/

static void flac_test_mainloop(flac_player_t *player)
{
    wiced_result_t      result;
    uint32_t            events;

    FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Begin flac_test mainloop\r\n"));

    /*
     * If auto play is set then start off by sending ourselves a play event.
     */

    while (player->tag == PLAYER_TAG_VALID)
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&player->events, PLAYER_ALL_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & PLAYER_EVENT_SHUTDOWN)
        {
            FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("mainloop received EVENT_SHUTDOWN\r\n"));
            break;
        }

        if (events & PLAYER_EVENT_CONNECT)
        {
            if (player->connect_state != WICED_TRUE)
            {
                player->source.source_type = FLAC_DECODE_HTTP;
                result = flac_connect(player);
                if (result != WICED_SUCCESS)
                {
                    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("FAILED flac_connect:%d %s\r\n", result, player->server_uri));
                }
                else
                {
                    result = flac_get_server_file_list(player);
                    if (result != WICED_SUCCESS)
                    {
                        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("FAILED flac_get_server_file_list:%d %s\r\n", result, player->server_uri));
                    }
                    result = flac_disconnect(player);
                    if (result != WICED_SUCCESS)
                    {
                        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("FAILED flac_disconnect:%d %s\r\n", result, player->server_uri));
                    }
                }
            }
            else
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Already connected to %s\r\n", player->server_uri));
            }
        }

        if (events & PLAYER_EVENT_DISCONNECT)
        {
            if (player->connect_state == WICED_TRUE)
            {
                result = flac_disconnect(player);
                if (result != WICED_SUCCESS)
                {
                    FLAC_APP_PRINT(FLAC_LOG_ERROR, ("FAILED to disconnect: %s\r\n", player->server_uri));
                }
            }
            else
            {
                FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Not connected !\r\n"));
            }
        }

        if (events & PLAYER_EVENT_PLAY)
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("PLAY\r\n"));
            flac_app_print_sys_info(player);

            memset(&player->source, 0, sizeof(flac_source_info_t));
            player->source.source_type = player->source_type;
            if (player->source.source_type == FLAC_DECODE_RESOURCE)
            {

                player->source.src.res.resource = &resources_apps_DIR_flac_DIR_left_right_48k_16bit_2ch_flac;
                player->source.src.res.size = resources_apps_DIR_flac_DIR_left_right_48k_16bit_2ch_flac.size;

                wiced_flac_decoder_start(player->internal);
                flac_app_fake_send_packets(player, &player->source);
            }
            else
            {
                player->source.source_type = FLAC_DECODE_HTTP;
                player->source.src.http.uri = player->uri_to_stream;
                player->source.src.http.size = 0;               /* unknown */

                /* START STREAMING DATA ! */

                FLAC_APP_PRINT(FLAC_LOG_INFO, ("Start streaming %s\r\n", player->source.src.http.uri));

                player->stop_received = WICED_FALSE;
                player->skip_received = WICED_FALSE;
                if (player->http_thread_ptr == NULL)
                {
                    if (flac_http_stream_start(player) != WICED_SUCCESS)
                    {
                        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_http_stream_start() returned Error\r\n"));
                    }
                }
                else
                {
                    FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("HTTP thread already running\r\n"));
                }
            }
        }

        if (events & (PLAYER_EVENT_SKIP))
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("SKIP song\r\n"));
            flac_app_print_sys_info(player);

            flac_skip_playback(player);
        }

        if (events & (PLAYER_EVENT_STOP | PLAYER_EVENT_AUTOSTOP))
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("STOP\r\n"));
            flac_app_print_sys_info(player);

            flac_stop_playback(player);
        }

        if (events & PLAYER_EVENT_HTTP_THREAD_DONE)
        {
            wiced_rtos_delete_thread( &player->http_thread );
            FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("mainloop received PLAYER_EVENT_HTTP_THREAD_DONE\r\n"));

            /* if in "all" play mode and not at end of list, start next playback */
            if ((player->playback_type & PLAYBACK_TYPE_ALL) != 0)
            {
                if (player->stop_received != WICED_TRUE)
                {
                    player->current_play_list_index++;
                    if (player->current_play_list_index < (player->server_playlist_count - 1))
                    {
                        strncpy(g_player->uri_to_stream,
                                (char*)g_player->server_playlist[player->current_play_list_index].uri,
                                sizeof(g_player->uri_to_stream));
                        /* kick off another play */
                        wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_PLAY );
                        continue;
                    }
                    else if((player->playback_type & PLAYBACK_TYPE_LOOP) != 0)
                    {
                        player->current_play_list_index = 0;
                        strncpy(g_player->uri_to_stream,
                                (char*)g_player->server_playlist[player->current_play_list_index].uri,
                                sizeof(g_player->uri_to_stream));
                        /* kick off another play */
                        wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_PLAY );
                        continue;
                    }
                }
            }
            else if((player->playback_type & PLAYBACK_TYPE_LOOP) != 0)
            {
                if (player->stop_received != WICED_TRUE)
                {
                    /* kick off another play */
                    wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_PLAY );
                    continue;
                }
            }
            player->playback_type = PLAYBACK_TYPE_NONE;
            player->current_play_list_index = 0;
        }

        if (events & PLAYER_EVENT_LIST)
        {
            flac_test_print_server_song_list(player);
        }
        if (events & PLAYER_EVENT_INFO)
        {
            flac_app_print_song_info(player);
        }
        if (events & PLAYER_EVENT_LOG_LEVEL_OFF)
        {
            player->log_level = FLAC_LOG_ERROR + 1;
        }
        if (events & PLAYER_EVENT_LOG_LEVEL_INFO)
        {
            player->log_level = FLAC_LOG_INFO + 1;
        }
        if (events & PLAYER_EVENT_LOG_LEVEL_DEBUG)
        {
            player->log_level = FLAC_LOG_DEBUG + 1;
        }

        if (events & PLAYER_EVENT_RELOAD_DCT_WIFI)
        {
            flac_config_reload_dct_wifi(player);
        }
        if (events & PLAYER_EVENT_RELOAD_DCT_NETWORK)
        {
            flac_config_reload_dct_network(player);
        }

    }   /* while */

    /*
     * Make sure that playback has been shut down.
     */

    if (player->connect_state == WICED_TRUE)
    {
        result = flac_disconnect(player);
        if (result == WICED_SUCCESS)
        {
            player->connect_state = WICED_FALSE;
        }
        else
        {
            FLAC_APP_PRINT(FLAC_LOG_ERROR, ("FAILED to disconnect: %s\r\n", player->server_uri));
        }
    }

    FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("End flac_test mainloop\r\n"));
}

static void shutdown_flac_test(flac_player_t *player)
{

    /*
     * Shutdown the console.
     */
    command_console_deinit();

    if (player->tcp_socket_created == WICED_TRUE)
    {
        wiced_tcp_delete_socket( &player->tcp_socket );
        FLAC_APP_PRINT(FLAC_LOG_INFO, ("delete_socket()\r\n"));
    }
    player->tcp_socket_created = WICED_FALSE;

    wiced_flac_decoder_stop(player->internal);

    wiced_flac_decoder_deinit(player->internal);

    if (flac_app_audio_buffer_list_deinit(player) != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_audio_buffer_list_deinit() failed!\r\n"));
    }

    wiced_rtos_deinit_event_flags(&player->events);

    flac_config_deinit(player);

    player->tag = PLAYER_TAG_INVALID;
    free(player);
}

static void flac_console_dct_callback(console_dct_struct_type_t struct_changed, void* app_data)
{
    flac_player_t*      player;

    /* sanity check */
    if (app_data == NULL)
    {
        return;
    }

    player = (flac_player_t*)app_data;
    switch(struct_changed)
    {
        case CONSOLE_DCT_STRUCT_TYPE_WIFI:
            /* Get WiFi configuration */
            wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_RELOAD_DCT_WIFI);
            break;
        case CONSOLE_DCT_STRUCT_TYPE_NETWORK:
            /* Get network configuration */
            wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_RELOAD_DCT_NETWORK);
            break;
        default:
            break;
    }
}

static flac_player_t *init_player(void)
{
    flac_player_t*      player;
    wiced_result_t      result;
    uint32_t            tag;
    wiced_flac_params_t flac_params;

    tag = PLAYER_TAG_VALID;

    /* Initialize the device */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return NULL;
    }

    /* initialize audio */
    platform_init_audio();

    /*
     * Allocate the main player structure.
     */
    player = calloc_named("flac_player", 1, sizeof(flac_player_t));
    if (player == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Unable to allocate player structure\r\n"));
        return NULL;
    }

    /*
     * Create the command console.
     */

    FLAC_APP_PRINT(FLAC_LOG_INFO, ("Start the command console\r\n"));
    result = command_console_init(STDIO_UART, sizeof(flac_command_buffer), flac_command_buffer, FLAC_CONSOLE_COMMAND_HISTORY_LENGTH, flac_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Error starting the command console\r\n"));
        free(player);
        return NULL;
    }
    console_add_cmd_table(flac_command_table);
    console_dct_register_callback(flac_console_dct_callback, player);

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&player->events);
    if (result != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Error initializing event flags\r\n"));
        tag = PLAYER_TAG_INVALID;
    }

    /* init the FLAC decoder */
    flac_params.buff_get  = flac_app_audio_render_buffer_get;
    flac_params.buff_push = flac_app_audio_render_buffer_push;
    player->internal = wiced_flac_decoder_init(&flac_params);
    if (player->internal == NULL)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("Error initializing FLAC decoder\r\n"));
        tag = PLAYER_TAG_INVALID;
    }

    if (flac_app_audio_buffer_list_init(player) != WICED_SUCCESS)
    {
        FLAC_APP_PRINT(FLAC_LOG_DEBUG, ("flac_app_audio_buffer_list_init() failed!\r\n"));
    }

    /* read in our configurations */
    flac_config_init(player);

    /* print out our current configuration */
    flac_config_print_info(player);

    /* set the initial volume level */
    audio_render_command(player->audio_render, AUDIO_RENDER_CMD_SET_VOLUME, (void*)((uint32_t)player->dct_app->volume));

    /* Bring up the network interface */
    result = wiced_network_up_default(&player->dct_network->interface, NULL);
    if (result != WICED_SUCCESS)
    {
        /*
         * The network didn't initialize but we don't want to consider that a fatal error.
         * Make sure that autoplay is disabled to we don't try and use the network.
         */

        FLAC_APP_PRINT(FLAC_LOG_ERROR, ("Bringing up network interface failed!\r\n"));
    }
    else
    {
        /* create a socket */
        flac_check_socket_created(player);
    }

    flac_setup_stream_buffers(player);

    /* set our valid tag */
    player->tag = tag;

    return player;
}


void application_start(void)
{
    flac_player_t *player;

    /*
     * Main initialization.
     */

    if ((player = init_player()) == NULL)
    {
        return;
    }
    g_player = player;

    wiced_time_get_time( &g_player->start_time );

    /*
     * Drop into our main loop.
     */
    flac_test_mainloop(player);

    /*
     * Cleanup and exit.
     */

    g_player = NULL;
    shutdown_flac_test(player);
    player = NULL;
}
