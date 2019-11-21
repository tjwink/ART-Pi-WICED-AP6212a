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
 * Console command handling for the Apollo application.
 *
 */

#include <unistd.h>
#include <malloc.h>

#include "apollo_context.h"
#include "platform_audio.h"
#include "command_console.h"
#include "console_wl.h"
#ifndef APOLLO_NO_WIFI_COMMANDS
#include "wifi/command_console_wifi.h"
#endif
#include "dct/command_console_dct.h"

#include "apollo_debug.h"
#include "apollo_wl_utils.h"
#include "apollo_tracex.h"

#if defined(USE_AMBILIGHT)
#include "amblt_source_types.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#if defined(OTA2_SUPPORT)
#define OTA2_SUPPORT_COMMANDS   \
    { (char*) "get_update",     apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Get OTA2 Update (extract on reboot)" }, \
    { (char*) "timed_update",   apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Start OTA2 timed Update (extract on reboot)" }, \
    { (char*) "stop_update",    apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Stop  OTA2 timed Update (extract on reboot)" }, \
    { (char*) "ota2_status",    apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Current OTA2 status (log_level > 0)" }, \

#else
#define OTA2_SUPPORT_COMMANDS
#endif

#define APOLLO_CONSOLE_COMMANDS \
    { (char*) "exit",           apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Exit application" }, \
    { (char*) "start",          apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Start source/sink" }, \
    { (char*) "stop",           apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Stop source/sink" }, \
    { (char*) "volume",         apollo_console_command, 1, NULL, NULL, (char *)"", (char *)"Set the audio volume (0-100)" }, \
    { (char*) "config",         apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display / change config values" }, \
    { (char*) "log",            apollo_console_command, 1, NULL, NULL, (char *)"", (char *)"Set the logging level" }, \
    { (char*) "ascu_time",      apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display current time" }, \
    { (char*) "stats",          apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display player stats" }, \
    { (char*) "netlog",         apollo_console_command, 1, NULL, NULL, (char *)"", (char *)"Turn network logging on|off" }, \
    { (char*) "rmc_rate",       apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Get/Set RMC Tx Rate" }, \
    { (char*) "rmc_status",     apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display rmc_status info" }, \
    { (char*) "reboot",         apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Reboot the platform" }, \
    { (char*) "reset_counters", apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Reset apollo and WiFi persistent counters" }, \
    { (char*) "rx_counters",    apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Print persistent WiFi receive counters" }, \
    { (char*) "tx_counters",    apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Print persitent WiFi transmit counters" }, \
    { (char*) "memory",         apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display memory stats" }, \
    { (char*) "rmc_down",       apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Bring down RMC Network" },  \
    { (char*) "rmc_up",         apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Bring up RMC Network" }, \
    { (char*) "rmc_stats",      apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Display rmc stats" }, \
    { (char*) "rmc_ackreq",     apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Set/Get rmc_ackreq" }, \
    { (char*) "rtp_dump",       apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Dump RTP stats from FW" }, \
    { (char*) "amblt",          apollo_console_command, 0, NULL, NULL, (char *)"", (char *)"Ambilight control command" }, \
    OTA2_SUPPORT_COMMANDS

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_CONSOLE_CMD_EXIT = 0,
    APOLLO_CONSOLE_CMD_START,
    APOLLO_CONSOLE_CMD_STOP,
    APOLLO_CONSOLE_CMD_VOLUME,
    APOLLO_CONSOLE_CMD_CONFIG,
    APOLLO_CONSOLE_CMD_LOG,
    APOLLO_CONSOLE_CMD_TIME,
    APOLLO_CONSOLE_CMD_STATS,
    APOLLO_CONSOLE_CMD_NETLOG,
    APOLLO_CONSOLE_CMD_RMC_RATE,
    APOLLO_CONSOLE_CMD_RMC_STATUS,
    APOLLO_CONSOLE_CMD_REBOOT,
    APOLLO_CONSOLE_CMD_RESET_COUNTERS,
    APOLLO_CONSOLE_CMD_GET_RX_COUNTERS,
    APOLLO_CONSOLE_CMD_GET_TX_COUNTERS,
    APOLLO_CONSOLE_CMD_MEMORY,
    APOLLO_CONSOLE_CMD_RMC_DOWN,
    APOLLO_CONSOLE_CMD_RMC_UP,
    APOLLO_CONSOLE_CMD_RMC_COUNTERS,
    APOLLO_CONSOLE_CMD_RMC_ACKREQ,
    APOLLO_CONSOLE_CMD_RTP_DUMP,
#if defined(USE_AMBILIGHT)
    APOLLO_CONSOLE_CMD_AMBILIGHT,
#endif
#if defined(OTA2_SUPPORT)
    APOLLO_CONSOLE_CMD_GET_UPDATE,
    APOLLO_CONSOLE_CMD_START_TIMED_UPDATE,
    APOLLO_CONSOLE_CMD_STOP_TIMED_UPDATE,
    APOLLO_CONSOLE_CMD_OTA2_STATUS,
#endif
    APOLLO_CONSOLE_CMD_MAX,
} APOLLO_CONSOLE_CMDS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    char *cmd;
    uint32_t event;
} cmd_lookup_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static int apollo_console_command(int argc, char *argv[]);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static char apollo_command_buffer[APOLLO_CONSOLE_COMMAND_MAX_LENGTH];
static char apollo_command_history_buffer[APOLLO_CONSOLE_COMMAND_MAX_LENGTH * APOLLO_CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t apollo_command_table[] =
{
    APOLLO_CONSOLE_COMMANDS
    WL_COMMANDS
#ifndef APOLLO_NO_WIFI_COMMANDS
    WIFI_COMMANDS
#endif
    DCT_CONSOLE_COMMANDS
    APOLLO_TRACEX_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[APOLLO_CONSOLE_CMD_MAX] =
{
    { "exit",           APOLLO_EVENT_SHUTDOWN   },
    { "start",          APOLLO_EVENT_START      },
    { "stop",           APOLLO_EVENT_STOP       },
    { "volume",         0                       },
    { "config",         0                       },
    { "log",            0                       },
    { "ascu_time",      0                       },
    { "stats",          0                       },
    { "netlog",         0                       },
    { "rmc_rate",       0                       },
    { "rmc_status",     0                       },
    { "reboot",         0                       },
    { "reset_counters", 0                       },
    { "rx_counters",    0                       },
    { "tx_counters",    0                       },
    { "memory",         0                       },
    { "rmc_down",       APOLLO_EVENT_RMC_DOWN   },
    { "rmc_up",         APOLLO_EVENT_RMC_UP     },
    { "rmc_stats",      0                       },
    { "rmc_ackreq",     0                       },
    { "rtp_dump",       0                       },
#if defined(USE_AMBILIGHT)
    { "amblt",          APOLLO_EVENT_AMBILIGHT  },
#endif
#if defined(OTA2_SUPPORT)
    { "get_update",     APOLLO_EVENT_GET_UPDATE         },
    { "timed_update",   APOLLO_EVENT_START_TIMED_UPDATE },
    { "stop_update",    APOLLO_EVENT_STOP_TIMED_UPDATE  },
    { "ota2_status",    APOLLO_EVENT_OTA2_STATUS  },
#endif
};


const char* apollo_console_delimiter_string = " ";


/******************************************************
 *               Function Definitions
 ******************************************************/

static void apollo_console_print_socket_info(NX_UDP_SOCKET* nx_socket_ptr)
{
    ULONG packets_sent;
    ULONG bytes_sent;
    ULONG packets_received;
    ULONG bytes_received;
    ULONG packets_queued;
    ULONG receive_packets_dropped;
    ULONG checksum_errors;

    if (nx_udp_socket_info_get(nx_socket_ptr, &packets_sent, &bytes_sent, &packets_received, &bytes_received,
                               &packets_queued, &receive_packets_dropped, &checksum_errors) == NX_SUCCESS)
    {
        wiced_log_printf("UDP stack: packets sent %u, bytes sent %u, packets received %u, bytes received %u\n", packets_sent, bytes_sent, packets_received, bytes_received);
        wiced_log_printf("           packets queued %u, receive packets dropped %u, checksum errors %u\n", packets_queued, receive_packets_dropped, checksum_errors);
    }
}


int apollo_console_command(int argc, char *argv[])
{
    apollo_player_stats_t stats;
    apollo_streamer_stats_t streamer_stats;
    wiced_udp_socket_t  socket;
    wiced_udp_socket_t* socket_ptr;
    wiced_counters_t counters;
    wiced_8021as_time_t as_time;
    uint32_t event = 0;
    int log_level;
    int result;
    int i;
    apollo_volume_t volume =
    {
        .volume       = 0,
        .caching_mode = (APOLLO_CACHE_TO_MEMORY|APOLLO_STORE_TO_NVRAM),
    };

#if defined(USE_AMBILIGHT)
    apollo_cmd_ambilight_t cmd_ambilight;
#endif

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Apollo console received command: %s\n", argv[0]);

    if (g_apollo == NULL || (g_apollo->tag != APOLLO_TAG_VALID && !g_apollo->initializing))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bad app structure\n");
        return ERR_CMD_OK;
    }

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < APOLLO_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= APOLLO_CONSOLE_CMD_MAX)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized command: %s\n", argv[0]);
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case APOLLO_CONSOLE_CMD_EXIT:
        case APOLLO_CONSOLE_CMD_START:
        case APOLLO_CONSOLE_CMD_STOP:
        case APOLLO_CONSOLE_CMD_RMC_DOWN:
        case APOLLO_CONSOLE_CMD_RMC_UP:
            event = command_lookup[i].event;
            break;

        case APOLLO_CONSOLE_CMD_VOLUME:
            volume.volume = atoi(argv[1]);
            if (g_apollo->player_handle)
            {
                apollo_player_set_volume(g_apollo->player_handle, volume.volume);
            }
            else if (g_apollo->streamer_handle)
            {
                apollo_streamer_set_volume(g_apollo->streamer_handle, volume.volume);

                /*
                 * Since we're the streamer, we want to tell all the sinks about the volume change.
                 */

                if (g_apollo->cmd_sender_handle != NULL)
                {
                    apollo_cmd_sender_command(g_apollo->cmd_sender_handle, APOLLO_CMD_SENDER_COMMAND_VOLUME, WICED_TRUE, NULL, &volume);
                }
            }
            break;

        case APOLLO_CONSOLE_CMD_CONFIG:
            apollo_set_config(&g_apollo->dct_tables, argc, argv);
            break;

        case APOLLO_CONSOLE_CMD_LOG:
            log_level = atoi(argv[1]);
            wiced_log_printf("Setting new log level to %d (0 - off, %d - max debug)\n", log_level, WICED_LOG_DEBUG4);
            wiced_log_set_facility_level(WLF_AUDIO, log_level);
            break;

        case APOLLO_CONSOLE_CMD_TIME:
            /*
             * wiced_time_read_8021as() requires a valid audio sample rate in order to get a valid audio time;
             * however, if audio time is not required, audio sample rate MUST be set to 0
             */
            as_time.audio_sample_rate = 0;
            result = wiced_time_read_8021as(&as_time);

            if (result == WICED_SUCCESS)
            {
                wiced_time_t wtime;

                wiced_time_get_time(&wtime);
                wiced_log_printf("Master time = %u.%09u secs\n", (unsigned int)as_time.master_secs, (unsigned int)as_time.master_nanosecs);
                wiced_log_printf("Current local time = %u.%09u secs\n", (unsigned int)as_time.local_secs, (unsigned int)as_time.local_nanosecs);
                wiced_log_printf("wtime is %d\n", (int)wtime);
            }
            else
            {
                wiced_log_printf("Error returned from wiced_time_read_8021as\n");
            }
            break;

        case APOLLO_CONSOLE_CMD_STATS:
            if (g_apollo->player_handle)
            {
                if (apollo_player_ioctl(g_apollo->player_handle, APOLLO_PLAYER_IOCTL_GET_SOCKET, &socket) == WICED_SUCCESS)
                {
                    apollo_console_print_socket_info(&socket.socket);
                }

                if (apollo_player_ioctl(g_apollo->player_handle, APOLLO_PLAYER_IOCTL_GET_STATS, &stats) == WICED_SUCCESS)
                {
                    wiced_log_printf("Player stats: packets received %lu, packets dropped %lu\n", stats.rtp_packets_received, stats.rtp_packets_dropped);
                    wiced_log_printf("              total bytes received %lu, audio bytes received %lu\n", (uint32_t)stats.total_bytes_received, (uint32_t)stats.audio_bytes_received);
                    wiced_log_printf("              payload size %lu\n", stats.payload_size);
                    wiced_log_printf("Render stats: frames played %lu, dropped %lu, inserted %lu\n",
                                     (uint32_t)stats.audio_frames_played, (uint32_t)stats.audio_frames_dropped, (uint32_t)stats.audio_frames_inserted);
                    wiced_log_printf("              max early: %09ld, max late: %09ld, audio underruns: %lu\n",
                                     (int32_t)stats.audio_max_early, (int32_t)stats.audio_max_late, (uint32_t)stats.audio_underruns);
                }
                else
                {
                    memset(&stats, 0, sizeof(stats));
                }

                wiced_log_printf("Apollo stats: total packets received %lu, total packets dropped %lu\n",
                                 (uint32_t)(g_apollo->total_rtp_packets_received + stats.rtp_packets_received),
                                 (uint32_t)(g_apollo->total_rtp_packets_dropped + stats.rtp_packets_dropped));
                wiced_log_printf("              total audio frames played %lu, total underruns %lu\n",
                                 (uint32_t)(g_apollo->total_audio_frames_played + stats.audio_frames_played),
                                 (uint32_t)(g_apollo->total_audio_underruns + stats.audio_underruns));
                wiced_log_printf("              total audio dropped %lu, total frames inserted %lu\n",
                                 (uint32_t)(g_apollo->total_audio_frames_dropped + stats.audio_frames_dropped),
                                 (uint32_t)(g_apollo->total_audio_frames_inserted + stats.audio_frames_inserted));
            }
            else if (g_apollo->streamer_handle)
            {
                if (apollo_streamer_send_command(g_apollo->streamer_handle, APOLLO_STREAMER_COMMAND_GET_SOCKET, &socket_ptr, sizeof(socket_ptr)) == WICED_SUCCESS)
                {
                    apollo_console_print_socket_info(&(socket_ptr->socket));
                    wwd_get_counters(&counters);
                    wiced_log_printf("Good frames TX %u, TX retries %u, TX failures %u, TX errors %u.\n", counters.txframe, counters.txretry, counters.txfail, counters.txerror);
                }

                if (apollo_streamer_send_command(g_apollo->streamer_handle, APOLLO_STREAMER_COMMAND_GET_STATS, &streamer_stats, sizeof(streamer_stats)) == WICED_SUCCESS)
                {
                    wiced_log_printf("Streamer stats: packets sent %lu, total bytes sent %lu, audio frames sent %lu\n",
                                     (uint32_t)streamer_stats.rtp_packets_sent, (uint32_t)streamer_stats.total_bytes_sent, streamer_stats.audio_frames_sent);
                    wiced_log_printf("                audio frame size %hu, payload size %lu\n", streamer_stats.audio_frame_size, streamer_stats.payload_size);
                    wiced_log_printf("                Synth clock drift error max usecs     = %ld\n", (int32_t)(streamer_stats.synth_clock_drift_error_max_nsecs/1000LL));
                    wiced_log_printf("                Synth clock drift error min usecs     = %ld\n", (int32_t)(streamer_stats.synth_clock_drift_error_min_nsecs/1000LL));
                    wiced_log_printf("                Synth clock drift error current usecs = %ld\n", (int32_t)(streamer_stats.synth_clock_drift_error_current_nsecs/1000LL));
                    wiced_log_printf("                Media clock drift PPM                 = %ld\n", (int32_t)(streamer_stats.media_clock_drift_ppb/1000LL));
                    wiced_log_printf("Render stats:   frames played %lu, dropped %lu, inserted %lu\n",
                                     (uint32_t)streamer_stats.audio_frames_played, (uint32_t)streamer_stats.audio_frames_dropped, (uint32_t)streamer_stats.audio_frames_inserted);
                    wiced_log_printf("                max early: %09ld, max late: %09ld, audio underruns: %lu\n",
                                     (int32_t)streamer_stats.audio_max_early, (int32_t)streamer_stats.audio_max_late, (uint32_t)streamer_stats.audio_underruns);
                }
            }
            break;

        case APOLLO_CONSOLE_CMD_NETLOG:
            if (!strcmp(argv[1], "on"))
            {
                if (!g_apollo->network_logging_active)
                {
                    if (apollo_debug_create_tcp_data_socket(NULL, 0) == WICED_SUCCESS)
                    {
                        wiced_log_set_platform_output(apollo_debug_tcp_log_output_handler);
                        g_apollo->network_logging_active = WICED_TRUE;
                        {
                            wiced_8021as_time_t as_time;
                            wiced_8021as_time_t as_time2;

                            as_time.audio_sample_rate = 0;
                            as_time2.audio_sample_rate = 0;

                            wiced_time_read_8021as(&as_time);
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Network logging enabled\n");
                            wiced_time_read_8021as(&as_time2);
                            as_time.local_secs = as_time2.master_secs - as_time.master_secs;
                            if (as_time2.master_nanosecs < as_time.master_nanosecs)
                            {
                                as_time.local_secs--;
                                as_time.local_nanosecs = 1000000000 - as_time.master_nanosecs + as_time2.master_nanosecs;
                            }
                            else
                            {
                                as_time.local_nanosecs = as_time2.master_nanosecs - as_time.master_nanosecs;
                            }
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Log time delay is %lu.%09lu\n", as_time.local_secs, as_time.local_nanosecs);
                        }
                    }
                }
                else
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Network logging is already active\n");
                }
            }
            else if (g_apollo->network_logging_active)
            {
                wiced_log_set_platform_output(apollo_log_output_handler);
                apollo_debug_close_tcp_data_socket();
                g_apollo->network_logging_active = WICED_FALSE;
            }
            break;

        case APOLLO_CONSOLE_CMD_RMC_RATE:
        {
            /*
             * Usage:  Help: rmc_rate -help                          - gives help string
             *         Get:  rmc_rate [-2 | -5]                      - Default: both bands.
             *         Set:  rmc_rate [-2 | -5] legacy | -h ht_mcs   - Default: 5 Ghz band.
             */
            uint32_t rspec;
            char mybuf[32];
            int do_2 = 0, do_5 = 0;

            /*
             * Help string
             */
            if (argv[1] && (!strncmp(argv[1], "-help", 5) || !strncmp(argv[1], "-?", 2) || !strncmp(argv[1], "help", 4)))
            {
                wiced_log_printf("Get: rmc_rate [-2 | -5]:                     - Default: both bands.\n");
                wiced_log_printf("Set: rmc_rate [-2 | -5] legacy | -h ht_mcs   - Default: 5 Ghz band.\n");
                break;
            }

            /*
             * Parse bands
             */
            if (!argv[1]) {
                /* Get both bands if non specified */
                do_2++;
                do_5++;
            }
            if (argv[1] && !strncmp(argv[1], "-2", 2)) {
                do_2++;
                argv++;
                argc--;
            } else {
                if (argv[1] && !strncmp(argv[1], "-5", 2)) {
                    do_5++;
                    argv++;
                    argc--;
                } else {
                    /* When Setting, If no band specified, use 5 Ghz for backward compatability */
                    if (argv[1]) {
                        do_5++;
                    }
                }
            }

            /*
             * Handle 'Get's
             */
            if (!argv[1]) {
                if (do_2) {
                    rspec = -1;
                    if (apollo_wl_rmc_rate(&rspec, WLC_BAND_2G) < 0) {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_RATE: Cannot get 2 GHz rmc_rate\n");
                        break;
                    }
                    apollo_wl_rate_print(mybuf, sizeof(mybuf), rspec);
                    wiced_log_printf("2 Ghz: %s\n", mybuf);
                }
                if (do_5) {
                    rspec = -1;
                    if (apollo_wl_rmc_rate(&rspec, WLC_BAND_5G) < 0) {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_RATE: Cannot get 5 Ghz rmc_rate\n");
                        break;
                    }
                    apollo_wl_rate_print(mybuf, sizeof(mybuf), rspec);
                    wiced_log_printf("5 Ghz: %s\n", mybuf);
                }
                break;
            }

            /*
             * Handle 'Set's
             */
            rspec = 0;
            if (!strncmp("-h", argv[1], 2)) {
                if (argv[2] && atoi(argv[2])) {
                    rspec = WL_RSPEC_ENCODE_HT | atoi(argv[2]);
                }
            } else {
                /* Legacy rates are stored as number of .5 Mbit units */
                if (atoi(argv[1])) {
                    rspec = WL_RSPEC_ENCODE_RATE | (2 * atoi(argv[1]));
                }
            }
            if (!rspec) {
                wiced_log_printf("rmc_rate [-2|-5] [legacy rate | -h ht_mcs]\n");
                break;
            }
            if (do_2) {
                if (apollo_wl_rmc_rate(&rspec, WLC_BAND_2G) < 0) {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_RATE: Cannot set 2 Ghz rmc_rate\n");
                }
            }
            if (do_5) {
                if (apollo_wl_rmc_rate(&rspec, WLC_BAND_5G) < 0) {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_RATE: Cannot set 5 Ghz rmc_rate\n");
                }
            }
            break;
        }

        case APOLLO_CONSOLE_CMD_RMC_ACKREQ:
            if (argv[1]) {
                if (wwd_wifi_set_iovar_value(IOVAR_STR_RMC_ACKREQ, atoi(argv[1]), WICED_STA_INTERFACE) != WWD_SUCCESS) {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_ACKREQ: Cannot set rmc_ackreq\n");
                }
            } else {
                uint32_t ackreq;
                if (wwd_wifi_get_iovar_value(IOVAR_STR_RMC_ACKREQ, &ackreq, WICED_STA_INTERFACE) != WWD_SUCCESS) {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RMC_ACKREQ: Cannot get rmc_ackreq\n");
                } else {
                    wiced_log_printf("%d\n", ackreq & 0xf);
                }
            }
            break;

        case APOLLO_CONSOLE_CMD_RTP_DUMP:
        {
            uint8_t buf[512];

            wiced_log_printf("RTP Dump\n");
            if (wwd_wifi_get_iovar_buffer("rtp_dump", buf, sizeof(buf), WWD_STA_INTERFACE) != WWD_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "rtp_dump failed\n");
            }
            else
            {
                wiced_log_printf("%s", buf);
            }
            break;
        }

        case APOLLO_CONSOLE_CMD_RMC_COUNTERS:
        {
            wl_rmc_cnts_t rmc_stats;

            if (apollo_wl_get_rmc_counts(&rmc_stats) < 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from apollo_wl_get_rmc_counts\n");
                break;
            }

            if (rmc_stats.version != WL_RMC_CNT_VERSION) {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bad CNT_VERSION\n");
                break;
            }

            wiced_log_printf("Master:\r\n");

            wiced_log_printf(" Acker Timeouts \t%u \tNon-Acker Tmo \t%u \tNew Acker Selected \t%u\r\n",
                rmc_stats.mc_no_glb_slot, rmc_stats.mc_not_mirrored, rmc_stats.mc_not_exist_in_gbl);

            wiced_log_printf(" Mcast frames tx'ed \t%u\r\n", rmc_stats.rmc_tx_frames_mac);

            wiced_log_printf("Sinks:\r\n");

            wiced_log_printf(" Data frames rx'ed \t%u \tRetrys Rx'ed \t%u \tRetry %% %2.3f\r\n",
                rmc_stats.rmc_rx_frames_mac, rmc_stats.dupcnt, ((float)rmc_stats.dupcnt * 100)/(float)rmc_stats.rmc_rx_frames_mac);

            wiced_log_printf(" Action frames rx'ed \t%u \tFailed Null Tx \t%u\r\n",
                rmc_stats.mc_ar_role_selected, rmc_stats.null_tx_err);

            wiced_log_printf(" Time As Acker       \t%u\r\n", rmc_stats.mc_taken_other_tr);
            break;
        }

        case APOLLO_CONSOLE_CMD_RMC_STATUS:
        {
            wl_relmcast_status_t rmc_status;

            if (apollo_wl_get_rmc_status(&rmc_status) < 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from apollo_wl_get_rmc_status\n");
                break;
            }
            wiced_log_printf("%d peers associated\n", rmc_status.num);
            for (i = 0; i < rmc_status.num; i++)
            {
                wiced_log_printf("  %02x:%02x:%02x:%02x:%02x:%02x  %3d  %c %c %c %c\n",
                                 rmc_status.clients[i].addr.octet[0], rmc_status.clients[i].addr.octet[1],
                                 rmc_status.clients[i].addr.octet[2], rmc_status.clients[i].addr.octet[3],
                                 rmc_status.clients[i].addr.octet[4], rmc_status.clients[i].addr.octet[5],
                                 rmc_status.clients[i].rssi,
                                 rmc_status.clients[i].flag & WL_RMC_FLAG_ACTIVEACKER ? 'A' : ' ',
                                 rmc_status.clients[i].flag & WL_RMC_FLAG_MASTER_TX   ? 'M' : ' ',
                                 rmc_status.clients[i].flag & WL_RMC_FLAG_RELMCAST    ? 'R' : ' ',
                                 rmc_status.clients[i].flag & WL_RMC_FLAG_INBLACKLIST ? 'B' : ' ');
            }
            break;
        }

        case APOLLO_CONSOLE_CMD_REBOOT:
            wiced_framework_reboot();
            break;

        case APOLLO_CONSOLE_CMD_RESET_COUNTERS:
            if (wwd_reset_statistics_counters() == WWD_SUCCESS)
            {
                g_apollo->total_rtp_packets_dropped = 0;
            }
            else
            {
                wiced_log_printf("%s: command failed).\n", argv[0]);
            }
            break;

        case APOLLO_CONSOLE_CMD_GET_RX_COUNTERS:
            wwd_get_counters(&counters);
            wiced_log_printf("Good frames RX %u, RX errors %u. total RTP dropped %u\n",
                             counters.rxframe, counters.rxerror, (uint32_t)g_apollo->total_rtp_packets_dropped);
            wiced_log_printf("Bad PLCP %u, CRS glitch %u, Bad FCS %u, RX Fifo Overflow %u.\n",
                             counters.rxbadplcp, counters.rxcrsglitch, counters.rxbadfcs, counters.rxf0ovfl);
            break;

        case APOLLO_CONSOLE_CMD_GET_TX_COUNTERS:
            wwd_get_counters(&counters);
            wiced_log_printf("Good frames TX %u, TX retries %u, TX failures %u, TX errors %u.\n",
                             counters.txframe, counters.txretry, counters.txfail, counters.txerror);
            break;

        case APOLLO_CONSOLE_CMD_MEMORY:
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

#if defined(OTA2_SUPPORT)
        case APOLLO_CONSOLE_CMD_GET_UPDATE:
            if (argc > 1)
            {
                strlcpy(g_apollo->ota2_info.update_uri, argv[1], sizeof(g_apollo->ota2_info.update_uri) );
            }
            else
            {
                wiced_log_printf("Use app_dct->ota2_default_update_uri\n");
                strlcpy(g_apollo->ota2_info.update_uri, g_apollo->dct_tables.dct_app->ota2_default_update_uri, sizeof(g_apollo->ota2_info.update_uri) );
            }
            event = command_lookup[i].event;
            break;

        case APOLLO_CONSOLE_CMD_START_TIMED_UPDATE:
            if (argc > 1)
            {
                strlcpy(g_apollo->ota2_info.update_uri, argv[1], sizeof(g_apollo->ota2_info.update_uri) );
            }
            else
            {
                wiced_log_printf("Use app_dct->ota2_default_update_uri\n");
                strlcpy(g_apollo->ota2_info.update_uri, g_apollo->dct_tables.dct_app->ota2_default_update_uri, sizeof(g_apollo->ota2_info.update_uri) );
            }
            event = command_lookup[i].event;
            break;

        case APOLLO_CONSOLE_CMD_STOP_TIMED_UPDATE:
        case APOLLO_CONSOLE_CMD_OTA2_STATUS:
            event = command_lookup[i].event;
            break;
#endif

#if defined(USE_AMBILIGHT)
        case APOLLO_CONSOLE_CMD_AMBILIGHT:

            //apollo_cmd_ambilight_t cmd_ambilight;

            if(argc > 3)
            {
                /* prepare message */
                cmd_ambilight.amblt_front  = g_apollo->amblt_front;
                cmd_ambilight.amblt_source = g_apollo->amblt_source;
                cmd_ambilight.num_entries  = argc;
                cmd_ambilight.entries      = argv;

                /* sent message request */
                if (g_apollo->player_handle)
                {
                    wiced_log_printf("Ambient Light control available on Apollo SOURCE only.\n");
                }
                else if (g_apollo->streamer_handle)
                {
                    if (g_apollo->cmd_sender_handle != NULL)
                    {
                        apollo_cmd_sender_command(g_apollo->cmd_sender_handle, APOLLO_CMD_SENDER_COMMAND_AMBILIGHT , WICED_TRUE, NULL, &cmd_ambilight);
                    }
                    else
                    {
                        wiced_log_printf("cmd_sender is not ready (NULL).\n");
                    }
                }
                break;
            }
            else
            {
                    wiced_log_printf("Ambient Light syntax : amblt [amblt_cmd] [position] [delay] {cmd payload params}\n");
            }
#endif
    }

    if (event)
    {
        /*
         * Send off the event to the main loop.
         */

        wiced_rtos_set_event_flags(&g_apollo->events, event);
    }

    return ERR_CMD_OK;
}


wiced_result_t apollo_console_start(apollo_app_t *apollo)
{
    wiced_result_t result = WICED_SUCCESS;

    /*
     * Create the command console.
     */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Start the command console\n");

    command_console_cfg_t cfg;
    cfg.uart               = STDIO_UART;
    cfg.line_len           = sizeof(apollo_command_buffer);
    cfg.buffer             = apollo_command_buffer;
    cfg.history_len        = APOLLO_CONSOLE_COMMAND_HISTORY_LENGTH;
    cfg.history_buffer_ptr = apollo_command_history_buffer;
    cfg.delimiter_string   = apollo_console_delimiter_string;
    cfg.thread_priority    = WICED_DEFAULT_LIBRARY_PRIORITY+2;
    cfg.params_num         = APOLLO_CONSOLE_COMMAND_MAX_PARAMS;

    result = command_console_init_cfg(&cfg);
    if (result != WICED_SUCCESS)
    {
        return result;
    }
    console_add_cmd_table(apollo_command_table);

    return result;
}


wiced_result_t apollo_console_stop(apollo_app_t *apollo)
{
    /*
     * Shutdown the console.
     */

    command_console_deinit();

    return WICED_SUCCESS;
}
