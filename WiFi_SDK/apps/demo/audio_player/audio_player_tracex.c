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

/** @file Audio Player TraceX support routines
 *
 */

#include "wiced_log.h"
#include "command_console.h"

#if defined(TX_ENABLE_EVENT_TRACE)

#include "TraceX.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_PLAYER_TRACEX_TCP_SERVER_IP          MAKE_IPV4_ADDRESS(192,168,0,200)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_PLAYER_TRACEX_CMD_ENABLE,
    AUDIO_PLAYER_TRACEX_CMD_DISABLE,
    AUDIO_PLAYER_TRACEX_CMD_STATUS,
    AUDIO_PLAYER_TRACEX_CMD_SEND,
    AUDIO_PLAYER_TRACEX_CMD_HELP,

    AUDIO_PLAYER_TRACEX_CMD_MAX,
} AUDIO_PLAYER_TRACEX_CMD_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static char* audio_player_tracex_cmds[AUDIO_PLAYER_TRACEX_CMD_MAX] =
{
    "enable",
    "disable",
    "status",
    "send",
    "help"
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static int audio_player_tracex_send(int argc, char *argv[])
{
    wiced_result_t        result;
    wiced_tracex_config_t config;

    if (wiced_tracex_config_get(&config) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX configuration\n");
        return ERR_UNKNOWN;
    }

    if (argc > 2)
    {
        uint32_t a, b, c, d;

        /*
         * Use the specified an IP address.
         */

        if (sscanf(argv[2], "%lu.%lu.%lu.%lu", &a, &b, &c, &d) != 4)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid TCP server IP address\n");
            return ERR_INSUFFICENT_ARGS;
        }

        SET_IPV4_ADDRESS(config.tcp_server.ip, MAKE_IPV4_ADDRESS(a, b, c, d));
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Sending TraceX buffer to TCP server (%lu.%lu.%lu.%lu:%u)\n",
                  (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF, (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                  (GET_IPV4_ADDRESS(config.tcp_server.ip) >>  8) & 0xFF, (GET_IPV4_ADDRESS(config.tcp_server.ip) >>  0) & 0xFF,
                  config.tcp_server.port);

    result = wiced_tracex_buf_send(&config.tcp_server);
    if (result == WICED_BADVALUE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Cannot send TraceX buffer while TraceX is enabled\n");
        return ERR_UNKNOWN;
    }
    else if (result == WICED_NOT_CONNECTED)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to connect to TCP server\n");
        return ERR_UNKNOWN;
    }
    else if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to send TraceX buffer\n");
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}


static int audio_player_tracex_status(void)
{
    wiced_tracex_config_t config;
    wiced_tracex_status_t status;
    char                  str[WICED_TRACEX_FILTER_MAX_STRING_LEN];

    if (wiced_tracex_status_get(&status) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX status\n");
        return ERR_UNKNOWN;
    }

    if (wiced_tracex_config_get(&config) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX configuration\n");
        return ERR_UNKNOWN;
    }

    if (wiced_tracex_filter_mask_to_str(config.filter, str, sizeof(str)) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX filters\n");
        return ERR_UNKNOWN;
    }

    wiced_log_printf("TraceX state   : ");
    switch (status.state)
    {
        case WICED_TRACEX_STATE_DISABLED:
            wiced_log_printf("Disabled\n");
            break;

        case WICED_TRACEX_STATE_DISABLED_BUF_FULL:
            wiced_log_printf("Disabled (buffer full)\n");
            break;

        case WICED_TRACEX_STATE_DISABLED_TCP_ERR:
            wiced_log_printf("Disabled (TCP error)\n");
            break;

        case WICED_TRACEX_STATE_DISABLED_TRACEX_ERR:
            wiced_log_printf("Disabled (TraceX error)\n");
            break;

        case WICED_TRACEX_STATE_ENABLED:
            wiced_log_printf("Enabled\n");
            break;

        default:
            wiced_log_printf("Unknown\n");
            break;
    }

    wiced_log_printf("Buffer         : %u bytes @ %p with %lu objects\n",
                     config.buf.size, config.buf.addr, config.buf.obj_cnt);
    wiced_log_printf("Loop recording : %s\n",
                     config.loop_rec == WICED_TRUE ? "Enabled" : "Disabled");
    wiced_log_printf("Filters        : '%s' (0x%08lx)\n", str, config.filter);
    wiced_log_printf("TCP server     : %lu.%lu.%lu.%lu:%u (%s); max data length of %lu bytes\n",
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF,
                     config.tcp_server.port,
                     config.tcp_server.enable == WICED_FALSE ? "Disabled" :
                     status.tcp_connected == WICED_TRUE ? "Connected" :
                     "Disconnected", config.tcp_server.max_data_len);

    return ERR_CMD_OK;
}


static int audio_player_tracex_disable(void)
{
    wiced_tracex_status_t status;

    if (wiced_tracex_status_get(&status) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX status\n");
        return ERR_UNKNOWN;
    }

    if (status.state < WICED_TRACEX_STATE_ENABLED)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "TraceX not running\n");
        return ERR_CMD_OK;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Disabling TraceX\n");
    if (wiced_tracex_disable() != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to disable TraceX\n");
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}


static int audio_player_tracex_enable(int argc, char *argv[])
{
    wiced_result_t        result;
    wiced_tracex_config_t config;

    /* Start with current config and just change the things we care about */
    if (wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get TraceX configuration\n");
        return ERR_UNKNOWN;
    }

    /*
     * Configure the TCP server address. When the TraceX buffer is full, it will be automatically
     * sent to the TCP server.
     */

    config.loop_rec          = WICED_FALSE;
    config.tcp_server.enable = WICED_TRUE;

    if (argc > 2)
    {
        uint32_t a, b, c, d;

        /*
         * Looks like the caller specified an IP address.
         */

        if (sscanf(argv[2], "%lu.%lu.%lu.%lu", &a, &b, &c, &d) != 4)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid TCP server IP address\n");
            return ERR_INSUFFICENT_ARGS;
        }

        SET_IPV4_ADDRESS(config.tcp_server.ip, MAKE_IPV4_ADDRESS(a, b, c, d));
    }
    else
    {
        /*
         * Use the default IP address.
         */

        SET_IPV4_ADDRESS(config.tcp_server.ip, AUDIO_PLAYER_TRACEX_TCP_SERVER_IP);
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Enabling TraceX\n");
    result = wiced_tracex_enable(&config);

    if (result == WICED_NOT_CONNECTED)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to connect to TCP server (%lu.%lu.%lu.%lu:%u)\n",
                       (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF, (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                       (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,  (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF, config.tcp_server.port);
        return ERR_UNKNOWN;
    }
    else if (result == WICED_BADVALUE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "TraceX was already enabled\n");
        return ERR_UNKNOWN;
    }
    else if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to enable TraceX\n");
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}


int audio_player_tracex_command(int argc, char *argv[])
{
    int i;

    if (argc < 2)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No TraceX command specified\n");
        return ERR_CMD_OK;
    }

    /*
     * Look the command requested.
     */

    for (i = 0; i < AUDIO_PLAYER_TRACEX_CMD_MAX; i++)
    {
        if (!strcasecmp(argv[1], audio_player_tracex_cmds[i]))
        {
            break;
        }
    }

    if (i >= AUDIO_PLAYER_TRACEX_CMD_MAX)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized TraceX command: %s\n", argv[1]);
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case AUDIO_PLAYER_TRACEX_CMD_ENABLE:
            audio_player_tracex_enable(argc, argv);
            break;

        case AUDIO_PLAYER_TRACEX_CMD_DISABLE:
            audio_player_tracex_disable();
            break;

        case AUDIO_PLAYER_TRACEX_CMD_STATUS:
            audio_player_tracex_status();
            break;

        case AUDIO_PLAYER_TRACEX_CMD_SEND:
            audio_player_tracex_send(argc, argv);
            break;

        case AUDIO_PLAYER_TRACEX_CMD_HELP:
            wiced_log_printf("Audio Player TraceX commands:\n");
            for (i = 0; i < AUDIO_PLAYER_TRACEX_CMD_MAX; i++)
            {
                wiced_log_printf("  %s\n", audio_player_tracex_cmds[i]);
            }
            break;

        default:
            break;
    }

    return ERR_CMD_OK;
}

#endif
