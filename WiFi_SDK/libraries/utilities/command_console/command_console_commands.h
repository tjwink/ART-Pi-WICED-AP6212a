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
#include "command_console_ping.h"
#ifndef CONSOLE_DISABLE_WPS_COMMANDS
#include "command_console_wps.h"
#endif
#include "command_console.h"
#include "console_wl.h"
#include "wiced_management.h"
#include "command_console_wifi.h"
#ifndef CONSOLE_DISABLE_MALLINFO_COMMANDS
#include "command_console_mallinfo.h"
#endif
#include "console_iperf.h"
#include "command_console_thread.h"
#include "command_console_platform.h"
#ifndef CONSOLE_DISABLE_TRACEX_COMMANDS
#include "command_console_tracex.h"
#endif

#ifdef CONSOLE_INCLUDE_AUDIO
#include "command_console_audio.h"
#endif

#ifdef CONSOLE_INCLUDE_BT
#include "command_console_bt_hci.h"
#endif

#ifdef CONSOLE_INCLUDE_P2P
#include "command_console_p2p.h"
#endif

#ifdef CONSOLE_INCLUDE_ETHERNET
#include "command_console_ethernet.h"
#endif

#ifdef CONSOLE_ENABLE_WL
#include "console_wl.h"
#endif

#ifdef CONSOLE_INCLUDE_TRAFFIC_GENERATION
#include "command_console_traffic_generation.h"
#endif

#ifdef CONSOLE_INCLUDE_FILESYSTEM
#include "command_console_fs.h"
#endif

#ifdef CONSOLE_INCLUDE_BT_THROUGHPUT_TEST_COMMANDS
#include "command_console_bt.h"
#endif

#ifdef  WICED_POWER_LOGGER_ENABLE
#include "command_console_wpl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define MAX_LINE_LENGTH    (128)
#define MAX_HISTORY_LENGTH (20)

#ifdef CONSOLE_INCLUDE_P2P
#define ALL_COMMANDS_P2P_COMMANDS                P2P_COMMANDS
#else
#define ALL_COMMANDS_P2P_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_ETHERNET
#define ALL_COMMANDS_ETHERNET_COMMANDS           ETHERNET_COMMANDS
#else
#define ALL_COMMANDS_ETHERNET_COMMANDS
#endif

#ifdef CONSOLE_ENABLE_WL
#define ALL_COMMANDS_WL_COMMANDS                 WL_COMMANDS
#else
#define ALL_COMMANDS_WL_COMMANDS
#endif

#ifdef CONSOLE_DISABLE_ENTERPRISE_COMMANDS
#define ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS
#else
#define ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS           WIFI_ENTERPRISE_SECURITY_COMMANDS
#endif

#ifdef CONSOLE_DISABLE_WIFI_COMMANDS
#define ALL_COMMANDS_WIFI_COMMANDS
#else
#ifdef CONSOLE_INCLUDE_WIFI_LIMITED_SET_COMMANDS
#define ALL_COMMANDS_WIFI_COMMANDS     WIFI_COMMANDS_LIMITED_SET
#else
#define ALL_COMMANDS_WIFI_COMMANDS     WIFI_COMMANDS
#endif
#endif

#ifdef CONSOLE_DISABLE_WPS_COMMANDS
#define ALL_COMMANDS_WPS_COMMANDS
#else
#define ALL_COMMANDS_WPS_COMMANDS      WPS_COMMANDS
#endif

#ifdef CONSOLE_DISABLE_TRACEX_COMMANDS
#define ALL_COMMANDS_TRACEX_COMMANDS
#else
#define ALL_COMMANDS_TRACEX_COMMANDS   TRACEX_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_TRAFFIC_GENERATION
#define ALL_COMMANDS_TRAFFIC_GENERATION_COMMANDS TRAFFIC_GENERATION_COMMANDS
#else
#define ALL_COMMANDS_TRAFFIC_GENERATION_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_BT
#define ALL_COMMANDS_BT_COMMANDS       BT_COMMANDS
#else
#define ALL_COMMANDS_BT_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_AUDIO
#define ALL_COMMANDS_AUDIO_COMMANDS       AUDIO_COMMANDS
#else
#define ALL_COMMANDS_AUDIO_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_FILESYSTEM
#define ALL_COMMANDS_FS_COMMANDS       FS_COMMANDS
#else
#define ALL_COMMANDS_FS_COMMANDS
#endif

#ifdef CONSOLE_DISABLE_MALLINFO_COMMANDS
#define ALL_MALLINFO_COMMANDS
#else
#define ALL_MALLINFO_COMMANDS MALLINFO_COMMANDS
#endif

#ifdef CONSOLE_INCLUDE_BT_THROUGHPUT_TEST_COMMANDS
#define ALL_BT_THOUGHPUT_TEST_COMMANDS BT_THROUGHPUT_TEST_COMMANDS
#else
#define ALL_BT_THOUGHPUT_TEST_COMMANDS
#endif

#ifdef  WICED_POWER_LOGGER_ENABLE
#define ALL_WPL_COMMANDS       WPL_COMMANDS
#else
#define ALL_WPL_COMMANDS
#endif

#define ALL_COMMANDS                         \
    ALL_COMMANDS_WIFI_COMMANDS               \
    IPERF_COMMANDS                           \
    ALL_MALLINFO_COMMANDS                        \
    PING_COMMANDS                            \
    PLATFORM_COMMANDS                        \
    THREAD_COMMANDS                          \
    ALL_COMMANDS_WPS_COMMANDS                \
    ALL_COMMANDS_TRACEX_COMMANDS             \
    ALL_COMMANDS_P2P_COMMANDS                \
    ALL_COMMANDS_ETHERNET_COMMANDS           \
    ALL_COMMANDS_WL_COMMANDS                 \
    ALL_COMMANDS_TRAFFIC_GENERATION_COMMANDS \
    ALL_COMMANDS_BT_COMMANDS                 \
    ALL_COMMANDS_AUDIO_COMMANDS              \
    ALL_COMMANDS_FS_COMMANDS                 \
    ALL_BT_THOUGHPUT_TEST_COMMANDS           \
    ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS \
    FRAM_COMMANDS\
    ALL_WPL_COMMANDS

#ifdef __cplusplus
}
#endif
