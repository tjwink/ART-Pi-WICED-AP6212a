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
 * OTA Image test Application
 *
 * Over
 * The
 * Air
 *
 * Update Test
 *
 * This application snippet demonstrates how to use the WICED
 * interface for performing Over The Air Updates to your device
 *
 * Application Instructions
 * 1. wifi_config_dct.h: Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the header file to match your Wi-Fi access point
 * 2. ota2_test_dct.h: Modify the OTA2_UPDATE_SSID/OTA2_UPDATE_PASSPHRASE Wi-Fi credentials
 *    in the header file to match your Wi-Fi access point for connecting to your OTA2 image file server
 *    NOTE: If both APs are the same, use NULL in ota2_test_get_update() in this file:
 *
 *    player->ota2_bg_params.ota2_ap_info = NULL;
 *
 * 3. Connect a PC terminal to the serial port of the WICED Eval board,
 *    then build and download the application as described in the WICED
 *    Quick Start Guide
 *
 * After the download completes, it connects to the Wi-Fi AP specified in apps/snip/ota2_test/wifi_config_dct.h
 *
 * When you issue a "get_update <host>/ota2_image_file" the app will connect directly ONCE to download the OTA2 Image file
 *                  <ota2_image_file> from <host>.
 *
 * When you issue a "timed_update" <host>/ota2_image_file" the app will connect to download the OTA2 Image file
 *                  <ota2_image_file> from <host> on a regularly timed basis
 *                  See ota2_test_get_update() in this file.
 *
 *  case OTA2_EXAMPLE_UPDATE_TYPE_TIMED:
 *      player->ota2_bg_params.initial_check_interval   = 5;            <-- start in 5 seconds
 *      player->ota2_bg_params.check_interval           = 5 * 60;       <-- regularly timed check every 5 minutes
 *      player->ota2_bg_params.retry_check_interval     = 5;            <-- on Failure, retry in 5 seconds
 *
 */

#include <unistd.h>
#include <ctype.h>

#include "wiced.h"
#include "wiced_log.h"
#include "wiced_tcpip.h"
#include "platform.h"
#include "command_console.h"
#include "command_console_wifi.h"
#include "command_console_dct.h"
#include "console_wl.h"
#include "resources.h"
#include "internal/wwd_sdpcm.h"
#include "wiced_dct_common.h"

#include "ota2_test.h"
#include "ota2_test_config.h"

#ifdef WWD_TEST_NVRAM_OVERRIDE
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#endif

#include "wiced_ota2_image.h"
#include "wiced_ota2_service.h"
#include "wiced_ota2_network.h"
#include "../../WICED/internal/wiced_internal_api.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return WWD_BUFFER_ALLOC_FAIL; }
#define CHECK_RETURN( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }

#define Mod32_GT( A, B )        ( (int32_t)( ( (uint32_t)( A ) ) - ( (uint32_t)( B ) ) ) >   0 )

#define OTA2_CONSOLE_COMMANDS \
    { (char*) "exit",           ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Exit application" }, \
    { (char*) "log",            ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Set log level" }, \
    { (char*) "get_update",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Get OTA2 update - use connection" }, \
    { (char*) "timed_update",   ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Get Timed OTA2 update" }, \
    { (char*) "stop_update",    ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Stop Timed OTA2 update" }, \
    { (char*) "factory_status", ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Factory Reset - show status" }, \
    { (char*) "factory_reboot", ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Factory Reset on Reboot" }, \
    { (char*) "factory_now",    ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Factory Reset - extract NOW" }, \
    { (char*) "update_now",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"OTA2 update - Update from staging now" }, \
    { (char*) "update_reboot",  ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"OTA2 update - Update from staging on boot" }, \
    { (char*) "update_status",  ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"OTA2 update - show status / valid" }, \
    { (char*) "config",         ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Display / change config values" }, \
    { (char*) "disconnect",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Disconnect from AP" }, \
    { (char*) "default_ap",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Join Default AP" }, \
    { (char*) "ota2_ap",        ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Join OTA2 AP" }, \
    { (char*) "status",         ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"show status" }, \
    { (char*) "wlog",           read_wlan_chip_console_log,         0, NULL, NULL, (char*) "",                                           (char*) "Dump WLAN chip console log"}, \


/******************************************************
 *                    Constants
 ******************************************************/

#define MY_DEVICE_NAME                      "ota2_test"
#define MY_DEVICE_MODEL                     "1.0"
#define MAX_COMMAND_LENGTH                   (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH      (10)

#define FIRMWARE_VERSION                    "wiced-1.0"

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    OTA2_CONSOLE_CMD_EXIT = 0,

    OTA2_CONSOLE_CMD_CONFIG,

    OTA2_CONSOLE_CMD_DISCONNECT_AP,
    OTA2_CONSOLE_CMD_CONNECT_DEFAULT_AP,
    OTA2_CONSOLE_CMD_CONNECT_OTA2_AP,
    OTA2_CONSOLE_CMD_STATUS,

    OTA2_CONSOLE_CMD_GET_UPDATE,
    OTA2_CONSOLE_CMD_GET_TIMED_UPDATE,
    OTA2_CONSOLE_CMD_STOP_TIMED_UPDATE,

    OTA2_CONSOLE_CMD_FACTORY_RESET_STATUS,
    OTA2_CONSOLE_CMD_FACTORY_RESET_REBOOT,
    OTA2_CONSOLE_CMD_FACTORY_NOW,

    OTA2_CONSOLE_CMD_UPDATE_NOW,
    OTA2_CONSOLE_CMD_UPDATE_REBOOT,
    OTA2_CONSOLE_CMD_UPDATE_STATUS,

    OTA2_CONSOLE_CMD_LOG_LEVEL,


    OTA2_CONSOLE_CMD_MAX,
} OTA2_CONSOLE_CMDS_T;

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

int ota2_console_command(int argc, char *argv[]);

/******************************************************
 *               Variables Definitions
 ******************************************************/
static wiced_time_t ota2_test_start_time;

#define WICED_OTA2_BUFFER_NODE_COUNT         (256)
#define RESTORE_DCT_APP_SETTINGS             (1)

#ifdef PLATFORM_L1_CACHE_BYTES
#define NUM_BUFFERS_POOL_SIZE(x)       ((WICED_LINK_MTU_ALIGNED + sizeof(wiced_packet_t) + 1) * (x))
#define APP_RX_BUFFER_POOL_SIZE        NUM_BUFFERS_POOL_SIZE(WICED_OTA2_BUFFER_NODE_COUNT)
#endif

#ifdef PLATFORM_L1_CACHE_BYTES
uint8_t                          ota2_rx_packets[APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES]        __attribute__ ((section (".external_ram")));
#else
uint8_t                          ota2_rx_packets[WICED_NETWORK_MTU_SIZE * WICED_OTA2_BUFFER_NODE_COUNT]     __attribute__ ((section (".external_ram")));
#endif

static char ota2_command_buffer[MAX_COMMAND_LENGTH];
static char ota2_command_history_buffer[MAX_COMMAND_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

uint8_t ota2_thread_stack_buffer[OTA2_THREAD_STACK_SIZE]                               __attribute__ ((section (".bss.ccm")));

const command_t ota2_command_table[] = {
    OTA2_CONSOLE_COMMANDS
    WL_COMMANDS
    DCT_CONSOLE_COMMANDS
    WIFI_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[OTA2_CONSOLE_CMD_MAX] = {
        { "exit",           PLAYER_EVENT_SHUTDOWN             },
        { "config",         0                                 },
        { "disconnect",     PLAYER_EVENT_DISCONNECT_AP        },
        { "default_ap",     PLAYER_EVENT_CONNECT_DEFAULT_AP   },
        { "ota2_ap",        PLAYER_EVENT_CONNECT_OTA2_AP      },
        { "status",         PLAYER_EVENT_STATUS               },
        { "get_update",     PLAYER_EVENT_GET_UPDATE           },
        { "timed_update",   PLAYER_EVENT_GET_TIMED_UPDATE     },
        { "stop_update",    PLAYER_EVENT_STOP_TIMED_UPDATE    },
        { "factory_status", PLAYER_EVENT_FACTORY_RESET_STATUS },
        { "factory_reboot", PLAYER_EVENT_FACTORY_RESET_REBOOT },
        { "factory_now",    PLAYER_EVENT_FACTORY_RESET_NOW    },
        { "update_now",     PLAYER_EVENT_UPDATE_NOW           },
        { "update_reboot",  PLAYER_EVENT_UPDATE_REBOOT        },
        { "update_status",  PLAYER_EVENT_UPDATE_STATUS        },
        { "log",            0                                 },
};

#ifdef CONNECTING_TO_ADHOC_NETWORK
/* for when we are connecting to an ADHOC network */
static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};
#endif /* #ifdef CONNECTING_TO_ADHOC_NETWORK */

/* template for HTTP GET */
char ota2_get_request_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s%s \r\n"
    "\r\n"
};

const char* firmware_version = FIRMWARE_VERSION;

ota2_data_t *g_player;

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t over_the_air_2_app_restore_settings_after_update( ota2_data_t* player, ota2_boot_type_t boot_type );

static ota2_data_t* init_player(void);
static void ota2_test_mainloop(ota2_data_t *player);
static void ota2_test_shutdown(ota2_data_t *player)
;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    ota2_data_t*   player;

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
    ota2_test_mainloop(player);

    /*
     * Cleanup and exit.
     */

    g_player = NULL;
    ota2_test_shutdown(player);
    player = NULL;
}

/****************************************************************
 *  Console command Function Declarations
 ****************************************************************/
int printf_memory( char *message, uint8_t*addr, uint16_t length)
{
    uint16_t offset, i = 0;
    if (addr == NULL)
        return 0;

    printf("\r\nMemory  addr:%p len:%d dump: %s\r\n", addr, length, message);
    for (offset = 0; offset < length; offset += 16)
    {
        printf("%p  ", &addr[offset]);
        for (i= offset; (i < (offset + 16)) && (i < length); i++)
        {
            printf("%02x ", addr[i]);
        }
        printf("    ");
        for (i= offset; (i < (offset + 16)) && (i < length); i++)
        {
            printf("%c ", (isprint(addr[i]) ? addr[i] : ' '));
        }
        printf("\r\n");
    }

    return offset + i;
}



int ota2_console_command(int argc, char *argv[])
{
    uint32_t event = 0;
    int i;

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Received command: %s\n", argv[0]);

    if (g_player == NULL || g_player->tag != PLAYER_TAG_VALID)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "ota2_console_command() Bad player structure\r\n");
        return ERR_CMD_OK;
    }

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < OTA2_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= OTA2_CONSOLE_CMD_MAX)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Unrecognized command: %s\n", argv[0]);
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case OTA2_CONSOLE_CMD_EXIT:
            break;
        case OTA2_CONSOLE_CMD_DISCONNECT_AP:
        case OTA2_CONSOLE_CMD_CONNECT_DEFAULT_AP:
        case OTA2_CONSOLE_CMD_CONNECT_OTA2_AP:
        case OTA2_CONSOLE_CMD_STATUS:
            event = command_lookup[i].event;
            break;
        case OTA2_CONSOLE_CMD_GET_UPDATE:
            memset(g_player->uri_to_stream, 0, sizeof(g_player->uri_to_stream));
            if (argc > 1)
            {
                strlcpy(g_player->uri_to_stream, argv[1], (sizeof(g_player->uri_to_stream) - 1) );
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Use app_dct->ota2_update_uri\r\n");
                strlcpy(g_player->uri_to_stream, g_player->dct_app->ota2_update_uri, (sizeof(g_player->uri_to_stream) - 1) );
            }
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Get update file:%s\r\n", g_player->uri_to_stream);
            event = command_lookup[i].event;
            break;
        case OTA2_CONSOLE_CMD_GET_TIMED_UPDATE:
            memset(g_player->uri_to_stream, 0, sizeof(g_player->uri_to_stream));
            if (argc > 1)
            {
                strlcpy(g_player->uri_to_stream, argv[1], sizeof(g_player->uri_to_stream) - 1 );
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Use app_dct->ota2_update_uri\r\n");
                strlcpy(g_player->uri_to_stream, g_player->dct_app->ota2_update_uri, (sizeof(g_player->uri_to_stream) - 1) );
            }
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Get update using BG service uri:%s \r\n", g_player->uri_to_stream);
            event = command_lookup[i].event;
            break;
        case OTA2_CONSOLE_CMD_LOG_LEVEL:
            if (argc > 1)
            {
                int new_level;
                new_level = atoi(argv[1]);
                if ((new_level > 0) && (new_level < WICED_LOG_DEBUG4))
                {
                    g_player->log_level = new_level;
                    wiced_log_set_all_levels(new_level);
                }
            }
            wiced_log_printf( "log level = %d\r\n", g_player->log_level);
            break;
        case OTA2_CONSOLE_CMD_FACTORY_RESET_STATUS:
        case OTA2_CONSOLE_CMD_FACTORY_RESET_REBOOT:
        case OTA2_CONSOLE_CMD_FACTORY_NOW:
        case OTA2_CONSOLE_CMD_UPDATE_NOW:
        case OTA2_CONSOLE_CMD_UPDATE_REBOOT:
        case OTA2_CONSOLE_CMD_UPDATE_STATUS:
        case OTA2_CONSOLE_CMD_STOP_TIMED_UPDATE:
            event = command_lookup[i].event;
            break;

        case OTA2_CONSOLE_CMD_CONFIG:
            ota2_set_config(g_player, argc, argv);
            break;
    }

    if (event)
    {
        /*
         * Send off the event to the main loop.
         */

        wiced_rtos_set_event_flags(&g_player->events, event);
    }

    return ERR_CMD_OK;
}

wiced_result_t my_ota2_callback(void* session_id, wiced_ota2_service_status_t status, uint32_t value, void* opaque )
{
    ota2_data_t*    player = (ota2_data_t*)opaque;

    UNUSED_PARAMETER(session_id);
    UNUSED_PARAMETER(player);


    switch( status )
    {
    case OTA2_SERVICE_STARTED:      /* Background service has started
                                         * return - None                                                             */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : SERVE STARTED -----------------------------\r\n");
        break;
    case OTA2_SERVICE_AP_CONNECT_ERROR:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : AP CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_SERVER_CONNECT_ERROR:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : SERVER_CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_AP_CONNECTED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : AP_CONNECTED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_SERVER_CONNECTED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : SERVER_CONNECTED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        break;


    case OTA2_SERVICE_CHECK_FOR_UPDATE: /* Time to check for updates.
                                         * return - WICED_SUCCESS = Service will check for update availability
                                         *        - WICED_ERROR   = Application will check for update availability   */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : CHECK_FOR_UPDATE -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCCESS, let Service do the checking.\r\n");
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_AVAILABLE: /* Service has contacted server, update is available
                                         * return - WICED_SUCCESS = Application indicating that it wants the
                                         *                           OTA Service to perform the download
                                         *        - WICED_ERROR   = Application indicating that it will perform
                                         *                           the download, the OTA Service will do nothing.  */
    {
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */
        wiced_ota2_image_header_t* ota2_header;

        ota2_header = (wiced_ota2_image_header_t*)value;

        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : UPDATE_AVAILABLE -----------------------------\r\n");
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */

        /*
         * In an actual application, the application would look at the headers information and decide if the
         * file on the update server is a newer version that the currently running application.
         *
         * If the application wants the update to continue, it would return WICED_SUCCESS here
         * If not, return WICED_ERROR
         *
         */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Current Version %d.%d\r\n", player->dct_app->ota2_major_version,
                                                                      player->dct_app->ota2_minor_version);
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "   OTA2 Version %d.%d\r\n", ota2_header->major_version, ota2_header->minor_version);

#if defined(CHECK_OTA2_UPDATE_VERSION)
        if ((player->dct_app->ota2_major_version > ota2_header->major_version) ||
            ((player->dct_app->ota2_major_version == ota2_header->major_version) &&
             (player->dct_app->ota2_minor_version >= ota2_header->minor_version)) )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "OTA2 Update Version Fail - return ERROR, do not update!\r\n");
            return WICED_ERROR;
        }
#endif
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCCESS, let Service perform the download.\r\n");
        return WICED_SUCCESS;
    }

    case OTA2_SERVICE_DOWNLOAD_STATUS:  /* Download status - value has % complete (0-100)
                                         *   NOTE: This will only occur when Service is performing download
                                         * return - WICED_SUCCESS = Service will continue download
                                         *        - WICED_ERROR   = Service will STOP download and service will
                                         *                          issue OTA2_SERVICE_TIME_TO_UPDATE_ERROR           */
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "my_ota2_callback() OTA2_SERVICE_DOWNLOAD_STATUS %ld %%!\r\n", value);
        return WICED_SUCCESS;

    case OTA2_SERVICE_PERFORM_UPDATE:   /* Download is complete
                                         * return - WICED_SUCCESS = Service will inform Bootloader to extract
                                         *                          and update on next power cycle
                                         *        - WICED_ERROR   = Service will inform Bootloader that download
                                         *                          is complete - Bootloader will NOT extract        */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : PERFORM_UPDATE -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCCESS, let Service extract update on next reboot.\r\n");
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ERROR:     /* There was an error in transmission
                                         * This will only occur if Error during Service performing data transfer
                                         * upon return - if retry_interval is set, Service will use retry_interval
                                         *               else, Service will retry on next check_interval
                                         */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : UPDATE_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCCESS, Service will retry as parameters defined in init.\r\n");
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ENDED:     /* All update actions for this check are complete.
                                         * This callback is to allow the application to take any actions when
                                         * the service is done checking / downloading an update
                                         * (succesful, unavailable, or error)
                                         * return - None                                                             */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : UPDATE_ENDED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        if ((value == WICED_SUCCESS) && (player->dct_app->ota2_reboot_after_download != 0))
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "        REBOOTING !!!\r\n");
            wiced_framework_reboot();
        }

        if (player->deinit_ota2_bg == WICED_TRUE)
        {
            wiced_rtos_set_event_flags(&player->events, PLAYER_EVENT_STOP_TIMED_UPDATE);
        }
        break;

    case OTA2_SERVICE_STOPPED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : SERVICE ENDED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "        return SUCESS (not used by service). This is informational \r\n");
        break;

    default:
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "my_ota2_callback() UNKNOWN STATUS %d!\r\n", status);
        break;
    }

    return WICED_SUCCESS;
}

wiced_result_t ota2_test_stop_timed_update(ota2_data_t *player)
{
    wiced_result_t                          result;

    result = wiced_ota2_service_deinit(player->ota2_bg_service);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "wiced_ota2_service_deinit() returned:%d\r\n", result);
    }
    player->ota2_bg_service = NULL;
    return result;
}

wiced_result_t ota2_test_get_update(ota2_data_t* player)
{
    wiced_result_t result = WICED_ERROR;

    /* get the image from the server & save in staging area */

    wiced_ota2_service_uri_split(player->uri_to_stream, player->ota2_host_name, sizeof(player->ota2_host_name),
                                     player->ota2_file_path, sizeof(player->ota2_file_path), &player->ota2_bg_params.port);

    player->ota2_bg_params.host_name                = player->ota2_host_name;
    player->ota2_bg_params.file_path                = player->ota2_file_path;

    player->ota2_bg_params.auto_update              = 0;
    player->ota2_bg_params.initial_check_interval   = 5;            /* initial check in 5 seconds */
    player->ota2_bg_params.check_interval           = 10 * 60;      /* 10 minutes - use SECONDS_IN_24_HOURS for 1 day */
    player->ota2_bg_params.retry_check_interval     = SECONDS_PER_MINUTE;   /* minimum retry is 1 minute */
    player->ota2_bg_params.max_retries              = 3;       /* maximum retries per update attempt         */
    player->ota2_bg_params.default_ap_info          = player->dct_wifi->stored_ap_list;
    player->ota2_bg_params.ota2_interface           = player->dct_network->interface;
#ifdef WICED_USE_ETHERNET_INTERFACE
    if (player->ota2_bg_params.ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        if ( wiced_network_is_up( WICED_ETHERNET_INTERFACE) == WICED_FALSE )
        {
            /* Currently not connected to Ethernet, use WiFI */
            player->ota2_bg_params.ota2_interface = WICED_STA_INTERFACE;
        }
    }
#endif
    if (player->ota2_bg_params.ota2_interface != WICED_ETHERNET_INTERFACE)
    {
        player->ota2_bg_params.ota2_ap_info             = NULL;
        player->ota2_bg_params.ota2_ap_list             = &player->dct_wifi->stored_ap_list[0]; /* use the DCT AP list */
        player->ota2_bg_params.ota2_ap_list_count       = CONFIG_AP_LIST_SIZE;
    }

    player->deinit_ota2_bg = WICED_TRUE;
    if (player->ota2_bg_service == NULL)
    {
        player->ota2_bg_service = wiced_ota2_service_init(&player->ota2_bg_params, player);
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_get_update() wiced_ota2_service_init() bg_service:%p \r\n", player->ota2_bg_service);
    }
    else
    {
        /* bg service already started - this is OK, just don't deinit at the end of this function */
        player->deinit_ota2_bg = WICED_FALSE;
    }

    if (player->ota2_bg_service != NULL)
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(player->ota2_bg_service, my_ota2_callback);
        if (result != WICED_SUCCESS)
        {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_get_update register callback failed! %d \r\n", result);
                wiced_ota2_service_deinit(player->ota2_bg_service);
                player->ota2_bg_service = NULL;
        }

        if (player->ota2_bg_service != NULL)
        {
            wiced_log_printf( "Download the OTA Image file - get it NOW!\r\n");
            /* NOTE: THis is a blocking call! */
            result = wiced_ota2_service_check_for_updates(player->ota2_bg_service);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_get_update wiced_ota2_service_check_for_updates() failed! %d \r\n", result);
            }
        }
    }
    return result;
}

wiced_result_t ota2_test_start_timed_update(ota2_data_t* player)
{
    wiced_result_t result = WICED_ERROR;

    /* get the image from the server & save in staging area */

    wiced_ota2_service_uri_split(player->uri_to_stream, player->ota2_host_name, sizeof(player->ota2_host_name),
                                     player->ota2_file_path, sizeof(player->ota2_file_path), &player->ota2_bg_params.port);

    player->ota2_bg_params.host_name                = player->ota2_host_name;
    player->ota2_bg_params.file_path                = player->ota2_file_path;
    player->ota2_bg_params.auto_update              = 0;
    player->ota2_bg_params.initial_check_interval   = 5;            /* initial check in 5 seconds */
    player->ota2_bg_params.check_interval           = 10 * 60;      /* 10 minutes - use SECONDS_IN_24_HOURS for 1 day */
    player->ota2_bg_params.retry_check_interval     = SECONDS_PER_MINUTE;   /* minimum retry is 1 minute */
    player->ota2_bg_params.max_retries              = 3;       /* maximum retries per update attempt         */
    player->ota2_bg_params.default_ap_info          = player->dct_wifi->stored_ap_list;
    player->ota2_bg_params.ota2_interface           = player->dct_network->interface;
#ifdef WICED_USE_ETHERNET_INTERFACE
    if (player->ota2_bg_params.ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        if ( wiced_network_is_up( WICED_ETHERNET_INTERFACE) == WICED_FALSE )
        {
            /* Currently not connected to Ethernet, use WiFI */
            player->ota2_bg_params.ota2_interface = WICED_STA_INTERFACE;
        }
    }
#endif
    if (player->ota2_bg_params.ota2_interface != WICED_ETHERNET_INTERFACE)
    {
        player->ota2_bg_params.ota2_ap_info             = NULL;
        player->ota2_bg_params.ota2_ap_list             = &player->dct_wifi->stored_ap_list[0]; /* use the DCT AP list */
        player->ota2_bg_params.ota2_ap_list_count       = CONFIG_AP_LIST_SIZE;
    }

    if (player->ota2_bg_service != NULL)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "OTA2 session already inited, deinit() previous session!\r\n");
        wiced_ota2_service_deinit(player->ota2_bg_service);
        player->ota2_bg_service = NULL;
    }

    if (player->ota2_bg_service == NULL)
    {
        player->ota2_bg_service = wiced_ota2_service_init(&player->ota2_bg_params, player);
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_start_timed_update() wiced_ota2_service_init() bg_service:%p \r\n", player->ota2_bg_service);
    }
    if (player->ota2_bg_service != NULL)
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(player->ota2_bg_service, my_ota2_callback);
        if (result != WICED_SUCCESS)
        {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_start_timed_update register callback failed! %d \r\n", result);
                wiced_ota2_service_deinit(player->ota2_bg_service);
                player->ota2_bg_service = NULL;
        }

        if (player->ota2_bg_service != NULL)
        {
            wiced_log_printf( "Download the OTA Image file - get it Timed !\r\n");
            /* NOTE: This is a non-blocking call (async) */
            result = wiced_ota2_service_start(player->ota2_bg_service);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_test_start_timed_update wiced_ota2_service_start() failed! %d \r\n", result);
            }
            /* do not de-init the service - it needs to be running for background update to work ! */
        }
    }
    return result;
}

/****************************************************************
 *  Application Main loop Function
 ****************************************************************/

static void ota2_test_mainloop(ota2_data_t *player)
{
    wiced_result_t      result;
    uint32_t            events;

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Begin ota2_test mainloop\r\n");
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
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "mainloop received EVENT_SHUTDOWN\r\n");
            break;
        }

        if (events & PLAYER_EVENT_DISCONNECT_AP)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "mainloop received PLAYER_EVENT_CONNECT_OTA2_AP\r\n");

            /* disconnect from current AP and connect to OTA2 update AP */
            /* drop the current network interface */
            player->ota2_bg_params.ota2_interface = player->dct_network->interface;
            result = wiced_ota2_network_down( player->ota2_bg_params.ota2_interface);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2_SERVICE_CHECK_FOR_UPDATE: Bringing down network interface failed!\r\n");
            }
            wiced_ota2_service_app_network_is_down(player->ota2_bg_service);
        }

        if (events & PLAYER_EVENT_CONNECT_OTA2_AP)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "mainloop received PLAYER_EVENT_CONNECT_OTA2_AP\r\n");

            if(player->dct_wifi->stored_ap_list[0].details.SSID.length == 0)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_CONNECT_OTA2_AP: No secondary AP defined!\r\n");
            }
            else
            {
                /* disconnect from current AP and connect to OTA2 update AP */
                player->ota2_bg_params.ota2_interface = player->dct_network->interface;
                result = wiced_ota2_network_up(player->ota2_bg_params.ota2_interface, player->dct_wifi->stored_ap_list);
                if (result != WICED_SUCCESS)
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_CONNECT_OTA2_AP: Bringing up network interface failed!\r\n");
                }
            }
        }

        if (events & PLAYER_EVENT_CONNECT_DEFAULT_AP)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "mainloop received PLAYER_EVENT_CONNECT_DEFAULT_AP\r\n");

            /* Bring up the default network interface */
            player->ota2_bg_params.ota2_interface = player->dct_network->interface;
            result = wiced_ota2_network_up(player->ota2_bg_params.ota2_interface, player->dct_wifi->stored_ap_list);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_CONNECT_DEFAULT_AP: Bringing up network interface failed!\r\n");
            }
        }

        if (events & PLAYER_EVENT_STATUS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "mainloop received connect STATUS\r\n");
            if (player->ota2_bg_service != NULL)
            {
                if (wiced_ota2_service_status(player->ota2_bg_service) != WICED_SUCCESS)
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "OTA2 Service Info: No service running.\r\n");
                }
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "OTA2 Service Info: service not initialized !\r\n");
            }
        }

        if (events & PLAYER_EVENT_GET_UPDATE)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_GET_UPDATE ! \r\n");
            result = ota2_test_get_update(player);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_GET_UPDATE ota2_test_get_update() failed! %d \r\n", result);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_GET_UPDATE ota2_test_get_update() done.\r\n");
            }
        }

        if (events & PLAYER_EVENT_GET_TIMED_UPDATE)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2_EXAMPLE_UPDATE_TYPE_TIMED ! \r\n");
            result = ota2_test_start_timed_update(player);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_GET_TIMED_UPDATE Download setup failed! %d \r\n", result);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_GET_TIMED_UPDATE function call ok, wait for timeout to start download.\r\n");
            }
        }

        if (events & PLAYER_EVENT_STOP_TIMED_UPDATE)
        {
            result = ota2_test_stop_timed_update(player);
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_STOP_TIMED_UPDATE called ota2_test_stop_timed_update()! %d \r\n", result);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_STOP_TIMED_UPDATE Download failed! %d \r\n", result);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_STOP_TIMED_UPDATE stopped download ! :) \r\n");
            }
        }

        if (events & PLAYER_EVENT_FACTORY_RESET_STATUS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_FACTORY_RESET_STATUS Status validate\n");
            wiced_ota2_image_validate ( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP );
        }

        if (events & PLAYER_EVENT_FACTORY_RESET_REBOOT)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_FACTORY_RESET_REBOOT Reset on Reboot\n");
            wiced_ota2_force_factory_reset_on_reboot();
        }

        if (events & PLAYER_EVENT_FACTORY_RESET_NOW)
        {
            /* extract the image in the Factory Reset area */
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Extract the OTA2 Image in the Factory Reset Area NOW (will FAIL on Internal Flash systems)\r\n");

            /* save current DCT settings*/
            wiced_dct_ota2_save_copy(OTA2_BOOT_FACTORY_RESET);

            wiced_ota2_image_extract ( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP );
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Reboot now to see change!\r\n");
        }

        if (events & PLAYER_EVENT_UPDATE_NOW)
        {
            /* extract the image in the staging area */
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Extract the OTA2 Image in the Staged Area NOW (will FAIL on Internal Flash systems)\r\n");

            /* save current DCT settings */
            wiced_dct_ota2_save_copy(OTA2_BOOT_UPDATE);

            /* Fake the Staged Area download status so we can extract it */
            wiced_ota2_image_fakery(WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE);
            wiced_ota2_image_extract ( WICED_OTA2_IMAGE_TYPE_STAGED );
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Reboot now to see change!\r\n");
        }

        if (events & PLAYER_EVENT_UPDATE_REBOOT)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Mark the update to be extracted on next Boot.\r\n");
            wiced_ota2_image_fakery(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT);
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "ReBoot now to watch the extraction.\r\n");
        }

        if (events & PLAYER_EVENT_UPDATE_STATUS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_UPDATE Status validate\n");
            wiced_ota2_image_validate ( WICED_OTA2_IMAGE_TYPE_STAGED );
        }

        if (events & PLAYER_EVENT_RELOAD_DCT_WIFI)
        {
            ota2_test_config_reload_dct_wifi(player);
        }
        if (events & PLAYER_EVENT_RELOAD_DCT_NETWORK)
        {
            ota2_test_config_reload_dct_network(player);
        }

    }   /* while */

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "End ota2_test mainloop\r\n");
}


static void ota2_test_shutdown(ota2_data_t *player)
{

    /*
     * Shutdown the console.
     */

    command_console_deinit();

    ota2_test_stop_timed_update(player);

    wiced_rtos_deinit_event_flags(&player->events);

    ota2_test_config_deinit(player);

    player->tag = PLAYER_TAG_INVALID;
    free(player);
}

static void ota2_test_console_dct_callback(console_dct_struct_type_t struct_changed, void* app_data)
{
    ota2_data_t*      player;

    /* sanity check */
    if (app_data == NULL)
    {
        return;
    }

    player = (ota2_data_t*)app_data;
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

int ota2_test_log_output_handler(WICED_LOG_LEVEL_T level, char *logmsg)
{
    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}


static wiced_result_t ota2_test_log_get_time(wiced_time_t* time)
{
    wiced_time_t now;
    wiced_result_t result;

    /*
     * Get the current time.
     */

    result = wiced_time_get_time(&now);
    *time  = now - ota2_test_start_time;

    return result;
}

static ota2_data_t* init_player(void)
{
    ota2_data_t*            player = NULL;
    wiced_result_t          result;
    uint32_t                tag;
    ota2_boot_type_t        boot_type;

    tag = PLAYER_TAG_VALID;

    /*
     * Initialize the logging subsystem.
     */

    result = wiced_log_init(WICED_LOG_ERR, ota2_test_log_output_handler, ota2_test_log_get_time);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("wiced_log_init() failed !\r\n"));
    }

    /* Initialize the device */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return NULL;
    }

   /*
     * Allocate the main data structure.
     */
    player = calloc_named("ota2_test", 1, sizeof(ota2_data_t));
    if (player == NULL)
    {
        wiced_log_printf( "Unable to allocate player structure\r\n");
        return NULL;
    }

    /*
     * Create the command console.
     */

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Start the command console\r\n");
    result = command_console_init(STDIO_UART, sizeof(ota2_command_buffer), ota2_command_buffer, CONSOLE_COMMAND_HISTORY_LENGTH, ota2_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Error starting the command console\r\n");
        free(player);
        return NULL;
    }
    console_add_cmd_table(ota2_command_table);
    console_dct_register_callback(ota2_test_console_dct_callback, player);

    /*
     * Create our event flags.
     */
    result = wiced_rtos_init_event_flags(&player->events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Error initializing event flags\r\n");
        tag = PLAYER_TAG_INVALID;
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "\nOTA2_IMAGE_FLASH_BASE                0x%08lx\n", (uint32_t)OTA2_IMAGE_FLASH_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_FACTORY_RESET_AREA_BASE   0x%08lx\n", (uint32_t)OTA2_IMAGE_FACTORY_RESET_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_FAILSAFE_APP_AREA_BASE    0x%08lx\n", (uint32_t)OTA2_IMAGE_FAILSAFE_APP_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_APP_DCT_SAVE_AREA_BASE    0x%08lx\n", (uint32_t)OTA2_IMAGE_APP_DCT_SAVE_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_LUT_AREA_BASE        0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_LUT_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_DCT_1_AREA_BASE      0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_DCT_1_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_DCT_2_AREA_BASE      0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_DCT_2_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_OTA_APP_AREA_BASE    0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_OTA_APP_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_FS_AREA_BASE         0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_FS_AREA_BASE);
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_CURR_APP0_AREA_BASE       0x%08lx\n", (uint32_t)OTA2_IMAGE_CURR_APP0_AREA_BASE);
#if defined(OTA2_IMAGE_LAST_KNOWN_GOOD_AREA_BASE)
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_LAST_KNOWN_GOOD_AREA_BASE 0x%08lx\n", (uint32_t)OTA2_IMAGE_LAST_KNOWN_GOOD_AREA_BASE);
#endif
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2_IMAGE_STAGING_AREA_BASE         0x%08lx\r\n", (uint32_t)OTA2_IMAGE_STAGING_AREA_BASE);

    /* read in our configurations */
    ota2_test_config_init(player);

    /* determine if this is a first boot, factory reset, or after an update boot */
    boot_type = wiced_ota2_get_boot_type();
    switch( boot_type )
    {
        case OTA2_BOOT_FAILSAFE_FACTORY_RESET:
        case OTA2_BOOT_FAILSAFE_UPDATE:
        default:
            /* We should never get here! */
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Unexpected boot_type %d!\r\n", boot_type);
            /* FALL THROUGH */
        case OTA2_BOOT_NEVER_RUN_BEFORE:
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "First BOOT EVER\r\n");
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_NORMAL:
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Normal reboot - count:%ld.\r\n", player->dct_app->reboot_count);
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Factory Reset Occurred!\r\n");
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_SOFTAP_UPDATE:    /* pre-OTA2 failsafe ota2_bootloader designation for a SOFTAP update */
        case OTA2_BOOT_UPDATE:
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Update Occurred!\r\n");
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_LAST_KNOWN_GOOD:
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Last Known Good used!\r\n");
            break;
    }

    /* Get RTC Clock time and set it here */
    {
        wiced_time_t time = 0;
        wiced_time_set_time( &time );
    }


    /* keep track of # of reboots */
    player->dct_app->reboot_count++;
    wiced_dct_write( (void*)player->dct_app, DCT_APP_SECTION, 0, sizeof( ota2_dct_t ) );

    /* print out our current configuration */
    ota2_config_print_info(player);

    /* connect to the network */
    player->ota2_bg_params.ota2_interface           = player->dct_network->interface;
    result = wiced_ota2_network_up(player->ota2_bg_params.ota2_interface, player->dct_wifi->stored_ap_list);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "init: Bringing up network interface failed!\r\n");
    }

    /* set our valid tag */
    player->tag = tag;

    return player;
}


wiced_result_t over_the_air_2_app_restore_settings_after_update(ota2_data_t* player, ota2_boot_type_t boot_type)
{
    uint16_t major = 0, minor = 0;
    platform_dct_network_config_t   dct_network = { 0 };
    platform_dct_wifi_config_t      dct_wifi = { 0 };
    ota2_dct_t                      dct_app = { 0 };

    /* read in our configurations from the DCT copy */
    /* network */
    if (wiced_dct_ota2_read_saved_copy( &dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "over_the_air_2_app_restore_settings_after_update() failed reading Network Config!\r\n");
        return WICED_ERROR;
    }

    /* wifi */
    if (wiced_dct_ota2_read_saved_copy( &dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "over_the_air_2_app_restore_settings_after_update() failed reading WiFi Config!\r\n");
        return WICED_ERROR;
    }

    /* App */
    if (wiced_dct_ota2_read_saved_copy( &dct_app, DCT_APP_SECTION, 0, sizeof(ota2_dct_t)) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "over_the_air_2_app_restore_settings_after_update() failed reading App Config!\r\n");
        return WICED_ERROR;
    }

    memcpy(player->dct_network, &dct_network, sizeof(platform_dct_network_config_t));
    memcpy(player->dct_wifi, &dct_wifi, sizeof(platform_dct_wifi_config_t));
    memcpy(player->dct_app, &dct_app, sizeof(ota2_dct_t));

    /* update version number based on boot type */
    switch (boot_type)
    {
        default:
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, &major, &minor) == WICED_SUCCESS)
            {
                player->dct_app->ota2_major_version = major;
                player->dct_app->ota2_minor_version = minor;
            }
            break;
        case OTA2_BOOT_SOFTAP_UPDATE:
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_UPDATE:
            if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_STAGED, &major, &minor) == WICED_SUCCESS)
            {
                player->dct_app->ota2_major_version = major;
                player->dct_app->ota2_minor_version = minor;
            }
            break;
    }

    /* now, save them all! */
    if (ota2_save_config(player) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "over_the_air_2_app_restore_settings_after_update() failed Saving Config!\r\n");
        return WICED_ERROR;
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Restored saved Configuration!\r\n");
    return WICED_SUCCESS;
}



