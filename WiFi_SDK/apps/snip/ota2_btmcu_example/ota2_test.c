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
 * OTA Image test Application.
 *
 * This application snippet demonstrates how to use the WICED
 * interface for performing Over The Air Updates to your device
 *
 * Application Instructions
 * 1. wifi_config_dct.h: Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the header file to match your Wi-Fi access point
 * 2. ota2_test_dct.c: Modify the ota2_update_uri filed with appropriate server address and file name
 * 3. Connect a PC terminal to the serial port of the WICED Eval board,
 *    then build and download the application as described in the WICED
 *    Quick Start Guide
 *
 * After the download completes, it connects to the Wi-Fi AP specified in apps/snip/ota2_test/wifi_config_dct.h
 *
 * When you issue a "get_ap_update <host>/ota2_image_file" the app will connect directly ONCE to download the OTA2 Image file
 *                  <ota2_image_file> from <host>.
 *
 * Same procedure for upgrading filesystem image. use command get_fs_update.
 *
 */

#include <unistd.h>
#include <ctype.h>

#include "wiced.h"
#include "wiced_log.h"
#include "wiced_ota2_image.h"
#include "wiced_ota2_service.h"
#include "ota2_test.h"
#include "ota2_test_dct.h"
#include "wiced_rtos.h"
#include "command_console.h"

/******************************************************
 *                      Macros
 ******************************************************/
 #ifdef OTA2_CMD_CONSOLE_ENABLED
#define OTA2_CONSOLE_COMMANDS \
    { (char*) "log",            ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Set log level" }, \
    { (char*) "get_app_update",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Get OTA2 APP update - use connection" }, \
    { (char*) "get_fs_update",     ota2_console_command,    0, NULL, NULL, (char *)"", (char *)"Get OTA2 FS update - use connection" }, \

#endif

/******************************************************
 *                    Constants
 ******************************************************/
//#define SECONDS_PER_MINUTE          60
#ifdef OTA2_CMD_CONSOLE_ENABLED
#define MAX_COMMAND_LENGTH                   (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH      (4)
#endif

#define RESTORE_DCT_APP_SETTINGS    (0)

/******************************************************
 *                   Enumerations
 ******************************************************/
#ifdef OTA2_CMD_CONSOLE_ENABLED
typedef enum {

    OTA2_CONSOLE_CMD_GET_APP_UPDATE,
    OTA2_CONSOLE_CMD_GET_FS_UPDATE,
    OTA2_CONSOLE_CMD_LOG_LEVEL,
    OTA2_CONSOLE_CMD_MAX,
} OTA2_CONSOLE_CMDS_T;

#else

typedef enum {

    OTA2_STAGE_IMAGE_INDEX =0,
    OTA2_FS_IMAGE_INDEX,

    OTA2_MAX_IMAGE_INDEX

} OTA2_IMAGE_INDEX_T;

#endif

/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/
#ifdef OTA2_CMD_CONSOLE_ENABLED

 typedef struct cmd_lookup_s {
        char *cmd;
        uint32_t event;
} cmd_lookup_t;
#endif
/******************************************************
 *               Function Declarations
 ******************************************************/
#ifdef OTA2_CMD_CONSOLE_ENABLED
int ota2_console_command(int argc, char *argv[]);
#endif
static void ota2_test_mainloop ( void );
#if RESTORE_DCT_APP_SETTINGS
static wiced_result_t over_the_air_2_app_restore_settings_after_update(ota2_data_t* player, ota2_boot_type_t boot_type);
static wiced_result_t ota2_save_config(ota2_data_t* player);
static wiced_result_t ota2_save_network_dct(ota2_data_t* player);
static wiced_result_t ota2_save_wifi_dct(ota2_data_t* player);
static wiced_result_t ota2_save_app_dct(ota2_data_t* player);
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef OTA2_CMD_CONSOLE_ENABLED

static char ota2_command_buffer[MAX_COMMAND_LENGTH];
static char ota2_command_history_buffer[MAX_COMMAND_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];


const command_t ota2_command_table[] = {
    OTA2_CONSOLE_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[OTA2_CONSOLE_CMD_MAX] = {
        { "get_app_update",     PLAYER_EVENT_GET_UPDATE  },
        { "get_fs_update",        PLAYER_EVENT_GET_UPDATE  },
        { "log",                       0                                                },
};

#else

uint32_t        g_curr_uri_index=OTA2_STAGE_IMAGE_INDEX;

#endif

/* source info for http streaming */
//char                            uri_to_stream[WICED_OTA2_HTTP_QUERY_SIZE];

/* background service for OTA2 info */
//char                            ota2_host_name[WICED_OTA2_HOST_NAME];
//char                            ota2_file_path[WICED_OTA2_FILE_PATH];

ota2_data_t   g_player;

/******************************************************
 *               Function Declarations
 ******************************************************/
/******************************************************
 *               Function Definitions
 ******************************************************/
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
    *time  = now;

    return result;
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

#ifdef OTA2_CMD_CONSOLE_ENABLED

int ota2_console_command(int argc, char *argv[])
{
    uint32_t event = 0;
    int i;

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Received command: %s\n", argv[0]);

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

        case OTA2_CONSOLE_CMD_GET_APP_UPDATE:
            memset(g_player.uri_to_stream, 0, sizeof(g_player.uri_to_stream));
            if (argc > 1)
            {
                strlcpy(g_player.uri_to_stream, argv[1], (sizeof(g_player.uri_to_stream) - 1) );
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Use .ota2_uris[0].ota2_update_uri\r\n");
                strlcpy(g_player.uri_to_stream, g_player.dct_app.ota2_uris[0].ota2_update_uri, (sizeof(g_player.uri_to_stream) - 1) );
            }
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Get update file:%s\r\n", g_player.uri_to_stream);
            event = command_lookup[i].event;

            /* set the image type to ota service */
            wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_STAGED);
            break;

        case OTA2_CONSOLE_CMD_GET_FS_UPDATE:
            memset(g_player.uri_to_stream, 0, sizeof(g_player.uri_to_stream));
            if (argc > 1)
            {
                strlcpy(g_player.uri_to_stream, argv[1], (sizeof(g_player.uri_to_stream) - 1) );
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Use .ota2_uris[1].ota2_update_uri\r\n");
                strlcpy(g_player.uri_to_stream, g_player.dct_app.ota2_uris[1].ota2_update_uri, (sizeof(g_player.uri_to_stream) - 1) );
            }
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Get update file:%s\r\n", g_player.uri_to_stream);
            event = command_lookup[i].event;

            /* set the image type to ota service */
            wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_FS);
            break;

        case OTA2_CONSOLE_CMD_LOG_LEVEL:
            if (argc > 1)
            {
                int new_level;
                new_level = atoi(argv[1]);
                if ((new_level > 0) && (new_level < WICED_LOG_DEBUG4))
                {
                    g_player.log_level = new_level;
                    wiced_log_set_all_levels(new_level);
                }
            }
            wiced_log_printf( "log level = %d\r\n", g_player.log_level);
            break;

        default:
            wiced_log_printf ("Invalid command !!!\r\n");
    }

    if (event)
    {
        /*
         * Send off the event to the main loop.
         */

        wiced_rtos_set_event_flags(&g_player.events, event);
    }

    return ERR_CMD_OK;
}

#endif
wiced_result_t ota2_test_config_init(ota2_data_t *player)
{
    wiced_result_t result;

    result = wiced_dct_read_with_copy(player->stored_ap_list, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t,stored_ap_list), sizeof(player->stored_ap_list));
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Can't get WIFi configuration!\r\n"));
        return WICED_ERROR;
    }

    result = wiced_dct_read_with_copy( &(player->dct_network), DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Can't get Network configuration!\r\n"));
        return WICED_ERROR;
    }

    /* App */
    result |= wiced_dct_read_with_copy( &(player->dct_app), DCT_APP_SECTION, 0, sizeof( ota2_dct_t ) );

    return result;
}


wiced_result_t test_init ( void )
{
    ota2_boot_type_t        boot_type;
    wiced_result_t          result;

    result = wiced_log_init(WICED_LOG_ERR, ota2_test_log_output_handler, ota2_test_log_get_time);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("wiced_log_init() failed !\r\n"));
        return WICED_ERROR;
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Start the command console\r\n");
    result = command_console_init(STDIO_UART, sizeof(ota2_command_buffer), ota2_command_buffer, CONSOLE_COMMAND_HISTORY_LENGTH, ota2_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Error starting the command console\r\n");
        return WICED_ERROR;
    }
    console_add_cmd_table(ota2_command_table);

    /*
     * Create our event flags.
     */
    result = wiced_rtos_init_event_flags(&g_player.events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Error initializing event flags\r\n");
        return WICED_ERROR;
    }

    /* read in our configurations */
    result = ota2_test_config_init(&g_player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Error initializing app configs\r\n");
        return WICED_ERROR;
    }

    /* determine if this is a first boot, factory reset, or after an update boot */
    boot_type = wiced_ota2_get_boot_type();

    switch( boot_type )
    {
        case OTA2_BOOT_FAILSAFE_FACTORY_RESET:
        case OTA2_BOOT_FAILSAFE_UPDATE:
        default:
            /* We should never get here! */
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR,  "Unexpected boot_type %d!\r\n", (int)boot_type);
            /* FALL THROUGH */
        case OTA2_BOOT_NEVER_RUN_BEFORE:
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR,  "First BOOT EVER\r\n");
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_NORMAL:
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "Normal reboot - count:%d\r\n", (int)g_player.dct_app.reboot_count);
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "Factory Reset Occurred!\r\n");
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(&g_player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_SOFTAP_UPDATE:    /* pre-OTA2 failsafe ota2_bootloader designation for a SOFTAP update */
        case OTA2_BOOT_UPDATE:
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "Update Occurred!\r\n");
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(&g_player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_LAST_KNOWN_GOOD:
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "Last Known Good used!\r\n");
            break;
    }

    /* keep track of # of reboots */
    g_player.dct_app.reboot_count++;
    wiced_dct_write( (void*)&(g_player.dct_app), DCT_APP_SECTION, 0, sizeof( ota2_dct_t ) );

    /* connect to the network */
    g_player.ota2_bg_params.ota2_interface           = g_player.dct_network.interface;

    result = wiced_ota2_network_up(g_player.ota2_bg_params.ota2_interface, g_player.stored_ap_list);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR,  "init: Bringing up network interface failed!\r\n");
    }

return result;
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
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "----------------------------- OTA2 Service Called : SERVICE STARTED -----------------------------\r\n");
        break;
    case OTA2_SERVICE_AP_CONNECT_ERROR:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : AP CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_SERVER_CONNECT_ERROR:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : SERVER_CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_AP_CONNECTED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : AP_CONNECTED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_SERVER_CONNECTED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : SERVER_CONNECTED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        break;


    case OTA2_SERVICE_CHECK_FOR_UPDATE: /* Time to check for updates.
                                         * return - WICED_SUCCESS = Service will check for update availability
                                         *        - WICED_ERROR   = Application will check for update availability   */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : CHECK_FOR_UPDATE -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCCESS, let Service do the checking.\r\n");
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

        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : UPDATE_AVAILABLE -----------------------------\r\n");
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */

        /*
         * In an actual application, the application would look at the headers information and decide if the
         * file on the update server is a newer version that the currently running application.
         *
         * If the application wants the update to continue, it would return WICED_SUCCESS here
         * If not, return WICED_ERROR
         *
         */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "   OTA2 Version %d.%d\r\n", ota2_header->major_version, ota2_header->minor_version);
/*
        if ((player->dct_app.ota2_major_version > ota2_header->major_version) ||
            ((player->dct_app.ota2_major_version == ota2_header->major_version) &&
             (player->dct_app.ota2_minor_version >= ota2_header->minor_version)) )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "OTA2 Update Version Fail - return ERROR, do not update!\r\n");
            return WICED_ERROR;
        }
*/
        /* check components inside image */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCCESS, let Service perform the download.\r\n");
        return WICED_SUCCESS;
    }

    case OTA2_SERVICE_DOWNLOAD_STATUS:  /* Download status - value has % complete (0-100)
                                         *   NOTE: This will only occur when Service is performing download
                                         * return - WICED_SUCCESS = Service will continue download
                                         *        - WICED_ERROR   = Service will STOP download and service will
                                         *                          issue OTA2_SERVICE_TIME_TO_UPDATE_ERROR           */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "my_ota2_callback() OTA2_SERVICE_DOWNLOAD_STATUS %d %%!\r\n", value);
        return WICED_SUCCESS;

    case OTA2_SERVICE_PERFORM_UPDATE:   /* Download is complete
                                         * return - WICED_SUCCESS = Service will inform Bootloader to extract
                                         *                          and update on next power cycle
                                         *        - WICED_ERROR   = Service will inform Bootloader that download
                                         *                          is complete - Bootloader will NOT extract        */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : PERFORM_UPDATE -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCCESS, let Service extract update on next reboot.\r\n");
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ERROR:     /* There was an error in transmission
                                         * This will only occur if Error during Service performing data transfer
                                         * upon return - if retry_interval is set, Service will use retry_interval
                                         *               else, Service will retry on next check_interval
                                         */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : UPDATE_ERROR -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCCESS, Service will retry as parameters defined in init.\r\n");
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ENDED:     /* All update actions for this check are complete.
                                         * This callback is to allow the application to take any actions when
                                         * the service is done checking / downloading an update
                                         * (succesful, unavailable, or error)
                                         * return - None                                                             */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : UPDATE_ENDED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        if (value == WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Download is completed for File %s \n",player->ota2_file_path);
#ifdef OTA2_CMD_CONSOLE_ENABLED
            wiced_rtos_set_event_flags(&g_player.events, PLAYER_EVENT_STOP_TIMED_UPDATE);
#else
            // check for current image. if fs_image then reboot
            if (g_curr_uri_index == OTA2_MAX_IMAGE_INDEX)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        REBOOTING !!!\r\n");
                wiced_framework_reboot();
            }
            else
            {
                wiced_rtos_set_event_flags(&g_player.events, PLAYER_EVENT_STOP_TIMED_UPDATE);
            }
#endif
        }

       else
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "\r\nRetrying after %d sec !!!\r\n\n",(int)(player->ota2_bg_params.check_interval));
        }
        // stop ota2 service
        break;

    case OTA2_SERVICE_STOPPED:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "----------------------------- OTA2 Service Called : SERVICE ENDED -----------------------------\r\n");
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "        return SUCESS (not used by service). This is informational \r\n");
        break;

    default:
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE,  "my_ota2_callback() UNKNOWN STATUS %d!\r\n", status);
        break;
    }

    return WICED_SUCCESS;
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
    player->ota2_bg_params.default_ap_info = player->stored_ap_list;
    player->ota2_bg_params.ota2_interface = player->dct_network.interface;

    if (player->ota2_bg_params.ota2_interface != WICED_ETHERNET_INTERFACE)
    {
        player->ota2_bg_params.ota2_ap_info             = NULL;
        player->ota2_bg_params.ota2_ap_list             = &player->ota2_bg_params.default_ap_info[0]; /* use the DCT AP list */
        player->ota2_bg_params.ota2_ap_list_count       = 1;
    }

    if (player->ota2_bg_service == NULL)
    {
        player->ota2_bg_service = wiced_ota2_service_init(&player->ota2_bg_params, (void*)player);
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "ota2_test_get_update() wiced_ota2_service_init() bg_service:%x \r\n", (unsigned int)player->ota2_bg_service);
    }
    else
    {
        /* bg service already started - this is OK, just don't deinit at the end of this function */
    }

    if (player->ota2_bg_service != NULL)
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(player->ota2_bg_service, my_ota2_callback);
        if (result != WICED_SUCCESS)
        {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR,  "ota2_test_get_update register callback failed! %d \r\n", result);
                wiced_ota2_service_deinit(player->ota2_bg_service);
                player->ota2_bg_service = NULL;
        }

        if (player->ota2_bg_service != NULL)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Download the OTA Image file - get it NOW!\r\n");
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

void ota2_test_mainloop ( void )
{
    wiced_result_t result;
    uint32_t            events;

    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Ready to get update \r\n");

    while ( 1 )
    {
        // waiting on events
        result = wiced_rtos_wait_for_event_flags(&g_player.events, PLAYER_ALL_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "%s %d --> %d\n",__func__,__LINE__,result);
            continue;
        }

        if (events & PLAYER_EVENT_STOP_TIMED_UPDATE)
        {
            result = ota2_test_stop_timed_update(&g_player);
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_STOP_TIMED_UPDATE called ota2_test_stop_timed_update()! %d \r\n", result);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "PLAYER_EVENT_STOP_TIMED_UPDATE Download failed! %d \r\n", result);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "PLAYER_EVENT_STOP_TIMED_UPDATE stopped download ! :) \r\n");
            }
#ifndef OTA2_CMD_CONSOLE_ENABLED
            //get next update
            wiced_rtos_set_event_flags(&g_player.events, PLAYER_EVENT_GET_UPDATE);
#endif
        }

        if (events & PLAYER_EVENT_GET_UPDATE)
        {
#ifdef OTA2_CMD_CONSOLE_ENABLED
            ota2_test_get_update(&g_player);
#else
            int i = 0;

            for (i=0; i<OTA2_MAX_URI; i++)
            {
                // check for current file index
                if (g_curr_uri_index == i)
                {
                    break;
                }
            }

            if (i == OTA2_MAX_URI)
            {
                g_curr_uri_index = OTA2_STAGE_IMAGE_INDEX;
            }

            // update the file path
            strlcpy(g_player.uri_to_stream, g_player.dct_app.ota2_uris[g_curr_uri_index].ota2_update_uri, (sizeof(g_player.uri_to_stream) - 1) );
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, " Get update URI:%s\r\n", g_player.dct_app.ota2_uris[g_curr_uri_index].ota2_update_uri);

            // check for fs case. if true then set image writer module to write in fs area instead of staging
            if (g_curr_uri_index == OTA2_FS_IMAGE_INDEX)
            {
                wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_FS);
            }
            else
            {
                wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_STAGED);
            }

            g_curr_uri_index++;

            ota2_test_get_update(&g_player);
#endif
        }
    }
}

void application_start ( void )
{
    wiced_result_t result;

    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Error !!! wiced_init \n\r");
        return;
    }

    result = test_init();
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Error !!! test_init \n\r");
        return;
    }

#ifndef OTA2_CMD_CONSOLE_ENABLED
    // Start the update
    g_curr_uri_index = OTA2_STAGE_IMAGE_INDEX;
    wiced_rtos_set_event_flags(&g_player.events, PLAYER_EVENT_GET_UPDATE);
#endif
    ota2_test_mainloop();

}

#if RESTORE_DCT_APP_SETTINGS

wiced_result_t over_the_air_2_app_restore_settings_after_update(ota2_data_t* player, ota2_boot_type_t boot_type)
{
    uint16_t major = 0, minor = 0;
    platform_dct_network_config_t   dct_network = { 0 };
    wiced_config_ap_entry_t         stored_ap_list[CONFIG_AP_LIST_SIZE];
    ota2_dct_t                      dct_app = { 0 };

    /* read in our configurations from the DCT copy */
    /* network */
    if (wiced_dct_ota2_read_saved_copy( &dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING,   "over_the_air_2_app_restore_settings_after_update() failed reading Network Config!\r\n");
        return WICED_ERROR;
    }

    /* wifi */
    if (wiced_dct_ota2_read_saved_copy( stored_ap_list, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t,stored_ap_list), sizeof(player->stored_ap_list)) != WICED_SUCCESS)
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

    memcpy(&(player->dct_network), &dct_network, sizeof(platform_dct_network_config_t));
    memcpy(player->stored_ap_list, stored_ap_list, sizeof(player->stored_ap_list));
    memcpy(&(player->dct_app), &dct_app, sizeof(ota2_dct_t));

    /* update version number based on boot type */
    switch (boot_type)
    {
        default:
            break;
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_UPDATE:
            if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_STAGED, &major, &minor) == WICED_SUCCESS)
            {
                player->dct_app.ota2_major_version = major;
                player->dct_app.ota2_minor_version = minor;
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

static wiced_result_t ota2_save_app_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)&(player->dct_app), DCT_APP_SECTION, 0, sizeof(ota2_dct_t) );
}

static wiced_result_t ota2_save_network_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)&(player->dct_network), DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
}

static wiced_result_t ota2_save_wifi_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)player->stored_ap_list, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t,stored_ap_list), sizeof(player->stored_ap_list) );
}

static wiced_result_t ota2_save_config(ota2_data_t* player)
{
    wiced_result_t result;
    /* save all the configuration info */
    result = ota2_save_app_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_save_config() ota2_save_app_dct() failed:%d\r\n", result);
        return result;
    }

    result = ota2_save_network_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_save_config() ota2_save_network_dct() failed:%d\r\n", result);
        return result;
    }
    result = ota2_save_wifi_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_save_config() ota2_save_wifi_dct() failed:%d\r\n", result);
        return result;
    }

    return result;
}
#endif
