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
 * SSDP Application
 *
 * Features demonstrated
 *  - WICED SSDP API
 *
 * This application snippet regularly advertises and checks for other devices using SSDP
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 * You will want to change the AP information in wifi_config_dct.h to
 * join your AP at startup.
 *
 *  use "help" to get general console commands
 *  use "config help" to get configuration change console commands
 *
 *  ssdp_msearch    - send M-Search, wait for results, print them out
 *  ssdp_info       - output some debugging info
 *  ssdp_log_off    - set library logging to off
 *  ssdp_log_low    - set library logging to low
 *  ssdp_log_info   - set library logging to info
 *  ssdp_log_debug  - set library logging to debug - you can see all the packets
 *
 */

#include <stdlib.h>
#include "wiced.h"
#include "wiced_tcpip.h"
#include "command_console.h"
#include "wifi/command_console_wifi.h"

#ifdef WWD_TEST_NVRAM_OVERRIDE
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#endif

#include "ssdp_app.h"
#include "ssdp_app_config.h"


/******************************************************
 *                      Macros
 ******************************************************/
int app_console_command(int argc, char *argv[]);

#define APP_CONSOLE_COMMANDS \
    { (char*) "exit",         app_console_command, 0, NULL, NULL, (char *)"", (char *)"Exit application" }, \
    { (char*) "ssdp_init",    app_console_command, 0, NULL, NULL, (char *)"", (char *)"Initialize SSDP" }, \
    { (char*) "ssdp_deinit",  app_console_command, 0, NULL, NULL, (char *)"", (char *)"De-Initialize SSDP" }, \
    { (char*) "ssdp_msearch", app_console_command, 0, NULL, NULL, (char *)"", (char *)"Send SSDP M-Search (scan)" }, \
    { (char*) "ssdp_info",    app_console_command, 0, NULL, NULL, (char *)"", (char *)"SSDP information" }, \
    { (char*) "ssdp_log_off",   app_console_command, 0, NULL, NULL, (char *)"", (char *)"Set SSDP LIB log level off"   }, \
    { (char*) "ssdp_log_low",   app_console_command, 0, NULL, NULL, (char *)"", (char *)"Set SSDP LIB log level low"   }, \
    { (char*) "ssdp_log_info",  app_console_command, 0, NULL, NULL, (char *)"", (char *)"Set SSDP LIB log level info"  }, \
    { (char*) "ssdp_log_debug", app_console_command, 0, NULL, NULL, (char *)"", (char *)"Set SSDP LIB log level debug" }, \
    { (char*) "config",       app_console_command, 0, NULL, NULL, (char *)"", (char *)"Display / change config values" }, \

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_APP_COMMAND_LENGTH              (85)
#define APP_CONSOLE_COMMAND_HISTORY_LENGTH  (10)

#define APP_MAIN_THREAD_PRIORITY             (4)

#define SSDP_THREAD_STACK_SIZE               (2 * 4096)

#define SSDP_PACKET_TIMEOUT_MS               (500)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    APP_CONSOLE_CMD_EXIT = 0,
    APP_CONSOLE_CMD_SSDP_INITIALISE,
    APP_CONSOLE_CMD_SSDP_DEINIT,
    APP_CONSOLE_CMD_SSDP_MSEARCH,
    APP_CONSOLE_CMD_SSDP_INFORMATION,

    APP_CONSOLE_CMD_SSDP_LOG_LEVEL_OFF,
    APP_CONSOLE_CMD_SSDP_LOG_LEVEL_LOW,
    APP_CONSOLE_CMD_SSDP_LOG_LEVEL_INFO,
    APP_CONSOLE_CMD_SSDP_LOG_LEVEL_DEBUG,

    APP_CONSOLE_CMD_CONFIG,

    APP_CONSOLE_CMD_MAX,
} APP_CONSOLE_CMDS_T;

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
 *               Static Function Declarations
 ******************************************************/

int ssdp_console_command(int argc, char *argv[]);

/******************************************************
 *               Variable Definitions
 ******************************************************/

#ifdef USE_DEVICE_INIT_IP_SETTINGS
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};
#endif /* #if USE_DEVICE_INIT_IP_SETTINGS */

static char app_command_buffer[MAX_APP_COMMAND_LENGTH];
static char app_command_history_buffer[MAX_APP_COMMAND_LENGTH * APP_CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t app_command_table[] = {
    APP_CONSOLE_COMMANDS
    WIFI_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[APP_CONSOLE_CMD_MAX] = {
        { "exit",           APP_EVENT_SHUTDOWN          },
        { "ssdp_init",      APP_EVENT_SSDP_INIT         },
        { "ssdp_deinit",    APP_EVENT_SSDP_DEINIT       },
        { "ssdp_msearch",   APP_EVENT_SSDP_MSEARCH      },
        { "ssdp_info",      APP_EVENT_SSDP_INFORMATION  },
        { "ssdp_log_off",   APP_EVENT_SSDP_LOG_LEVEL_OFF   },
        { "ssdp_log_low",   APP_EVENT_SSDP_LOG_LEVEL_LOW   },
        { "ssdp_log_info",  APP_EVENT_SSDP_LOG_LEVEL_INFO  },
        { "ssdp_log_debug", APP_EVENT_SSDP_LOG_LEVEL_DEBUG },

        { "config",         0                           },
};

static application_info_t *global_app_info;

/******************************************************
 *               Function Definitions
 ******************************************************/

int app_console_command(int argc, char *argv[])
{
    uint32_t event = 0;
    int i;

    WPRINT_APP_INFO(("Received command: %s\n", argv[0]));

    if (global_app_info == NULL || global_app_info->tag != APP_TAG_VALID)
    {
        WPRINT_APP_INFO(("Bad g_app_info structure\r\n"));
        return ERR_CMD_OK;
    }

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < APP_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= APP_CONSOLE_CMD_MAX)
    {
        WPRINT_APP_INFO(("Unrecognized command: %s\n", argv[0]));
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case APP_CONSOLE_CMD_EXIT:
        case APP_CONSOLE_CMD_SSDP_INITIALISE:
        case APP_CONSOLE_CMD_SSDP_DEINIT:
        case APP_CONSOLE_CMD_SSDP_MSEARCH:
        case APP_CONSOLE_CMD_SSDP_INFORMATION:
        case APP_CONSOLE_CMD_SSDP_LOG_LEVEL_OFF:
        case APP_CONSOLE_CMD_SSDP_LOG_LEVEL_LOW:
        case APP_CONSOLE_CMD_SSDP_LOG_LEVEL_INFO:
        case APP_CONSOLE_CMD_SSDP_LOG_LEVEL_DEBUG:
            event = command_lookup[i].event;
            break;

        case APP_CONSOLE_CMD_CONFIG:
            app_set_config(global_app_info, argc, argv);
            break;


        default:
            WPRINT_APP_INFO(("Unrecognized command: %s\n", argv[0]));
            break;
    }

    if (event)
    {
        /*
         * Send off the event to the main app loop.
         */

        wiced_rtos_set_event_flags(&global_app_info->events, event);
    }

    return ERR_CMD_OK;
}

static wiced_result_t ssdp_app_send_msearch(application_info_t *app_info)
{
    wiced_ssdp_msearch_params_t msearch_params = { 0 };

    msearch_params.msearch_scan_time = 3;
    msearch_params.msearch_search_target = "ssdp:all";
    msearch_params.msearch_user_agent = "WICED";
    msearch_params.response_array_size = MSEARCH_RESPONSES_MAX;
    msearch_params.responses = app_info->msearch_responses;
    msearch_params.num_responses = 0;

    WPRINT_APP_INFO(("Send M-Search...\r\n"));
    if( wiced_ssdp_send_msearch_wait_for_results(app_info->ssdp_info, &msearch_params) == WICED_SUCCESS)
    {
        int i;
        WPRINT_APP_INFO(("M-Search Results num:%d!\r\n", msearch_params.num_responses));
        for(i = 0; i < msearch_params.num_responses; i++)
        {
            WPRINT_APP_INFO(("%3d: ip  :%s\r\n", i, app_info->msearch_responses[i].ip_string));
            WPRINT_APP_INFO(("     cache-control :%s\r\n", app_info->msearch_responses[i].cache_control));
            WPRINT_APP_INFO(("     location      :%s\r\n", app_info->msearch_responses[i].location));
            WPRINT_APP_INFO(("     service target:%s\r\n", app_info->msearch_responses[i].st));
            WPRINT_APP_INFO(("     usn           :%s\r\n", app_info->msearch_responses[i].usn));
        }

    }
    else
    {
        WPRINT_APP_INFO(("Failed to send M-Search!\r\n"));
    }

    return WICED_SUCCESS;
}

static void ssdp_notify_callback(wiced_ssdp_notify_info_t* notify_info, void *data)
{
    application_info_t* app_info;

    /* sanity checks */
    if ((notify_info == NULL) || (data == NULL))
    {
        WPRINT_APP_INFO(("ssdp_notify_callback BAD ARGS! info:%p data:%p \r\n", notify_info, data));
        return;
    }

    app_info = (application_info_t *)data;
    if (app_info->tag != APP_TAG_VALID)
    {
        WPRINT_APP_INFO(("ssdp_notify_callback BAD app_info! info->tag:0x%0lx \r\n", app_info->tag));
        return;
    }

    WPRINT_APP_INFO(("NOTIFY INFO %p type:%s\r\n", notify_info, notify_info->nts));
    WPRINT_APP_INFO(("    IP             :%s\r\n", notify_info->ip_string));
    if (strstr(notify_info->nts, "byebye") == NULL)
    {
        /* alive packet! */
        WPRINT_APP_INFO(("    cache_control  :%s\r\n", notify_info->cache_control));
        WPRINT_APP_INFO(("    location       :%s\r\n", notify_info->location));
    }
    WPRINT_APP_INFO(("    server         :%s\r\n", notify_info->server));
    WPRINT_APP_INFO(("    nt             :%s\r\n", notify_info->nt));
    WPRINT_APP_INFO(("    nts            :%s\r\n", notify_info->nts));
    WPRINT_APP_INFO(("    usn            :%s\r\n", notify_info->usn));

    /* todo: keep track of active peers */

}

static void shutdown_application(application_info_t *app_info)
{

    /*
     * make sure we stopped the server
     */
    wiced_ssdp_deinit(app_info->ssdp_info);

    /*
     * Shutdown the console.
     */
    command_console_deinit();

    wiced_rtos_deinit_event_flags(&app_info->events);

    app_info->tag = APP_TAG_INVALID;
    free(app_info);
}

static void set_nvram_mac(void)
{
#ifdef WWD_TEST_NVRAM_OVERRIDE
    platform_dct_wifi_config_t dct_wifi;
    wiced_result_t result;
    uint32_t size;
    uint32_t i;
    char *nvram;

    result = wiced_dct_read_with_copy(&dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS)
    {
        return;
    }

    if (wwd_bus_get_wifi_nvram_image(&nvram, &size) != WWD_SUCCESS)
    {
        return;
    }

    /*
     * We have the mac address from the DCT so now we just need to update the nvram image.
     * Search for the 'macaddr=' token.
     */

    for (i = 0; i < size; )
    {
        if (nvram[i] == '\0')
        {
            break;
        }

        if (nvram[i] != 'm' || nvram[i+1] != 'a' || nvram[i+2] != 'c' || nvram[i+3] != 'a' ||
            nvram[i+4] != 'd' || nvram[i+5] != 'd' || nvram[i+6] != 'r' || nvram[7] != '=')
        {
            while(i < size && nvram[i] != '\0')
            {
                i++;
            }
            i++;
            continue;
        }

        /*
         * Found the macaddr token. Now we just need to update it.
         */

        sprintf(&nvram[i+8], "%02x:%02x:%02x:%02x:%02x:%02x", dct_wifi.mac_address.octet[0],
                dct_wifi.mac_address.octet[1], dct_wifi.mac_address.octet[2], dct_wifi.mac_address.octet[3],
                dct_wifi.mac_address.octet[4], dct_wifi.mac_address.octet[5]);
        break;
    }
#endif
}

static application_info_t *init_applicaiton(void)
{
    application_info_t *app_info;
    wiced_result_t result;
    uint32_t tag = APP_TAG_VALID;

    /*
    * Temporary - set the device MAC address in the NVRAM.
     */
    set_nvram_mac();

    /*
     * Initialize the device and WICED framework
     */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return NULL;
    }

    /*
     * allocate and init SSDP app info
     */
    app_info = (application_info_t *)malloc(sizeof(application_info_t));
    if (app_info == NULL)
    {
        WPRINT_APP_ERROR(("Failed to allocate application info structure!\r\n"));
        return NULL;
    }
    memset(app_info, 0, sizeof(application_info_t));

    /*
     * Create the command console.
     */

    WPRINT_APP_INFO(("Start the command console\r\n"));
    result = command_console_init(STDIO_UART, sizeof(app_command_buffer), app_command_buffer, APP_CONSOLE_COMMAND_HISTORY_LENGTH, app_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error starting the command console\r\n"));
        free(app_info);
        return NULL;
    }
    console_add_cmd_table(app_command_table);

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&app_info->events);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error initializing event flags\r\n"));
        tag = APP_TAG_INVALID;
        goto _init_app_exit;
    }

    /* Get network configuration */
    result = wiced_dct_read_lock((void **)&app_info->dct_network, WICED_FALSE, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t));
    if (result != WICED_SUCCESS || (app_info->dct_network->interface != WICED_STA_INTERFACE && app_info->dct_network->interface != WICED_AP_INTERFACE))
    {
        WPRINT_APP_INFO(("Can't get network configuration!\r\n"));
        tag = APP_TAG_INVALID;
        goto _init_app_exit;
    }

    /* Get WiFi configuration */
    result = wiced_dct_read_lock( (void **)&app_info->dct_wifi, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS || app_info->dct_wifi->device_configured != WICED_TRUE)
    {
        WPRINT_APP_INFO(("Can't get WiFi configuration!\r\n"));
        tag = APP_TAG_INVALID;
        goto _init_app_exit;
    }

    {
        wiced_mac_t wiced_mac = {{0}};
        /* Until the MAC is OTP'ed into the board, let's check to see if the Wifi DCT mac
         *       is non-zero and different from the NVRAM value
         */
        /* check if the DCT mac is non-zero */
        if (memcmp(&wiced_mac, &app_info->dct_wifi->mac_address, sizeof(wiced_mac_t)) != 0)
        {
            /* check to see if the MAC address in the driver is different from what is in the Wifi DCT */
            wwd_wifi_get_mac_address( &wiced_mac, WICED_STA_INTERFACE );
            if (memcmp(&wiced_mac, &app_info->dct_wifi->mac_address, sizeof(wiced_mac_t)) != 0)
            {
                /* It's different, set it from wifi DCT */
                WPRINT_APP_INFO(("SSDP App Setting MAC addr: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                        app_info->dct_wifi->mac_address.octet[0], app_info->dct_wifi->mac_address.octet[1],
                        app_info->dct_wifi->mac_address.octet[2], app_info->dct_wifi->mac_address.octet[3],
                        app_info->dct_wifi->mac_address.octet[4], app_info->dct_wifi->mac_address.octet[5]));

                if (wwd_wifi_set_mac_address( app_info->dct_wifi->mac_address ) != WWD_SUCCESS)
                {
                    WPRINT_APP_INFO(("Set MAC addr failed!\r\n"));
                }
            }
        }
    }

    /* Get app specific data from Non Volatile DCT */
    result = wiced_dct_read_lock((void **)&app_info->dct_app, WICED_FALSE, DCT_APP_SECTION, 0, sizeof(application_dct_t));
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Can't get app configuration!\r\n"));
        tag = APP_TAG_INVALID;
    }

    /* init the params for the ssdp_server */
    app_info->ssdp_params.notify_time       = app_info->dct_app->notify_time;
    app_info->ssdp_params.serve_page_path   = (char*)app_info->dct_app->serve_path_page;

_init_app_exit:
    /* may have been set to invalid by routines above */
    app_info->tag = tag;

    return app_info;
}


static void application_mainloop(application_info_t *app_info)
{
    wiced_result_t result;
    uint32_t events;

    WPRINT_APP_INFO(("Begin SSDP application mainloop\r\n"));

    /* always start right away for testing */

    result = wiced_ssdp_init(&app_info->ssdp_params, &app_info->ssdp_info, WICED_STA_INTERFACE );
    if (result !=WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error initializing SSDP system\r\n"));
        return;
    }

    /* set up the norify callback */
    result = wiced_ssdp_notify_register_callback( app_info->ssdp_info, ssdp_notify_callback, app_info );
    if (result !=WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error adding notify callback to SSDP system\r\n"));
    }

    /* service the console input */
    while (app_info->tag == APP_TAG_VALID)
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags(&app_info->events, APP_ALL_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & APP_EVENT_SHUTDOWN)
            break;

        if (events & APP_EVENT_SSDP_INIT)
        {
            if (app_info->ssdp_info == NULL)
            {
                result = wiced_ssdp_init(&app_info->ssdp_params, &app_info->ssdp_info, WICED_STA_INTERFACE );
                if (result !=WICED_SUCCESS)
                {
                    WPRINT_APP_INFO(("Error initializing SSDP system\r\n"));
                }
                else
                {
                    WPRINT_APP_INFO(("initializing SSDP returned %d\r\n", result));
                }
            }
            else
            {
                WPRINT_APP_INFO(("SSDP already initialized\r\n"));
            }
        }

        if (events & APP_EVENT_SSDP_DEINIT)
        {
            if (app_info->ssdp_info != NULL)
            {
                wiced_ssdp_deinit(app_info->ssdp_info);
            }
            else
            {
                WPRINT_APP_INFO(("SSDP not initialized\r\n"));
            }
            app_info->ssdp_info = NULL;
        }

        if (events & APP_EVENT_SSDP_MSEARCH)
        {
            if (app_info->ssdp_info != NULL)
            {
                ssdp_app_send_msearch(app_info);
            }
            else
            {
                WPRINT_APP_INFO(("SSDP not initialized\r\n"));
            }
        }

        if (events & APP_EVENT_SSDP_LOG_LEVEL_OFF)
        {
            app_info->ssdp_params.log_level         = SSDP_LOG_OFF;
            wiced_ssdp_set_log_level( app_info->ssdp_info, SSDP_LOG_OFF );
        }
        if (events & APP_EVENT_SSDP_LOG_LEVEL_LOW)
        {
            app_info->ssdp_params.log_level         = SSDP_LOG_LOW;
            wiced_ssdp_set_log_level( app_info->ssdp_info, SSDP_LOG_LOW );
        }
        if (events & APP_EVENT_SSDP_LOG_LEVEL_INFO)
        {
            app_info->ssdp_params.log_level         = SSDP_LOG_INFO;
            wiced_ssdp_set_log_level( app_info->ssdp_info, SSDP_LOG_INFO );
        }
        if (events & APP_EVENT_SSDP_LOG_LEVEL_DEBUG)
        {
            app_info->ssdp_params.log_level         = SSDP_LOG_DEBUG;
            wiced_ssdp_set_log_level( app_info->ssdp_info, SSDP_LOG_DEBUG );
        }

        if (events & APP_EVENT_SSDP_INFORMATION)
        {
            if (app_info->ssdp_info != NULL)
            {
                wiced_ssdp_dump_debug_info(app_info->ssdp_info);
            }
            else
            {
                WPRINT_APP_INFO(("SSDP not initialized\r\n"));
            }
        }
    };

    /*
     * Make sure we have been shut down.
     */

    if (app_info->ssdp_info != NULL)
    {
        wiced_ssdp_deinit(app_info->ssdp_info);
    }
    app_info->ssdp_info = NULL;

    WPRINT_APP_INFO(("End application mainloop\r\n"));
}

void application_start()
{
    application_info_t* app_info;
    wiced_result_t      result;

    if ((app_info = init_applicaiton()) == NULL)
    {
        return;
    }
    global_app_info = app_info;

    /* print out our current configuration */
    app_config_print_info(app_info);

    /* Bring up the network interface */
    result = wiced_network_up(app_info->dct_network->interface, WICED_USE_EXTERNAL_DHCP_SERVER, &app_info->ip_settings);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Bringing up network interface failed!\r\n"));
        free(app_info);
        return;
    }

    /*
     *  Fall into the main loop
     */

    application_mainloop(app_info);

    /*
     * Cleanup and exit.
     */

    global_app_info = NULL;
    shutdown_application(app_info);
    app_info = NULL;
}
