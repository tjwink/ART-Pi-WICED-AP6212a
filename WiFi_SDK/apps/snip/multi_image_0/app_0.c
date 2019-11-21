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
 * Multi-image Application 0
 *
 * This application snippet, along with multi_image_1 illustrate how to
 * use the Deepsleep / tiny_bootloader functionality to switch applications.
 *
 * When an application is compiled, a section of the .elf file contains
 * an AON section loaded into the Always On RAM (AON). That section
 * contains the tiny_bootloader. The purpose of the tiny_bootloader is
 * to re-load the application that was running before going into deep sleep,
 * as the RAM has been powered down.
 *
 * When the normal Bootloader loads an application, the value which the
 * tiny_bootloader uses to point to the application to load is set.
 *
 * By calling the function platform_deepsleep_set_boot(), we can tell the
 * tiny_bootloader which application to load.
 *
 * REQUIREMENTS:
 *  - All Applications using this technique MUST use the same exact DCT layout
 *  - All Applications using this technique MUST use the same exact AON RAM data
 *
 *  This application shows how to do that.
 *  Note that both multi_image_0 and multi_image_1 use the same common_dct.c & .h file.
 *      - This includes both the app_0_dct.h and app_1_dct.h files, which define the
 *        DCT entries for the two apps
 *  Note that both multi_image_0 and multi_image_1 use the same common_info.h file.
 *      - This file contains a Macro for both apps to use that defines the AON RAM use.
 *
 *  Note that when developing, there is an "always on" circuit on the platform that
 *       the IDE uses so that the system doesn't do a full power cycle on a reset.
 *       In order for the deep sleep to work properly, you must power cycle the
 *       device after programming from the IDE so that it will go into deep sleep.
 *
 *
 * Using this test:
 *
 * 1 - build snip/multi_image_1
 *
 *     make snip.multi_image_1-<platform>
 *
 * 2 - build and run snip/multi_image_0
 *
 *     make snip.multi_image_0-<platform> download download_apps run
 *
 * To switch applications:
 *
 *      In the command console, type "switch"
 *
 */

#include <ctype.h>
#include "wiced.h"
#include "wiced_low_power.h"
#include "waf_platform.h"       // for setting app index for tiny_bootloader

#include "command_console.h"
#include "command_console_dct.h"

/* common application information */
#include "../multi_image_0/common_dct.h"
#include "../multi_image_0/common_info.h"

#include "app_0.h"              /* This application header */

/******************************************************
 *                      Macros
 ******************************************************/

#define NUMBER_OF_PINGS                         3
#define DEADLINE_NETWORKING_TO_COMPLETE      1000

#define MAX_COMMAND_LENGTH                   (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH       (10)

#define APP_CONSOLE_COMMANDS \
    { (char*) "exit",           app_console_command,    0, NULL, NULL, (char *)"", (char *)"Exit application" }, \
    { (char*) "boot_connect",   app_console_command,    0, NULL, NULL, (char *)"", (char *)"Connect to AP on Boot (use AP in DCT)" }, \
    { (char*) "connect",        app_console_command,    0, NULL, NULL, (char *)"", (char *)"Connect to AP" }, \
    { (char*) "disconnect",     app_console_command,    0, NULL, NULL, (char *)"", (char *)"Disconnect from AP" }, \
    { (char*) "info",           app_console_command,    0, NULL, NULL, (char *)"", (char *)"Print DCT and App Context info" }, \
    { (char*) "ping",           app_console_command,    0, NULL, NULL, (char *)"", (char *)"Ping" }, \
    { (char*) "reboot",         app_console_command,    0, NULL, NULL, (char *)"", (char *)"Cold boot" }, \
    { (char*) "str_comm",       app_console_command,    0, NULL, NULL, (char *)"", (char *)"Change common DCT string" }, \
    { (char*) "str_app",        app_console_command,    0, NULL, NULL, (char *)"", (char *)"Change App 0 DCT string" }, \
    { (char*) "switch",         app_console_command,    0, NULL, NULL, (char *)"", (char *)"Switch to other app using deepsleep" }, \

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    APP_CONSOLE_CMD_EXIT = 0,

    APP_CONSOLE_CMD_BOOT_CONNECT,
    APP_CONSOLE_CMD_CONNECT,
    APP_CONSOLE_CMD_DISCONNECT,
    APP_CONSOLE_CMD_INFO,
    APP_CONSOLE_CMD_PING,
    APP_CONSOLE_CMD_REBOOT,
    APP_CONSOLE_CMD_STRING_COMMON,
    APP_CONSOLE_CMD_STRING_APP_0,
    APP_CONSOLE_CMD_SWITCH,

    APP_CONSOLE_CMD_MAX,
} APP_CONSOLE_CMDS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

int app_console_command(int argc, char *argv[]);

static wiced_result_t application_initialize( wiced_time_t cold_boot_time );
wiced_result_t application_connect( void );
wiced_result_t application_disconnect( void );
static void           deep_sleep( void );
static wiced_result_t application_ping_thrice( void );


/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_bool_t network_suspended = WICED_FALSE;

/* These variables are in AON */
COMMON_ALWAYS_ON_RAM_SHARED_VARIABLES

/* for the command console */
static char app_command_buffer[MAX_COMMAND_LENGTH];
static char app_command_history_buffer[MAX_COMMAND_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t app_command_table[] = {
    APP_CONSOLE_COMMANDS
    DCT_CONSOLE_COMMANDS
    CMD_TABLE_END
};

/* order must follow APP_CONSOLE_CMDS_T */
static char * command_lookup[APP_CONSOLE_CMD_MAX] = {
    "exit",
    "boot_connect",
    "connect",
    "disconnect",
    "info",
    "ping",
    "reboot",
    "str_comm",
    "str_app",
    "switch",
};

static app_0_context_t*  g_app_ctx;
/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    wiced_result_t      result;
    wiced_time_t        cold_boot_time = 0;

    WPRINT_APP_INFO(( "Starting Application 0\n" ));

    if ( !WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        /* measure cold boot startup time */
        wiced_time_get_time( &cold_boot_time );
    }

    result = application_initialize(cold_boot_time);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(( "Application 0 application_initialize() failed !\r\n\r\n"));
        exit(1);
    }

    /* Increment our wakeup count in the AON */
    wakeup_count++;

    WPRINT_APP_INFO(( "\r\n      Application 0 started! wakeup_count:%ld \r\n\r\n", wakeup_count));


    /*
     * Create the command console.
     */

    WPRINT_APP_INFO(("Start the command console\r\n"));
    result = command_console_init(STDIO_UART, sizeof(app_command_buffer), app_command_buffer, CONSOLE_COMMAND_HISTORY_LENGTH, app_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error starting the command console\r\n"));
        exit(1);
    }
    console_add_cmd_table(app_command_table);

    while(1)
    {
        wiced_rtos_delay_milliseconds( 1000 );

    };

}

static wiced_result_t  application_initialize( wiced_time_t cold_boot_time )
{
    wiced_result_t      result;
    app_0_context_t*    app_ctx = NULL;
    wiced_time_t        get_up_time;

    /* allocate our Application's context */
    app_ctx = (app_0_context_t*)calloc_named("App 0", 1, sizeof(app_0_context_t));
    if (app_ctx == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    wiced_dct_read_with_copy( (void**) &app_ctx->common_dct, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );

    /* assume cold boot */
    warmboot = WICED_FALSE;

    if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        warmboot = WICED_TRUE;      /* warm boot ! */

        result = wiced_resume_after_deep_sleep( );
        if (result == WICED_SUCCESS)
        {
            /* Resume network interface */
            if (connected == WICED_TRUE)
            {
                result = wiced_network_resume_after_deep_sleep( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
                if (result != WICED_SUCCESS)
                {
                    WPRINT_APP_INFO(( "wiced_network_resume_after_deep_sleep() failed\n" ));
                    connected = WICED_FALSE;
                }
            }
            if (go_to_sleep_time != 0)
            {
                wiced_time_get_time( &get_up_time );
                WPRINT_APP_INFO(( "\nWarm Boot with%s connect time from sleep: %ld\n",
                                    (connected == WICED_TRUE) ? "" : "out",
                                    (get_up_time - go_to_sleep_time) ));
            }
        }
    }
    else
    {
        /* cold boot */
        result = wiced_init( );
        if ( result == WICED_SUCCESS )
        {
            result = wiced_wifi_enable_powersave( );
            if( result != WICED_SUCCESS)
            {
                WPRINT_APP_INFO(( "enable powersave failed\n" ));
            }

            if (app_ctx->common_dct.connect_on_cold_boot == WICED_TRUE)
            {
                application_connect();
            }

            if (cold_boot_time != 0)
            {
                wiced_time_get_time( &get_up_time );
                WPRINT_APP_INFO(( "\nCold boot with%s connect time: %ld ms\n",
                        (app_ctx->common_dct.connect_on_cold_boot == WICED_TRUE) ? "" : "out",
                        (get_up_time - cold_boot_time) ));
            }
        }
    }

    if (result == WICED_SUCCESS)
    {
        app_ctx->tag = APP_0_VALID_TAG;
        g_app_ctx = app_ctx;
    }

    /* clear this so we don't get confused */
    go_to_sleep_time = 0;

    return result;
}

wiced_result_t application_connect( void )
{
    wiced_result_t result;

    platform_dct_wifi_config_t*     dct_wifi_config = NULL;
    wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_wifi_config ) );
    if ((dct_wifi_config != NULL) && (dct_wifi_config->stored_ap_list[0].details.SSID.length == 0) )
    {
        WPRINT_APP_INFO(( "To connect to an AP:\n" ));
        WPRINT_APP_INFO(( "   - Set up your AP info in wifi_dct_config.h \n" ));
        WPRINT_APP_INFO(( "   or\n" ));
        WPRINT_APP_INFO(( "   Type 'help' and use the dct_wifi_ap_list xx xxx xxx commands to set up AP info. \n" ));
        wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );
        return WICED_ERROR;
    }

    /* Bring up the network interface and connect to the Wi-Fi network */
    WPRINT_APP_INFO(( "Calling wiced_network_up().\n" ));
    result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if (result == WICED_SUCCESS)
    {
        wiced_bss_info_t*    ap_info;
        wiced_security_t    security;

        connected = WICED_TRUE;
        /* Find name of connection */
        ap_info = (wiced_bss_info_t*)malloc(sizeof(wiced_bss_info_t));
        if (ap_info != NULL)
        {
            if (wiced_wifi_get_ap_info( ap_info, &security ) == WICED_SUCCESS)
            {
                strlcpy(current_ap, (char *)&ap_info->SSID, sizeof(current_ap));
            }
            free(ap_info);
        }
        return WICED_SUCCESS;
    }

    return WICED_ERROR;
}

wiced_result_t application_disconnect( void )
{
    wiced_result_t result;

    /* Bring up the network interface and connect to the Wi-Fi network */
    result = wiced_network_down( WICED_STA_INTERFACE );
    if (result == WICED_SUCCESS)
    {
        connected = WICED_FALSE;
        /* clear connection ap name */
        strlcpy(current_ap, APP_NO_CONNECTION_STR, sizeof(current_ap));
        return WICED_SUCCESS;
    }

    return WICED_ERROR;
}

static void deep_sleep( void )
{
    int i;

    /* Wait till all packets are sent */
    for ( i = 0; i < DEADLINE_NETWORKING_TO_COMPLETE; ++i )
    {
        if ( wiced_deep_sleep_is_networking_idle( WICED_STA_INTERFACE ) )
        {
            break;
        }

        wiced_rtos_delay_milliseconds( 1 );
    }

    /* Suspend network timers */
    if ( !network_suspended )
    {
        if ( wiced_network_suspend( ) == WICED_SUCCESS )
        {
            network_suspended = WICED_TRUE;
        }
    }

    /*
     * Wakeup system monitor thread so it can:
     *     - kick watchdog
     */
    wiced_wakeup_system_monitor_thread( );

    /* Enable power save */
    wiced_platform_mcu_enable_powersave( );

    /* Figure time from sleep to wake up (after connect complete) */
    wiced_time_get_time( &go_to_sleep_time );

    /* Deep-sleep */
    wiced_rtos_delay_milliseconds( WIFI_SLEEP_TIME );

    /* NOTE: We should never get here - tinybootloader will load the other application */

    /* Disable power save */
    wiced_platform_mcu_disable_powersave( );

    /* Resume network timers */
    if ( network_suspended )
    {
        if ( wiced_network_resume( ) == WICED_SUCCESS )
        {
            network_suspended = WICED_FALSE;
        }
    }
}

static wiced_bool_t get_ping_destination( wiced_ip_address_t* ip_address )
{
    return ( wiced_ip_get_gateway_address( WICED_STA_INTERFACE, ip_address ) == WICED_SUCCESS ) ? WICED_TRUE : WICED_FALSE;
}

static wiced_result_t send_ping( void )
{
    const uint32_t     ping_timeout = 1000;
    uint32_t           elapsed_ms;
    wiced_result_t     result = WICED_SUCCESS;
    wiced_ip_address_t ping_target_ip;

    if ( !get_ping_destination( &ping_target_ip ) )
    {
        WPRINT_APP_INFO(("Unknown ping destination.\n"));
        result = WICED_ERROR;
    }
    else
    {
        WPRINT_APP_INFO(("Pinging %u.%u.%u.%u. timeout:%ld -> ",
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 24) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 16) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  8) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  0) & 0xFF), ping_timeout ));

        result = wiced_ping( WICED_STA_INTERFACE, &ping_target_ip, ping_timeout, &elapsed_ms );
    }

    if ( result == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(( "Ping timeout\n" ));
    }
    else if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Ping error: %d\n", (int)result ));
    }
    else
    {
        WPRINT_APP_INFO(( "Success\n"));
    }

    return result;
}

static void print_wlan_log( void )
{
    static char buffer[200];
    (void)wwd_wifi_read_wlan_log( buffer, sizeof( buffer ) );
}

static wiced_result_t application_ping_thrice( void )
{
    /* Send an ICMP ping to the gateway */
    int i;

    for ( i = 0; i < NUMBER_OF_PINGS; ++i )
    {
        if (send_ping( ) != WICED_SUCCESS)
        {
            print_wlan_log( );
        }
    }

    return WICED_SUCCESS;
}


static void application_print_info( app_0_context_t* app_ctx )
{
    multi_image_dct_t* app_dct;

    /* print common and application specific info */

    WPRINT_APP_INFO(("\n---- Application 0 ----\n"));
    WPRINT_APP_INFO(("\nCommon Info:\n"));
    WPRINT_APP_INFO(("     warmboot:%s\n", (warmboot == WICED_FALSE) ? "No" : "Yes"));
    WPRINT_APP_INFO(("    connected:%s\n", (connected == WICED_FALSE) ? "No" : "Yes"));
    WPRINT_APP_INFO((" wakeup_count:%ld\n", wakeup_count));
    WPRINT_APP_INFO(("   current_ap:%s\n", current_ap));

    WPRINT_APP_INFO(("\nApplication DCT:\n"));

    wiced_dct_read_lock( (void**) &app_dct, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );

    WPRINT_APP_INFO(("  Common DCT:\n"));
    WPRINT_APP_INFO(( "   connect_on_cold_boot: %d\n", app_dct->connect_on_cold_boot ));
    WPRINT_APP_INFO(("           string_common:%s\n", app_dct->string_common));

    WPRINT_APP_INFO(("  App 0 DCT:\n"));
    WPRINT_APP_INFO(("            app_0_string:%s\n", app_dct->app_0_data.app_0_string));

    wiced_dct_read_unlock( (void*) app_dct, WICED_TRUE);
}

void application_change_boot_connecct( char* value )
{
    if ((g_app_ctx != NULL) && (g_app_ctx->tag == APP_0_VALID_TAG))
    {
        if (value != NULL)
        {
            if ((*value == '0') || (tolower((int)*value) == 'n'))
            {
                g_app_ctx->common_dct.connect_on_cold_boot = WICED_FALSE;
                wiced_dct_write(&g_app_ctx->common_dct, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );
            }
            else if ((*value == '1') || (tolower((int)*value) == 'y'))
            {
                g_app_ctx->common_dct.connect_on_cold_boot = WICED_TRUE;
                wiced_dct_write(&g_app_ctx->common_dct, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );
            }
        }
        WPRINT_APP_INFO(( "connect_on_cold_boot: %d\n", g_app_ctx->common_dct.connect_on_cold_boot ));
    }

}

static void application_change_string_common( char * string )
{
    if ((g_app_ctx != NULL) && (g_app_ctx->tag == APP_0_VALID_TAG))
    {
        if (string != NULL)
        {
            strlcpy(g_app_ctx->common_dct.string_common, string, sizeof(g_app_ctx->common_dct.string_common));
            wiced_dct_write(&g_app_ctx->common_dct, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );
        }
        WPRINT_APP_INFO(( "string_common: %s\n", g_app_ctx->common_dct.string_common ));
    }
}

static void application_change_string_app_0( char * string )
{
    if ((g_app_ctx != NULL) && (g_app_ctx->tag == APP_0_VALID_TAG))
    {
        if (string != NULL)
        {
            strlcpy(g_app_ctx->common_dct.app_0_data.app_0_string, string, sizeof(g_app_ctx->common_dct.app_0_data.app_0_string));
            wiced_dct_write(&g_app_ctx->common_dct, DCT_APP_SECTION, 0, sizeof( multi_image_dct_t ) );
        }
        WPRINT_APP_INFO(( "app_0_data.app_0_string: %s\n", g_app_ctx->common_dct.app_0_data.app_0_string ));
    }
}

int app_console_command(int argc, char *argv[])
{
    int i;

    WPRINT_APP_INFO(("Received command: %s\n", argv[0]));

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < APP_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i], argv[0]) == 0)
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
            break;

        case APP_CONSOLE_CMD_BOOT_CONNECT:
            application_change_boot_connecct( argv[1] );
            break;

        case APP_CONSOLE_CMD_CONNECT:
            application_connect();
            break;

        case APP_CONSOLE_CMD_DISCONNECT:
            application_disconnect();
            break;

        case APP_CONSOLE_CMD_STRING_COMMON:
            application_change_string_common( argv[1] );
            break;

        case APP_CONSOLE_CMD_STRING_APP_0:
            application_change_string_app_0( argv[1] );
            break;

        case APP_CONSOLE_CMD_INFO:
            application_print_info(g_app_ctx);
            break;

        case APP_CONSOLE_CMD_PING:
            application_ping_thrice();
            break;

        case APP_CONSOLE_CMD_REBOOT:
            wiced_framework_reboot();
            break;

        case APP_CONSOLE_CMD_SWITCH:
            /* set the boot_detail so on next cold boot we boot into the new application as well */
            wiced_framework_set_boot(DCT_APP1_INDEX, WICED_FRAMEWORK_LOAD_ONCE);

            /* set the deepsleep boot application index */
            platform_deepsleep_set_boot(DCT_APP1_INDEX);

            WPRINT_APP_INFO(( "enter DeepSleep -->\n" ));

            /* Enter deep-sleep - we should not come back here - we set a new application to load with platform_deepsleep_set_boot() */
            deep_sleep( );

            WPRINT_APP_INFO(( "Exit DeepSleep. You need to disconnect from the IDE. Cycle Power to do this. \n" ));
            break;

        default:
            break;
    }

    return ERR_CMD_OK;
}
