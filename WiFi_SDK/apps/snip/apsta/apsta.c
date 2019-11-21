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
 * Concurrent APSTA Application
 *
 * This application snippet demonstrates how to use
 * the WICED Wi-Fi client and softAP interfaces at the same time.
 *
 * Features demonstrated
 *  - Wi-Fi client mode (to send a regular ICMP ping to an AP)
 *  - Wi-Fi softAP mode (to enable Wi-Fi clients to connect to the WICED webserver)
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Modify the SOFT_AP_SSID/SOFT_AP_PASSPHRASE Wi-Fi credentials
 *     as desired
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Joins a Wi-Fi network and pings the gateway. Ping results are
 *    printed to the UART and appear on the terminal
 *  - Starts a softAP and a webserver on the AP interface
 *
 * To connect a Wi-Fi client (eg. computer) to the softAP webserver:
 *  - Connect your computer to the softAP SSID configured in wifi_config_dct.h
 *  - Open a web browser
 *  - Enter wiced.com as the URL; a simple web page appears
 *    (or alternately, enter 192.168.0.1 as the URL, this is the
 *     IP address of the softAP interface)
 *
 * TROUBLESHOOTING
 *   If you are having difficulty connecting the web browser to the
 *   WICED softAP webserver, try the following:
 *   1. Disconnect other network interfaces from the computer (eg. wired ethernet)
 *   2. Check that your computer received a valid IP address eg. 192.168.0.2
 *   3. Try clearing the web browser cache and try connecting again
 *
 */

#include "wiced.h"
#include "http_server.h"
#include "resources.h"
#include "dns_redirect.h"
#ifdef __IAR_SYSTEMS_ICC__
  #include "iar_unistd.h"
#else
  #include "unistd.h"
#endif
#include "wiced_log.h"
#include "command_console_commands.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define PING_PERIOD         1000
#define PING_TIMEOUT        900
#define PING_TOTAL_COUNT    3

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
#ifdef INCLUDE_COMMAND_CONSOLE
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];
#endif

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t send_ping ( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

static wiced_http_server_t ap_http_server;

START_OF_HTTP_PAGE_DATABASE(ap_web_pages)
    ROOT_HTTP_PAGE_REDIRECT("/apps/apsta/ap_top.html"),
    { "/apps/apsta/ap_top.html",         "text/html",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_apps_DIR_apsta_DIR_ap_top_html,    },
    { "/images/favicon.ico",             "image/vnd.microsoft.icon",          WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_favicon_ico,            },
    { "/images/cypresslogo.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_png,        },
    { "/images/cypresslogo_line.png",    "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png,   },
END_OF_HTTP_PAGE_DATABASE();

static dns_redirector_t    dns_redirector;
static wiced_timed_event_t ping_timed_event;
static wiced_ip_address_t  ping_target_ip;

#ifdef INCLUDE_COMMAND_CONSOLE
static const command_t commands[] =
{
    ALL_COMMANDS
    CMD_TABLE_END
};
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

static int render_log_output(WICED_LOG_LEVEL_T level, char *logmsg)
{
    UNUSED_PARAMETER(level);

    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}

static wiced_result_t apsta_http_receive_callback ( wiced_http_response_stream_t* stream, uint8_t** data, uint16_t* data_length )
{
    if(*data && data_length)
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "http response stream callback, received %d bytes\n", *data_length);
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "data: \n");
        for(int i = 0; i < *data_length; i++)
        {
            wiced_log_printf("%c", (*data)[i]);
        }
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "\n");
    }
    return WICED_SUCCESS;
}

void application_start(void)
{
    uint16_t max_sockets = 10;

    /* Initialise the device */
    wiced_init();

    wiced_log_init(WICED_LOG_INFO, render_log_output, NULL);
    wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "wiced logging system is initialized\n");

#ifdef INCLUDE_COMMAND_CONSOLE
    /* Run the console for debugging issues ... */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );

#endif

    /* Bring up the STA (client) interface ------------------------------------------------------- */
    wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

    /* The ping target is the gateway that the STA is connected to*/
    wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &ping_target_ip );

    /* Print ping description to the UART */
    wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "Pinging %u.%u.%u.%u %d times every %ums with a %ums timeout.\n", (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 24) & 0xFF),
                                                                              (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 16) & 0xFF),
                                                                              (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  8) & 0xFF),
                                                                              (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  0) & 0xFF),
                                                                              PING_TOTAL_COUNT,
                                                                              PING_PERIOD,
                                                                              PING_TIMEOUT );

    /* Setup a regular ping event and setup the callback to run in the networking worker thread */
    wiced_rtos_register_timed_event( &ping_timed_event, WICED_NETWORKING_WORKER_THREAD, &send_ping, PING_PERIOD, 0 );


    /* Bring up the softAP interface ------------------------------------------------------------- */
    if(wiced_network_up(WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &ap_ip_settings) == WICED_SUCCESS)
    {
        platform_dct_wifi_config_t* dct_wifi_config = NULL;

        if ( wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_wifi_config ) ) == WICED_SUCCESS )
        {
            wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "softAP \"%s\" is up\n", dct_wifi_config->soft_ap_settings.SSID.value);
        }
        wiced_dct_read_unlock( dct_wifi_config, WICED_FALSE );
    }
    else
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "Failed to bring up the softAP interface\n");
    }

    /* Start a DNS redirect server to redirect wiced.com to the AP webserver database*/
    wiced_dns_redirector_start( &dns_redirector, WICED_AP_INTERFACE );

    /* Start a web server on the AP interface */
    wiced_http_server_start( &ap_http_server, 80, max_sockets, ap_web_pages, WICED_AP_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );
    wiced_http_server_register_callbacks(&ap_http_server, apsta_http_receive_callback, NULL);

}


/* Sends a ping to the target */
static wiced_result_t send_ping( void *arg )
{
    uint32_t elapsed_ms;
    wiced_result_t status;
    static int ping_count = 0;

    status = wiced_ping( WICED_STA_INTERFACE, &ping_target_ip, PING_TIMEOUT, &elapsed_ms );

    if ( status == WICED_SUCCESS )
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO,"Ping #%d Reply : %lu ms\n", ping_count, (unsigned long)elapsed_ms );
    }
    else if ( status == WICED_TIMEOUT )
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "Ping timeout\n");
    }
    else
    {
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "Ping error\n");
    }

    if (++ping_count > PING_TOTAL_COUNT){
        wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "deregister ping timer\n");
        wiced_rtos_deregister_timed_event( &ping_timed_event);
    }
    return WICED_SUCCESS;
}

