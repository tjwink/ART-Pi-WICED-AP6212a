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
 * Home Appliance Control Application
 *
 * This application demonstrates how a simple web page can be used to send
 * information to a UART when a button on the webpage is clicked. The application
 * mimics a very basic user interface to control a home appliance such as a washing
 * machine or dryer.
 *
 * Features demonstrated
 *      - WICED Configuration Mode
 *      - Wi-Fi client mode
 *      - HTTP web server with the ability to serve dynamic content
 *      - mDNS (or Bonjour) network service broadcasting
 *      - Powersave
 *
 * Application Instructions
 *   1. Modify the CONFIG_AP_SSID/CONFIG_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file as desired
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   3. After the download completes, the terminal displays WICED startup
 *      information and starts WICED Configuration Mode.
 *
 *   If you are unfamiliar with how WICED Configuration Mode works,
 *   please step through the config_mode snippet application in the
 *   apps/snip directory before proceeding further.
 *
 * Once the device has been configured then:
 *      - WICED connects as a client to the Wi-Fi AP selected during configuration mode
 *      - The app starts a web server that displays a web page with clickable buttons
 *      - A Wi-Fi client (eg. a computer) connected to the same home Wi-Fi network
 *    as the WICED module can then connect to the appliance web server using a web browser
 *
 * To view the Appliance web page in a web browser:
 *     - Open a web browser and go to http://192.168.1.99
 *       (replace this IP address with the IP of the WICED device; the IP
 *        address of the WICED device is available on the terminal after
 *        WICED Config Mode completes)
 *     - When a button on the webpage is clicked, button click information
 *       prints to the UART and is shown on the terminal
 *
 * Powersave Demonstration
 *   To demonstrate usage of the MCU & Wi-Fi powersave API in an application with
 *   an active network connection, a timed event is configured to intermittently
 *   enable, and then disable, MCU powersave mode. Using a WICED evaluation board,
 *   the power (current) profile can be measured on an oscilloscope by connecting
 *   to the testpoints labelled CURRENT & GND (J1). If you are unfamiliar with the
 *   WICED evaluation board or the WICED powersave API, refer to the WICED
 *   Evaluation Board User Guide and WICED Powersave Application Note in the
 *   <WICED-SDK>/Doc directory
 *

 * Notes.
 *   1. This application only works with Access Points configured for Open/WPA/WPA2
 *      security. It is not configured to work with APs that use WEP security!
 *   2. If your computer has an mDNS (or Bonjour) network service browser,
 *      the WICED device can be found by using the browser to search for
 *      and connect to the service called "Appliance Webserver". So this way
 *      you are able to discover the WICED device without relying on the IP
 *      address printed in the terminal.
 *
 */

#include "wiced.h"
#include "resources.h"
#include "http_server.h"
#include "gedday.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static int32_t        process_button_handler( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
static wiced_result_t powersave_toggle      ( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

START_OF_HTTP_PAGE_DATABASE(appliance_web_pages)
    { "/",                               "text/html",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_apps_DIR_appliance_DIR_top_web_page_top_html, },
    { "/button_handler",                 "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_button_handler, 0}, },
    { "/images/favicon.ico",             "image/vnd.microsoft.icon", WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_favicon_ico, },
    { "/images/cypresslogo.png",         "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_png, },
    { "/images/cypresslogo_line.png",    "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png, },
    { "/styles/buttons.css",             "text/css",                 WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_styles_DIR_buttons_css, },
    { "/scripts/general_ajax_script.js", "application/javascript",   WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_scripts_DIR_general_ajax_script_js, },
END_OF_HTTP_PAGE_DATABASE();


static wiced_http_server_t http_server;
static wiced_timed_event_t powersave_toggle_event;


/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    uint16_t max_sockets = 10;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Configure the device */
    wiced_configure_device( NULL );

    /* Bring up the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* Enable MCU powersave. MCU powersave API can be globally DISABLED in <WICED-SDK>/include/wiced_defaults.h */
    /* WARNING: Please read the WICED Powersave API documentation before enabling MCU powersave */
    /* wiced_platform_mcu_enable_powersave(); */

    /* Enable 802.11 powersave mode */
    wiced_wifi_enable_powersave_with_throughput( 40 );

    /* Register an event that toggles powersave every 2 seconds to
     * demonstrate power consumption with & without powersave enabled
     */
    wiced_rtos_register_timed_event( &powersave_toggle_event, WICED_HARDWARE_IO_WORKER_THREAD, &powersave_toggle, 2000, 0 );

    /* Start web-server */
    wiced_http_server_start( &http_server, 80, max_sockets, appliance_web_pages, WICED_STA_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );

    /* Advertise webpage services using Gedday */
    gedday_init( WICED_STA_INTERFACE, "wiced-appliance-app" );
    gedday_add_service( "Appliance Webserver", "_http._tcp.local", 80, 300, "" );
}


static int32_t process_button_handler( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    int params_len = strlen( url_parameters );
    char* found_loc = NULL;
    char  end_found = 0;
    UNUSED_PARAMETER( url_path );
    UNUSED_PARAMETER( http_message_body );

    /* Process the GET parameter list to determine if buttons have been pressed */

    /* Cycle through parameter list string until end or newline */
    while ( end_found == 0 )
    {
        /* Check if parameter is "btname" */
        if ( 0 == strncmp( url_parameters, "btname", 6 ) )
        {
            found_loc = (char*)&url_parameters[7];
        }

        /* Scan ahead to the next parameter or the end of the parameter list */
        while ( ( *url_parameters != '&' ) && ( *url_parameters != '\n' ) && ( params_len > 0 ) )
        {
            url_parameters++;
            params_len--;
        }

        if  ( *url_parameters != '&' )
        {
            end_found = 1;
        }


        if ( found_loc != NULL )
         {
             char* tmp = (char*)url_parameters;
             *tmp = '\x00';
             WPRINT_APP_INFO(( "\nDetected button press: %s\n\n", found_loc ));
         }


        if ( end_found == 0 )
        {
            /* Skip over the "&" which joins parameters if found */
            url_parameters++;
        }
    }

    /* Send the html page back to the client */
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_appliance_DIR_top_web_page_top_html );

    return 0;
}


wiced_result_t powersave_toggle( void* arg )
{
    static wiced_bool_t powersave_enabled = WICED_TRUE;
    if( powersave_enabled )
    {
        wiced_platform_mcu_disable_powersave( );
        powersave_enabled = WICED_FALSE;
    }
    else
    {
        wiced_platform_mcu_enable_powersave( );
        powersave_enabled = WICED_TRUE;
    }

    return WICED_SUCCESS;
}
