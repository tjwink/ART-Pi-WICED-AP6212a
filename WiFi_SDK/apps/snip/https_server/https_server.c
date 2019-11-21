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
 * HTTPS Webserver Application
 *
 * This app connects to a Wi-Fi network as a STA (client), then starts
 * a secure web-server and waits for requests.
 * It also starts a non-secure web-server which provides a redirect that
 * redirects any non-secure requests to the secure webpages.
 *
 * Features demonstrated
 *  - Wi-Fi STA (client) mode to serve HTTPS web pages
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   3. The app will display on the terminal :  Server ready at u.v.w.x
 *      Type this IP address into a web browser which is connected to
 *      the same Wi-Fi network as the WICED board.
 *      You will see the web browser load the secure web page.
 *
 */

#include "wiced.h"
#include "https_server.h"
#include "http_server.h"
#include "wiced_resource.h"
#include "resources.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTPS_PORT   443
#define HTTP_PORT    80

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_https_server_t https_server;

/* Config bypassed in local makefile and wifi_config_dct.h */
/*
static const configuration_entry_t const app_config[] =
{
    {0,0,0,0}
};
*/

START_OF_HTTP_PAGE_DATABASE(https_pages)
    ROOT_HTTP_PAGE_REDIRECT("/apps/https_server/https_server_top.html"),
    { "/apps/https_server/https_server_top.html",  "text/html",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_apps_DIR_https_server_DIR_https_server_top_html, },
    { "/images/favicon.ico",                       "image/vnd.microsoft.icon", WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_favicon_ico, },
    { "/images/cypresslogo.png",                   "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_png, },
    { "/images/cypresslogo_line.png",              "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png, },
END_OF_HTTP_PAGE_DATABASE();

wiced_tls_identity_t tls_identity;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    platform_dct_security_t* dct_security = NULL;
    wiced_result_t result;
    uint16_t max_sockets = 10;
    wiced_ip_address_t address;
    uint32_t ipv4;

    /* Initialise the device */
    wiced_init( );

    /* Configure the device */
    /* wiced_configure_device( app_config ); */ /* Config bypassed in local makefile and wifi_config_dct.h */

    /* Bring up the network on the STA interface */
    WPRINT_APP_INFO(( "Connect to the network\n" ));
    result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to bring up network connection\n"));
        return;
    }

    /* Lock the DCT to allow us to access the certificate and key */
    WPRINT_APP_INFO(( "Read the certificate Key from DCT\n" ));
    result = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return;
    }

    /* Setup TLS identity */
    result = wiced_tls_init_identity( &tls_identity, dct_security->private_key, strlen( dct_security->private_key ), (uint8_t*) dct_security->certificate, strlen( dct_security->certificate ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Unable to initialize TLS identity. Error = [%d]\n", result ));
        return;
    }

    /* Start the HTTPS server */
    WPRINT_APP_INFO(( "Start HTTPS server\n" ));
    result = wiced_https_server_start( &https_server, HTTPS_PORT, max_sockets, https_pages, &tls_identity, WICED_STA_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Unable start HTTPS server. Error = [%d]\n", result ));
        return;
    }

    /* Finished accessing the certificates */
    result = wiced_dct_read_unlock( dct_security, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        return;
    }

    WPRINT_APP_INFO(( "Get IPv4 address\n" ));
    result = wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address);
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Unable to get IPv4 address. Error = [%d]\n", result ));
        return;
    }

    ipv4 = GET_IPV4_ADDRESS(address);
    WPRINT_APP_INFO(( "Server ready at %u.%u.%u.%u\n",
                                             (unsigned int)((ipv4 >> 24) & 0xFF),
                                             (unsigned int)((ipv4 >> 16) & 0xFF),
                                             (unsigned int)((ipv4 >>  8) & 0xFF),
                                             (unsigned int)((ipv4 >>  0) & 0xFF) ));


}
