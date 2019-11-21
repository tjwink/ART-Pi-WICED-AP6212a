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

 /* AVS Authorization
 *
 * This application snippet helps to generate the refresh token for AVS( Amazon Voice Services).
 *
 *  NOTE : Before running this application on WICED, AVS registration is needed for certain
 *         informational parameters as part of the demo
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Joins a Wi-Fi network
 *  .
 *  Open a web browser on your computer connected to same Wi-Fi access point that the eval board
 *  is connected to and type in the ip address of the wiced device along with the port.
 *  Enter the details related to AVS on the webpage hosted by the eval board
 *  The page should direct to amazon website, login using your AVS registered account
 *  The token gets generated which can be downloaded to a text file.
 *
 */

#include "wiced.h"
#include "http_server.h"
#include "http.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_SOCKETS       ( 5 )
#define BUFFER_LENGTH     (2048)
#define SERVER_PORT        3000
#define AVS_HOST_URL "www.amazon.com"


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

static wiced_http_server_t http_server;

 static START_OF_HTTP_PAGE_DATABASE(web_pages)
  ROOT_HTTP_PAGE_REDIRECT("/main.html"),
  { "/main.html",    "text/html",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data   = &resources_apps_DIR_avs_DIR_main_html, },
  { "/authresponse", "text/html",                WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_apps_DIR_avs_DIR_auth_html,},
  { "/images/cypresslogo.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_png,        },
  { "/img0.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_apps_DIR_avs_DIR_img0_png,        },
  { "/img1.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_apps_DIR_avs_DIR_img1_png,        },
  { "/img1a.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_apps_DIR_avs_DIR_img1a_png,        },
  END_OF_HTTP_PAGE_DATABASE();


/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    wiced_ip_address_t ip_address;
    wiced_result_t result;

    /* Initialise the device */
    wiced_init();

    /* Bring up the STA (client) interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* Start a web server on the STA interface */
    wiced_http_server_start( &http_server, SERVER_PORT, MAX_SOCKETS, web_pages, WICED_STA_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );

    /* Resolving IP address of HTTPS Server */
    WPRINT_APP_INFO( ( "Resolving IP address of AVS server\n" ) );

    result = wiced_hostname_lookup(AVS_HOST_URL, &ip_address, 10000, WICED_STA_INTERFACE);
    WPRINT_APP_INFO(("Resolved Broker IP: %u.%u.%u.%u\n\n", (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0)));
    if ( result == WICED_ERROR || ip_address.ip.v4 == 0 )
    {
        WPRINT_APP_INFO(("Error in resolving DNS\n"));
        return;
    }
    WPRINT_APP_INFO( ( "Open a browser and type the IP address of the WICED device with the port( default port 3000)\n" ) );
}

