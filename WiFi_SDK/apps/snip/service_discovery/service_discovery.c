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
 * Service Discovery Application
 *
 * Features demonstrated
 *  - Gedday service discovery library
 *
 * Application Instructions
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in wifi_config_dct.h header file to match your Wi-Fi access point.
 *  2. Connect MacBook to the same WiFi network which is used in step#1.
 *     Run dns-sd on MacBook using following command to host the mDNS service.
 *          $ dns-sd -R WicedmDNSService _wiced123 local 5353
 *  3. Build and download the application as described in WICED Quick Start Guide.
 *  4. Now WICED device should be able to discover the mDNS service on the network, which is hosted in step #2.
 *
 */

#include "wiced.h"
#include "gedday.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MDNS_SERVICE_TOBE_DISCOVERED        "_wiced123._tcp.local"

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t  res;
    gedday_service_t service_result;
    int i = 0;
    char service_name[] = MDNS_SERVICE_TOBE_DISCOVERED;

    wiced_init( );

    res = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    if (res != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Failed to bring network up. Error [%d]\n", res));
        return;
    }

    res = gedday_init(WICED_STA_INTERFACE, "WICED_Gedday_Discovery_Example");
    if (res != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Failed to init Gedday. Error [%d]\n", res));
        return;
    }

    WPRINT_APP_INFO(("Discovering service [%s]\n", MDNS_SERVICE_TOBE_DISCOVERED));
    while ( 1 )
    {
        WPRINT_APP_INFO(("Discover Try [%d]\n", i++));
        memset(&service_result, 0x0, sizeof(service_result));
        res = gedday_discover_service(service_name, &service_result);
        if (res != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(("Service Discovery Failed with Error [%d]\n", res));
        }
        else
        {
            WPRINT_APP_INFO(("Service Discovery Result:\n"));
            WPRINT_APP_INFO(("  Service Name  = [%s]\n", service_result.service_name));
            WPRINT_APP_INFO(("  Instance Name = [%s]\n", service_result.instance_name));
            WPRINT_APP_INFO(("  Host Name     = [%s]\n", service_result.hostname));
            WPRINT_APP_INFO(("  TXT Record    = [%s]\n", service_result.txt));
            WPRINT_APP_INFO(("  Port          = [%d]\n", service_result.port));

            /* Free the records */
            if (service_result.instance_name != NULL)
            {
                free(service_result.instance_name);
            }
            if (service_result.hostname != NULL)
            {
                free(service_result.hostname);
            }
            if (service_result.txt != NULL)
            {
                free(service_result.txt);
            }
        }

        wiced_rtos_delay_milliseconds( 1000 );
    }
}
