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
 * softAP Clients Application
 *
 * This application snippet demonstrates how to retrieve a list
 * of Wi-Fi clients associated to the WICED softAP and obtain the
 * RSSI (signal strength) for each client.
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode and related APIs
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Using a Wi-Fi client such as an iOS or Android device, connect to
 *   the softAP with credentials : WICED Soft AP / 12345678
 *
 *   One per second, connection details of the client are printed to
 *   the serial port. The details of any additional clients are also
 *   printed if additional clients join.
 *
 */

#include "wiced.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_SOFT_AP_CLIENTS  (5)
/* this macro is just to prove that ip-address-list buffer size is independent of Mac-address list */
#define MAX_SOFT_AP_IP_CLIENTS (5)

#define MAX_IPV4_ADDRESS_STRING_LENGTH (17)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    int         count;
    wiced_mac_t mac_list[MAX_SOFT_AP_CLIENTS];
} client_info_t;

typedef struct
{
    int count;
    wiced_ip_address_t ip_address_list[MAX_SOFT_AP_IP_CLIENTS];
} client_ip_info_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static void convert_ip_addr_to_ip_string(wiced_ip_address_t* ip_addr, char* const ip_str)
{
    int     i;
    int     offset;
    char    *buf;

    /* TODO: Handle IPv6 address as well */

    offset = 0;
    buf = ip_str;
    for (i = 3; i >= 0; i--)
    {
        /* Cover the octet to string */
        offset = unsigned_to_decimal_string( ((GET_IPV4_ADDRESS(*ip_addr) >> (i << 3)) & 0xFF), buf, 1, 4 );
        buf += offset;
        /* Add '.' */
        *buf++ = '.';
    }
    /* Add NULL termination for the last octet */
    buf--;
    *buf = '\0';

    return;
}

void application_start(void)
{
    int32_t         rssi = 0;
    int             client_number = 0;
    wiced_result_t  result;

    client_info_t       client_info;
    client_ip_info_t    client_ip_info;

    char host_ip[MAX_IPV4_ADDRESS_STRING_LENGTH] = { 0 };

    /* Initialise Wiced system */
    wiced_init();

    /* Bring up the softAP interface  */
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &ap_ip_settings );

    while ( 1 )
    {
        /* Our client_info can only fit information about no more than 5 clients (4 if the STA interface is also used)*/
        client_info.count = MAX_SOFT_AP_CLIENTS;

        /* Get the list of the stations connected to Wiced soft AP */
        result = wiced_wifi_get_associated_client_list( &client_info, sizeof( client_info ) );
        if ( result != WICED_SUCCESS )
        {
            if ( result == WICED_WLAN_BUFTOOSHORT )
                WPRINT_APP_INFO( ("Number of Clients connected exceeds the allocated buffer size,\n consider increasing MAX_SOFT_AP_CLIENTS\r\n") );
            else
                WPRINT_APP_INFO( ("Error. Can't get the list of clients\r\n") );

            wiced_rtos_delay_milliseconds( 1000 );
            continue;
        }

        /* get_clients_ip_address_list internally uses associated_client_list() to get the list of connected client's
         * mac-address and then fetches their ip-addresses from DHCP server; hence, it should be safe to iterate the ip-address-list
         * with the same iterator used for associated client's mac-address list
         */
        client_ip_info.count = MAX_SOFT_AP_IP_CLIENTS;

        result = wiced_network_get_clients_ip_address_list( (void *)&client_ip_info );
        if ( result != WICED_SUCCESS )
        {
            if( result == WICED_WLAN_BUFTOOSHORT )
            {
                WPRINT_APP_INFO( ("Number of Clients connected exceeds the DHCP server cache...consider increasing DHCP cache size\r\n") );
            }
            else if( result == WICED_ERROR )
            {
                WPRINT_APP_INFO( ("Number of Clients connected exceeds the allocated IP-address-list buffer size...consider increasing MAX_SOFT_AP_IP_CLIENTS\r\n") );
            }
            else
            {
                WPRINT_APP_INFO( ("Error fetching IP-address list\r\n") );
            }
            wiced_rtos_delay_milliseconds( 1000 );
            continue;
        }

        if ( client_info.count == 0 )
        {
            WPRINT_APP_INFO( ("Clients connected 0..\r\n") );
        }
        else
        {
            WPRINT_APP_INFO( ("Clients connected %d..\r\n", client_info.count) );
            for ( client_number = 0; client_number < client_info.count; client_number++ )
            {
                /* Get the RSSI of every client currently connected to the soft AP */
                result = wiced_wifi_get_ap_client_rssi( &rssi, &client_info.mac_list[ client_number ] );

                convert_ip_addr_to_ip_string(&client_ip_info.ip_address_list[client_number], host_ip);

                if ( result != WICED_SUCCESS )
                {
                    WPRINT_APP_INFO( ("Error. Can't get rssi of the client\r\n") );
                }
                else
                {
                    WPRINT_APP_INFO( ("-------------------------------------------------------\r\n") );
                    WPRINT_APP_INFO( ("| %d | %02x:%02x:%02x:%02x:%02x:%02x | %3lddBm   | %16s |\r\n",
                        client_number,
                        client_info.mac_list[client_number].octet[0], client_info.mac_list[client_number].octet[1], client_info.mac_list[client_number].octet[2], client_info.mac_list[client_number].octet[3], client_info.mac_list[client_number].octet[4], client_info.mac_list[client_number].octet[5],
                        rssi, host_ip ) );
                }
            }
            WPRINT_APP_INFO( ("-------------------------------------------------------\r\n") );
        }

        /* Sleep for a second and do it again */
        WPRINT_APP_INFO( ("Waiting for an update...\r\n\r\n") );
        wiced_rtos_delay_milliseconds( 1000 );
    }
}


