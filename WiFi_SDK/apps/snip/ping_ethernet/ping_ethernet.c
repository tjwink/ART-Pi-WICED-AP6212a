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
 * ICMP Ping over Ethernet Application
 *
 */

#include "wiced.h"
#include "ping_ethernet.h"


/******************************************************
 *                      Macros
 ******************************************************/

#define PING_TIMEOUT_MS          2000
#define PING_PERIOD_MS           3000

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
 *               Function Declarations
 ******************************************************/

static wiced_result_t send_ping              ( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_timed_event_t ping_timed_event;
static wiced_ip_address_t  ping_target_ip;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t result;

    /* Initialise the device */
    wiced_core_init( );
    wiced_network_init( );

    /* Bring up the network on the ethernet interface */
    result = wiced_network_up( WICED_ETHERNET_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    if ( result == WICED_SUCCESS )
    {
        uint32_t ipv4;

        /* The ping target is the gateway */
        wiced_ip_get_gateway_address( WICED_ETHERNET_INTERFACE, &ping_target_ip );

        /* Setup a regular ping event and setup the callback to run in the networking worker thread */
        wiced_rtos_register_timed_event( &ping_timed_event, WICED_NETWORKING_WORKER_THREAD, &send_ping, PING_PERIOD_MS, 0 );

        /* Print ping parameters */
        ipv4 = GET_IPV4_ADDRESS(ping_target_ip);
        WPRINT_APP_INFO(("Pinging %u.%u.%u.%u every %ums with a %ums timeout.\n",
                                                 (unsigned int)((ipv4 >> 24) & 0xFF),
                                                 (unsigned int)((ipv4 >> 16) & 0xFF),
                                                 (unsigned int)((ipv4 >>  8) & 0xFF),
                                                 (unsigned int)((ipv4 >>  0) & 0xFF),
                                                 (unsigned int)PING_PERIOD_MS,
                                                 (unsigned int)PING_TIMEOUT_MS));
    }
    else
    {
        WPRINT_APP_INFO(("Unable to bring up network connection\n"));
    }
}

static wiced_result_t send_ping( void* arg )
{
    uint32_t elapsed_ms;
    wiced_result_t status;

    WPRINT_APP_INFO(("Ping about to be sent\n"));

    status = wiced_ping( WICED_ETHERNET_INTERFACE, &ping_target_ip, PING_TIMEOUT_MS, &elapsed_ms );

    if ( status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Ping Reply : %lu ms\n", (unsigned long)elapsed_ms ));
    }
    else if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(("Ping timeout\n"));
    }
    else
    {
        WPRINT_APP_INFO(("Ping error\n"));
    }

    return WICED_SUCCESS;
}

