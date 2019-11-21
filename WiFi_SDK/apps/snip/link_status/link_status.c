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
 * Link Status Application
 *
 * This application demonstrates how to asynchronously handle
 * changes in the Wi-Fi link connection status.
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - Wi-Fi link status callback API
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *
 * After download, the app joins the configured AP as a client.
 * To demonstrate the link up/link down API:
 *   - Power OFF the AP
 *   - Wait for a link down message to appear on the terminal
 *   - Power ON the AP
 *   - Wait for a link up message to appear on the terminal
 *
 */

#include "wiced.h"

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

static void link_up  ( void );
static void link_down( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_semaphore_t link_up_semaphore;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    /* Initialise the device */
    wiced_init();

    /* Register callbacks */
    wiced_network_register_link_callback( link_up, link_down, WICED_STA_INTERFACE );

    /* Bring up the STA (client) interface ------------------------------------------------------- */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* Initialise semaphore to notify when the network comes up */
    wiced_rtos_init_semaphore( &link_up_semaphore );

    WPRINT_APP_INFO( ("\nSwitch your AP off\n\n") );

    /* The link_up() function sets a semaphore when the link is back up. Wait here until the semaphore is set */
    wiced_rtos_get_semaphore( &link_up_semaphore, WICED_NEVER_TIMEOUT );

    /* Clean up and halt */
    wiced_rtos_deinit_semaphore( &link_up_semaphore );
    wiced_network_deregister_link_callback( link_up, link_down, WICED_STA_INTERFACE );
    wiced_deinit();
}


static void link_up( void )
{
    /* Set a semaphore to indicate the link is back up */
    wiced_rtos_set_semaphore( &link_up_semaphore );
    WPRINT_APP_INFO( ("And we're connected again!\n") );
}


static void link_down( void )
{
    WPRINT_APP_INFO( ("Network connection is down.\n") );
    WPRINT_APP_INFO( ("Switch on your AP and wait until the connection is up.\n") );
}
