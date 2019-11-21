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
 */

#include "wiced.h"
#include "wifi_config_dct.h"

/** @file
 *
 * pno (preferred network offload) application
 *
 * This application snippet demonstrates how to use
 * the WICED preferred network offload APIs.
 *
 * Features demonstrated
 *  - STA association
 *  - disassociation detection
 *  - starting a preferred network background scan
 *  - handling preferred network detection to reassociate
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Plug the WICED eval board into your computer
 *  3. Open a terminal application and connect to the WICED eval board
 *  4. Build and download the application (to the WICED board)
 *  5. After the application has connected to your Wi-Fi access point, you will need to power down the AP,
 *  wait until app prints out message from detecting disconnect, then sometime later power the AP back on.
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Joins a Wi-Fi network
 *  - Waits for a disconnect event to fire
 *  - Starts a pno scan for the AP which was just disconnected and waits for the AP to be found
 *  - Handles the AP being found again by re-connecting to the found AP
 *
 * TROUBLESHOOTING
 *  - Ensure AP information in the wifi_config_dct.h file in this directory is accurate.  E.g. CLIENT_AP_SSID
 *  needs to be defined to the APs SSID
 *
 * NOTES:
 *  - PNO may be used to do background scanning for several different APs at the same time.  This example only
 * has one preferred network.
 */
/******************************************************
 *                      Macros
 ******************************************************/
#define CHECK_RETURN( expr )  { wiced_result_t check_res = (expr); if ( check_res != WICED_SUCCESS ) { WPRINT_APP_INFO( ("Command %s failed \n", #expr) ); } }

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
static void link_up  ( void );
static void link_down( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Main thread.
 *
 * The application code will create a connection and then:
 *
 * - wait for disconnect
 * - print a message upon disconnect
 * - start up a pno scan
 * - print a message when preferred network is detected and reconnect
 * - then return to wait for disconnect state
 */
void application_start( void )
{
    wiced_result_t  ret = WICED_SUCCESS;

    /* Initialise the device */
    wiced_init( );

    /* Register callbacks */
    wiced_network_register_link_callback( link_up, link_down, WICED_STA_INTERFACE );

    /* Bringup the network interface */
    ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    if ( ret == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Join succeeded; unplug your Access Point (AP) to proceed\n"));
    }
    else
    {
        WPRINT_APP_INFO(("Initial join failed; starting PNO process\n"));
        link_down();
    }
}

/******************************************************
 *               Static Function Definitions
 ******************************************************/
static void link_up( void )
{
    /* Set a semaphore to indicate the link is back up */
    WPRINT_APP_INFO( ("Connection occurred; link up\n") );
}

wiced_result_t pno_app_result_callback( wiced_scan_handler_result_t* malloced_scan_result )
{
    wiced_result_t ret;

    /* must be done prior to free below -- this callback function owns the memory */
    malloc_transfer_to_curr_thread( malloced_scan_result );

    WPRINT_APP_INFO( ("Pno network detected.\n") );

    /* when network is detected, force a reassoc via wiced_network_up */
    if ( wiced_network_is_up( WICED_STA_INTERFACE ) == WICED_FALSE )
    {
        ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

        if ( ret == WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("Re-join succeeded; clearing pno.\n") );
        }
        else
        {
            WPRINT_APP_INFO( ("Re-join failed. Pno still running.\n") );
        }
    }

    /* if we're up, then clear out pno resources */
    if ( wiced_network_is_up( WICED_STA_INTERFACE ) == WICED_TRUE )
    {
        /* clear out pno, as it has detected the network registered earlier
                * Note: this also clears event registration
                */
        CHECK_RETURN( wiced_wifi_pno_stop( ) );
    }

    WPRINT_APP_INFO( ("Done handling pno event\n") );

    free( malloced_scan_result );

    return WICED_SUCCESS;
}

static void link_down( void )
{
    wiced_result_t result;
    wiced_ssid_t ssid;
    ssid.length = MIN( strlen( CLIENT_AP_SSID ), sizeof( ssid.value ) );
    memcpy( ssid.value, CLIENT_AP_SSID, ssid.length );

    WPRINT_APP_INFO( ("Network connection is down.\n") );

    /* Leave the network to avoid roam behaviors in this state (if desired).
        * Note: roam may cause faster re-association, but is not considered reliable
        * in cases when an AP is gone for more than 10s.
        * Best practice is to NOT include the leave call and allow either PNO or
        * roaming code to cause an association, giving greatest reliability.
        */
    wwd_wifi_leave( WWD_STA_INTERFACE );

    result = wiced_wifi_pno_start( &ssid, CLIENT_AP_SECURITY, pno_app_result_callback, NULL );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Unable to start pno scan result=%d\n", result) );
    }
    else
    {
        WPRINT_APP_INFO( ("Now WICED is successfully setup to detect your AP.\n") );
        WPRINT_APP_INFO( ("You may power on your AP now.\n") );
    }
}
