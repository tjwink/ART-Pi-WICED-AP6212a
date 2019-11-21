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
 * This application snippet demonstrates how to add and remove
 * custom IEs in AP Beacons, using Wi-Fi softAP interface.
 *
 * Features demonstrated
 *  - Add / Remove Custom IEs in SoftAP Beacons using SW1 button
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the SOFT_AP_SSID/SOFT_AP_PASSPHRASE Wi-Fi credentials
 *     as desired
 *  2. Plug the WICED eval board into your computer
 *  3. Open a terminal application and connect to the WICED eval board
 *  4. Build and download the application (to the WICED board)
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Starts a softAP interface
 *  - Press SW1 button to toggle between add custom IE and remove custom IE
 *    in the Beacon frame.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define EPGRAM_OUI                  "\x00\x90\x4C"
#define EPGRAM_OUI_TYPE             2
#define CUSTOM_IE_DATA              "WICED-CUSTOM-IE"
#define PUSH_BUTTON_POLLING_DELAY   (250 * MILLISECONDS)
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

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

static wiced_custom_ie_info_t ie_info = {.oui = EPGRAM_OUI,
        .subtype = EPGRAM_OUI_TYPE, .data = CUSTOM_IE_DATA,
        .length = sizeof(CUSTOM_IE_DATA) - 1,
        .which_packets = VENDOR_IE_BEACON};


static wiced_bool_t is_custom_ie_on = WICED_FALSE;

/******************************************************
 *               Function Definitions
 ******************************************************/
static void modify_custom_ie(void)
{
    wiced_result_t  res = WICED_SUCCESS;

    if(is_custom_ie_on)
    {
        /* Remove the custom IE */
        WPRINT_APP_INFO(( "Remove custom IE from Beacon\n" ));
        res = wiced_wifi_remove_custom_ie( WICED_AP_INTERFACE, &ie_info );
        if (res != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(( "Error in wiced_wifi_remove_custom_ie(). Error = [%d]\n", res ));
            return;
        }

        /* Turn OFF LED1 to indicate custom IEs are removed in the beacon */
        wiced_gpio_output_low( WICED_LED1 );
    }
    else
    {
        /* Add the custom IE */
        WPRINT_APP_INFO(( "Add custom IE in Beacon\n" ));
        res = wiced_wifi_add_custom_ie( WICED_AP_INTERFACE, &ie_info );
        if (res != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(( "Error in wiced_wifi_add_custom_ie(). Error = [%d]\n", res ));
            return;
        }

        /* Turn ON LED1 to indicate custom IEs are added in the beacon */
        wiced_gpio_output_high( WICED_LED1 );
    }

    /* Toggle the flag */
    is_custom_ie_on = !is_custom_ie_on;
    return;
}


void application_start(void)
{
    wiced_bool_t    is_button_pressed = WICED_FALSE;

    /* Initialise WICED stack */
    wiced_init();

    WPRINT_APP_INFO(( "Press SW1 button to toggle between Add custom IE / Remove custom IE to the Beacon\n" ));
    is_custom_ie_on = WICED_FALSE;

    /* Bring up the softAP interface ------------------------------------------------------------- */
    wiced_network_up(WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &ap_ip_settings);

    /* Check if user has pressed SW1 and take action accordingly */
    while(1)
    {
        /* Read the state of Button 1 */
        is_button_pressed = wiced_gpio_input_get( WICED_BUTTON1 ) ? WICED_FALSE : WICED_TRUE;  /* The button has inverse logic */
        if ( is_button_pressed == WICED_TRUE )
        {
            /* Toggle between add / remove custom IEs */
            modify_custom_ie();
        }
        wiced_rtos_delay_milliseconds( PUSH_BUTTON_POLLING_DELAY );
    }
}

