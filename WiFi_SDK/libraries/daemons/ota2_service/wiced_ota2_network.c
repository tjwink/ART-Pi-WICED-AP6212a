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
 * WICED Over The Air 2 Background Network interface (OTA2)
 *
 *        ***  PRELIMINARY - SUBJECT TO CHANGE  ***
 *
 *  This API allows for disconnecting from current network
 *      and connecting to an alternate network for accessing
 *      the internet and downloading an OTA2 Image File
 */
#include <ctype.h>
#include "wiced.h"
#include "internal/wwd_sdpcm.h"
#include "../../WICED/internal/wiced_internal_api.h"
#include "wwd_buffer_interface.h"

#include "wiced_ota2_service.h"
#include "wiced_ota2_network.h"

#include "wiced_log.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define CHECK_RETURN( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_AP_CONNECT_RETRIES      3

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                  Enumerations
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/


/******************************************************
 *               Variables Definitions
 ******************************************************/

/****************************************************************
 *  Internal functions
 ****************************************************************/

/****************************************************************
 *  External functions
 ****************************************************************/
wiced_result_t wiced_ota2_network_down( wiced_interface_t interface  )
{
    wiced_result_t result;

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_network_down: wiced_ip_down() interface: %d \r\n", interface);
    result = wiced_ip_down( interface );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_down: wiced_ip_down() failed! %d \r\n", result);
    }

    /* leave the AP */
    result = wiced_leave_ap(interface);
    if (interface == WICED_ETHERNET_INTERFACE )
    {
        return wiced_network_down( interface );
    }

    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_down: wiced_leave_ap() failed! %d \r\n", result);
    }

    return result;
}


wiced_result_t wiced_ota2_network_up( wiced_interface_t interface , wiced_config_ap_entry_t* ap_info )

{
    wiced_result_t  result = WICED_SUCCESS;

    /* Disable power save  */

    wiced_wifi_disable_powersave( );

    /*
      * Bring up the network.
      */
    if (wiced_network_is_up(interface) == WICED_FALSE)
    {
        wwd_result_t    retval;

        if (interface == WICED_ETHERNET_INTERFACE)
        {
            /* NOTE: wiced_network_up() calls wiced_ip_up() and ethernet_init() and ethernet_start() */
            return wiced_network_up( interface, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        }

        retval = wiced_wifi_up();
        if (retval != WWD_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_up() wiced_wifi_up() failed: %d\r\n", retval);
            return WICED_ERROR;
        }
    }


    if (interface == WICED_STA_INTERFACE )
    {
        /* sanity check */
        if (ap_info == NULL)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_up() Bad ARGS! ap_info %p \r\n", ap_info);
            return WICED_BADARG;
        }

        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "wiced_ota2_network_up: STA start %.*s\r\n", ap_info->details.SSID.length, ap_info->details.SSID.value);

        /* connect to OTA2 AP */
        int tries = 0;
        do
        {
            result = wiced_join_ap_specific( &ap_info->details, ap_info->security_key_length, ap_info->security_key );
            if (result != WICED_SUCCESS)
            {
                if (result == (wiced_result_t)WWD_NETWORK_NOT_FOUND)
                {
                    tries = MAX_AP_CONNECT_RETRIES; /* so we do not retry */
                    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_network_up: wiced_join_ap_specific() failed (NOT Found)! - we will not retry %d\r\n", result);
                }
                else
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "wiced_ota2_network_up: wiced_join_ap_specific() failed! result:%d try:%d\r\n", result, tries);
                }
            }
            else
            {
                break;
            }

            /* wait a bit and try again */
            wiced_rtos_delay_milliseconds(500);

        } while ((result != WICED_SUCCESS) && (tries++ < MAX_AP_CONNECT_RETRIES));
    }

    if (result == WICED_SUCCESS)
    {
        wiced_ip_address_t ip_addr;

        /* get our IP address */
        result = wiced_ip_up( interface, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_up: wiced_ip_up() failed: result %d \r\n", result);
        }
        else
        {
            wiced_ip_get_ipv4_address(interface, &ip_addr);
            wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "         IP addr: " IPV4_PRINT_FORMAT "\r\n", IPV4_SPLIT_TO_PRINT(ip_addr));
            if ((ip_addr.ip.v4 != 0x0000) && (ip_addr.ip.v4 != 0x0001))
            {
                result = WICED_SUCCESS;
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_up: wiced_ip_get_ipv4_address() failed. \r\n");
            }
        }
    }

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_network_up() failed.\r\n");
    }

    return result;
}
