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
 *  Interface header for a simple DHCP server
 */

#pragma once

#include "wiced_rtos.h"
#include "wiced_tcpip.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct
{
    wiced_thread_t         thread;
    wiced_udp_socket_t     socket;
    volatile wiced_bool_t  quit;
    wiced_interface_t      interface;
} wiced_dhcp_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/*****************************************************************************/
/**
 *
 *  @defgroup dhcp          DHCP Server
 *  @ingroup  ipcoms
 *
 * Communication functions for DHCP server
 *
 *  @{
 */
/*****************************************************************************/

/**
 *  Start a DHCP server instance.
 *
 * @param[in] server     Structure workspace that will be used for this DHCP server instance - allocated by caller.
 * @param[in] interface  Which network interface the DHCP server should listen on.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_start_dhcp_server( wiced_dhcp_server_t* server, wiced_interface_t interface );


/**
 *  Stop a DHCP server instance.
 *
 * @param[in] server     Structure workspace for the DHCP server instance - as used with @ref wiced_start_dhcp_server
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_stop_dhcp_server ( wiced_dhcp_server_t* server );

/**
 * Fetches the list of IP-addresses of associated clients from cached entries of DHCP server
 * @param[in]  server                 Structure workspace for the DHCP server instance - as used with @ref wiced_start_dhcp_server
 * @param[in]  ip_address_list        IP-address list structure for all associated clients
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_get_clients_ip_address_list_dhcp_server( wiced_dhcp_server_t* server, void* ip_address_list );

#ifdef __cplusplus
} /* extern "C" */
#endif
