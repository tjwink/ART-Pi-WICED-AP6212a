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

/**
 * @file Public API for DNS protocol
 */

#pragma once

#include "wiced_tcpip.h"
#include <stdint.h>

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

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* DNS Client API */

/** Adds the IP address of a DNS server to the list used by the DNS client
 *
 * @param[in] address : The IP address of the DNS server
 *
 * @return @ref wiced_result_t
 */
wiced_result_t dns_client_add_server_address          ( wiced_ip_address_t address );


/** Wipes out the list of DNS servers used by the DNS client
 *
 * @return @ref wiced_result_t
 */
wiced_result_t dns_client_remove_all_server_addresses ( void );


/** Lookup a hostname (domain name) using the DNS client
 *
 * @param[in]  hostname   : The hostname string to look-up
 * @param[out] list       : Receives the IP addresses matching the hostname
 * @param[in]  type       : Specifies the type of IP addresses which should be returned in the list
 * @param[in]  timeout_ms : The timeout period in milliseconds
 * @param[in]  interface  : Network interface to use for look-up
 *
 * @note : The returned list will contain addresses as specified by look-up 'type'. See @ref wiced_dns_lookup_address_type_t .
 * @return @ref wiced_result_t
 */
wiced_result_t dns_client_hostname_lookup ( const char* hostname, wiced_resolved_ip_address_list* list, wiced_dns_lookup_address_type_t type, uint32_t timeout_ms, wiced_interface_t interface );

#ifdef __cplusplus
} /* extern "C" */
#endif
