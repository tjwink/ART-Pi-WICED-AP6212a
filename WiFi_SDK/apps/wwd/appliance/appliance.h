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
 * @file
 * Main header file for the Appliance App, containing structures used by it etc.
 */

#ifndef INCLUDED_SENSOR_H
#define INCLUDED_SENSOR_H

#include "wwd_wifi.h"
#include "web_server.h"

#ifdef APPLIANCE_ENABLE_WPS
#include "wps_host.h"
#endif /* ifdef APPLIANCE_ENABLE_WPS */

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
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
    enum
    {
        CONFIG_NONE = 0,
        CONFIG_WPS_PBC,
        CONFIG_WPS_PIN,
        CONFIG_SCANJOIN,
    } config_type;
    union
    {
        struct
        {
            char                pin[9]; /* extra byte for terminating null */
        } wps_pin;
        struct
        {
            wiced_scan_result_t scanresult;
            char                passphrase[WSEC_MAX_PSK_LEN];
            unsigned char       passphrase_len;
        } scanjoin;
    } vals;
} appliance_config_t;

/******************************************************
 *                 Global Variables
 ******************************************************/
extern appliance_config_t       appliance_config;
extern const url_list_elem_t    config_STA_url_list[];
extern const url_list_elem_t    config_AP_url_list[];

/******************************************************
 *               Function Declarations
 ******************************************************/
void start_dns_server( uint32_t local_addr );
void quit_dns_server( void );
void start_dhcp_server( uint32_t local_addr );
void quit_dhcp_server( void );

#ifdef APPLIANCE_ENABLE_WPS
void do_wps( wiced_wps_mode_t wps_mode, char* pin );
#endif /* ifdef APPLIANCE_ENABLE_WPS */


#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef INCLUDED_SENSOR_H */
