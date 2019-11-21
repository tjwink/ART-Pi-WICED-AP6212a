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

#pragma once

#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "wiced.h"
#include "platform_dct.h"

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
/**
 * Disconnect from current network
 *  Does not change any firmware settings
 *
 * @param[in] interface     - wiced_interface_t type (WICED_STA_INTERFACE or WICED_ETHERNET_INTERFACE)
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 */
wiced_result_t wiced_ota2_network_down( wiced_interface_t interface  );

/**
 * Connect to a specific AP
 *  Set APSTA, AP, MPC, rmc_ackreq to 0x00
 *  Set channel
 *  Connect to designated AP
 *  get IP address using DHCP Client
 *
 * @param[in] interface     - wiced_interface_t type (WICED_STA_INTERFACE or WICED_ETHERNET_INTERFACE)
 * @param[in]  ap_info      - structure defining AP to connect to
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t wiced_ota2_network_up( wiced_interface_t interface , wiced_config_ap_entry_t* ap_info );


#ifdef __cplusplus
} /*extern "C" */
#endif

