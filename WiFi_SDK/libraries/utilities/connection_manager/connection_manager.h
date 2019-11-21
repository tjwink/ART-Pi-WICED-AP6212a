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

/** @file: connection_manager.h
 *  Defines functions to access WiFi connection manager.
 *  Following features are supported:
 *   Wi-Fi Direct Group Owner: Persistent group
 *   Wi-Fi Direct Client mode
 *   Wi-Fi WPS Registrar mode
 *   Wi-Fi WPS Enrollee mode
 */

#pragma once

#include "wiced_p2p.h"
#include "wiced_wps.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *            Enumerations
 ******************************************************/

typedef enum
{
    CONNECTION_IDLE                  = 0x0,
    CONNECTION_P2P_GO                = 0x1 << 0,
    CONNECTION_P2P_GC                = 0x1 << 1,
    CONNECTION_P2P_GC_REINVOKE       = 0x1 << 2,
    CONNECTION_P2P_GO_NEGOTIATION    = 0x1 << 3,
    CONNECTION_WPS_REGISTRAR         = 0x1 << 4,
    CONNECTION_WPS_ENROLLEE          = 0x1 << 5,
    CONNECTION_WPS_ENROLLEE_REINVOKE = 0x1 << 6,
} connection_status_t;

typedef enum
{
    CONNECTION_P2P_CONNECTED        = 0x0,
    CONNECTION_P2P_DISCONNECTED     = 0x1 << 0,
    CONNECTION_P2P_FAILED           = 0x1 << 1,
} connection_p2p_result_t;


/******************************************************
 *             Structures
 ******************************************************/

typedef struct
{
    besl_p2p_device_detail_t p2p_details;
    wiced_wps_device_detail_t registrar_details;
    wiced_wps_device_detail_t enrollee_details;
    wiced_ip_setting_t softap_ip_settings;
} connection_manager_context_t;


/******************************************************
 *               Function Declarations
 * ******************************************************/

/*****************************************************************************/
/**
 *  @addtogroup connection_manager     WiFi (802.11) P2P connection functions
 *  @ingroup    wifi
 *
 *  WiFi connection functions specific to P2P, and supporting WPS also
 *  The Connection Manager simplifies setting up Wi-Fi connections using
 *  either static configuration, Wireless Protected Setup (WPS) or Wi-Fi
 *  Direct connections. Using the Connection manager API's simplifies
 *  Wi-Fi connectivity.
 *
 *  @{
 */
/*****************************************************************************/
/** Requests a function be called by P2P connection event
 *
 * This function registers a function that will be called by P2P connection event.
 * Since the P2P process will be done in the other thread, need this to get an asynchronous P2P event.
 *
 * @param[in] p2p_result_callback : The callback function that is to be called by P2P connection event.
 */
void connection_register_p2p_result_callback        ( void ( *p2p_result_callback)(connection_p2p_result_t) );


/** Launch the connections with a specified connection bitmap.
 *
 * This functions launches the connections with a specified connection bitmap.
 * Each bit means:
 *                 WiFi Direct Group Owner  = 0x1 << 0
 *                 WiFi Direct Group Client = 0x1 << 1
 *                 WPS Registrar            = 0x1 << 3
 *                 WPS Enrollee             = 0x1 << 4
 *
 * @param[in] connections   : a connection bitmap to be launched
 *
 * @return    WICED_SUCCESS : on success.
 * @return    WICED_ERROR   : if an error occurred
 */
wiced_result_t connection_launch                    ( connection_status_t connections );


/** Kill the connections with a specified connection bitmap.
 *
 * This function kills the connections with a specified connection bitmap.
 * Each bit means:
 *                 WiFi Direct Group Owner  = 0x1 << 0
 *                 WiFi Direct Group Client = 0x1 << 1
 *                 WPS Registrar            = 0x1 << 3
 *                 WPS Enrollee             = 0x1 << 4
 *
 * @param[in] connections   : a connection bitmap to be killed
 *
 * @return    WICED_SUCCESS : on success.
 * @return    WICED_ERROR   : if an error occurred
 */
wiced_result_t connection_kill                      ( connection_status_t connections );


/** Kill all of connections.
 *
 * This function kills all of connections which are established.
 *
 * @return    WICED_SUCCESS : on success.
 * @return    WICED_ERROR   : if an error occurred.
 */
wiced_result_t connection_killall                   ( void );


/** Returns a connection bitmap which are established.
 *
 * This function returns a connection bitmap which are established.
 * Each bit means:
 *                 IDLE; No connections     = 0x0
 *                 WiFi Direct Group Owner  = 0x1 << 0
 *                 WiFi Direct Group Client = 0x1 << 1
 *                 WPS Registrar            = 0x1 << 3
 *                 WPS Enrollee             = 0x1 << 4
 *
 * @return    connection_status_t : established connections.
 */
connection_status_t connection_get_status           ( void );


/** Returns a copy of current settings
 *
 * This function copies current settings into the user variable.
 *
 * @param[out] cm_context    : a copy of current settings
 */
void connection_get_settings                        ( connection_manager_context_t* cm_context );


/** Override current settings with user settings
 *
 * This function overrides the current setttings with the input argument.
 *
 * @param[in] cm_context     : user specific settings
 */
void connection_set_settings                        ( connection_manager_context_t* cm_context );

/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif
