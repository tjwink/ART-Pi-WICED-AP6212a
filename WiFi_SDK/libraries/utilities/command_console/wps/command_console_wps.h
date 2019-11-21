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
#pragma once

#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

#define WPS_COMMANDS \
    { (char*) "force_alignment",    force_alignment,   0, NULL, NULL, (char*) "",                                           (char*) "Force aligned memory accesses"}, \
    { (char*) "join_wps",           join_wps,          1, NULL, NULL, (char*) "<pbc|pin> [pin] [<ip> <netmask> <gateway>]", (char*) "Join an AP using WPS"}, \
    { (char*) "join_wps_specific",  join_wps_specific, 0, NULL, NULL, (char*) "[pin] [<ip> <netmask> <gateway>]",           (char*) "Join a specific WLAN using WPS PIN mode"}, \
    { (char*) "scan_wps",           scan_wps,          0, NULL, NULL, (char*) "",                                           (char*) "Scan for APs supporting WPS"}, \
    { (char*) "start_registrar",    start_registrar,   1, NULL, NULL, (char*) "<pbc|pin> [pin]",                            (char*) "Start the WPS Registrar"}, \
    { (char*) "stop_registrar",     stop_registrar,    0, NULL, NULL, (char*) "",                                           (char*) "Stop the WPS Registrar"}, \

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

int join_wps( int argc, char* argv[] );
int start_registrar( int argc, char* argv[] );
int force_alignment( int argc, char* argv[] );
int stop_registrar( int argc, char* argv[] );
wiced_result_t enable_ap_registrar_events( void );
void disable_ap_registrar_events( void );
int scan_wps( int argc, char* argv[] );
int join_wps_specific( int argc, char* argv[] );

/* Function used by other modules */
void dehyphenate_pin(char* str );

#ifdef __cplusplus
} /*extern "C" */
#endif
