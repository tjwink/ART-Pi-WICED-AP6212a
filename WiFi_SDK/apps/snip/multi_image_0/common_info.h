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

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/* Define the common DEEP SLEEP Variables in a macro that will be used by both applications */
#define COMMON_ALWAYS_ON_RAM_SHARED_VARIABLES                                                   \
        static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR( warmboot )            = WICED_FALSE;    \
        static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR( connected )           = WICED_FALSE;    \
        static uint32_t     WICED_DEEP_SLEEP_SAVED_VAR( wakeup_count )        = 0;              \
        static wiced_time_t WICED_DEEP_SLEEP_SAVED_VAR( go_to_sleep_time )    = 0;              \
        static char         WICED_DEEP_SLEEP_SAVED_VAR( current_ap[APP_SSID_NAME_LEN + 1] ) =  APP_NO_CONNECTION_STR;

/******************************************************
 *                    Constants
 ******************************************************/

/* This needs to be long, but the switch is quick */
#define WIFI_SLEEP_TIME                      (10000 * MILLISECONDS)

/* This is to test passing a URI between applications using the Always On RAM (AON) and the app DCT */
#define     APP_STR_LEN         256
#define     APP_SSID_NAME_LEN    32

#define APP_NO_CONNECTION_STR  "Not Connected"
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
