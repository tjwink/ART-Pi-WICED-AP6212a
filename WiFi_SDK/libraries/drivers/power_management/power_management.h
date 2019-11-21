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

/** @file BCM943907WAE_1 Power Management API Header
 *
 *  This API utilizes the Maxim17040 and Maxim8971
 *  chips present on the BCM943907WAE_1 board. These
 *  functions should build on other platforms and
 *  just provide "platform unsupported" messages.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef POWER_MANAGEMENT_H
#include "max8971.h"
#include "max17040.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define POWER_MANAGEMENT_H

/* Management function options flags */
#define CHARGER_OPTIONS_ENABLE_PRINT        0x01
#define CHARGER_OPTIONS_ENABLE_DEBUG        0x02

/* Management function return flags */
#define CHARGER_RETURN_IS_DONE              0x01
#define CHARGER_RETURN_FAST_CHARGING        0x02
#define CHARGER_RETURN_TOPPING_OFF          0x04
#define CHARGER_RETURN_CHARGER_ERROR        0x08
#define CHARGER_RETURN_BATTERY_ERROR        0x10
#define CHARGER_RETURN_THERMISTOR_ERROR     0x20
#define CHARGER_RETURN_INPUT_ERROR          0x40
#define CHARGER_RETURN_PLATFORM_UNSUPPORTED 0x80

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    float voltage;
    uint8_t state_of_charge;
    float time_remaining;
    uint8_t return_flags;
} power_management_status_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Initializes and probes maxim8971 and maxim17040 chips.
 *
 * Parameters:
 *     minutes - The number of minutes the specific application takes to
 *               run from a full charge to state of charge = 0%. If runtime
 *               is unknown, this value can be set to 0.
 *
 * Returns:
 *     WICED_SUCCESS     - if successful
 *     WICED_UNSUPPORTED - on any platform except BCM943907WAE_1
 *     WICED_ERROR       - otherwise
 */
wiced_result_t power_management_init(uint32_t minutes);

/* Updates the parameters of the maxim chips and returns battery and charger status.
 *
 * Parameters:
 *     return_status - A pointer to a power management struct. Relevant status and details including
 *                     voltage, SOC, time remaining, and charger status is returned here.
 *     speed_setting - The max possible charging speed (SLOW, MED, FAST). If the input power is
 *                     insufficient for this setting, the battery will charge as fast as the input allows.
 *     options_flag  - Enables printing of status and debug messages.
 *
 * Returns:
 *     WICED_SUCCESS     - if successful
 *     WICED_BADARG      - if return_status parameter is a null pointer
 *     WICED_UNSUPPORTED - on any platform except BCM943907WAE_1
 */
wiced_result_t power_management_update(power_management_status_t* return_status, maxim_charge_speed_t speed_setting, uint8_t options_flag);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
