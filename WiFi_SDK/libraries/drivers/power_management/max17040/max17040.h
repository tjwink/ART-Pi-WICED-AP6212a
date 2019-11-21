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

/** @file Maxim17040 Library Header
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX17040_H
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX17040_H

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
 *               Function Declarations
 ******************************************************/

/* Initializes and probes the maxim17040 chip.
 *
 * Returns:
 *     WICED_SUCCESS - if successful.
 *     WICED_ERROR   - otherwise.
 */
wiced_result_t max17040_initialize_i2c_device();

/* Returns the current battery voltage in volts. */
float max17040_get_vcell_voltage();

/* Returns the calculated State Of Charge in percent.
 *
 * Note: The SOC calculation becomes more accurate
 *       with prolonged charging and discharging.
 */
uint8_t max17040_get_soc_percent();

/* Sets the runtime parameters for the time remaning calculation.
 *
 * Parameters:
 *     minutes - The number of minutes the specific application
 *               runs from a full charge to SOC = 0%.
 */
void max17040_set_max_runtime(uint32_t minutes);

/* Returns an estimate of remaining runtime duration in minutes.
 *
 * Note: The max runtime for the specific application must be
 *       set for this function to work. This estimate is less
 *       accurate as SOC approaches 0%.
 */
float max17040_get_time_remaining();

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
