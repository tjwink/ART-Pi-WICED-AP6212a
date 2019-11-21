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

/** @file Maxim8971 Library Header
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX8971_H
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX8971_H

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* Input current must be greater than
 * or equal to charge speed setting */
typedef enum
{
    MAXIM_CHARGE_SPEED_SLOW,   /* 0.5 Amp */
    MAXIM_CHARGE_SPEED_MEDIUM, /* 1.0 Amp */
    MAXIM_CHARGE_SPEED_FAST    /* 1.5 Amp */
} maxim_charge_speed_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Initializes and probes maxim8971 chip.
 *
 * Returns:
 *     WICED_SUCCESS - if successful.
 *     WICED_ERROR   - otherwise.
 */
wiced_result_t max8971_initialize();

/* Individual functions for setting specific charger parameters. */
void max8971_set_charge_speed(maxim_charge_speed_t speed);
void max8971_set_topoff_timer();

/* Individual functions for retrieving specific charger information.
 * Pass return byte to print functions for visible output.
 *
 * Returns:
 *     Single byte from relevant register containing settings flags.
 *
 * Note: Interrupt requests are cleared upon reading.
 */
uint8_t max8971_get_charger_status();
uint8_t max8971_get_thermistor_details();
uint8_t max8971_get_charger_and_battery_details();
uint8_t max8971_get_interrupt_requests();

/* Individual functions for printing specific charger information.
 *
 * Parameters:
 *     Single byte from related maxim register (use maxim8971_get_*() functions).
 */
void max8971_print_charger_status(uint8_t status);
void max8971_print_thermistor_details(uint8_t details);
void max8971_print_charger_and_battery_details(uint8_t details);

/* Prints all maxim8971 registers.
 *
 * Values in register 0x0F (Interrupt requests register) are cleared on read.
 */
void max8971_print_all_registers_debug();

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
