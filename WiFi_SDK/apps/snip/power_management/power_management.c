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
 *
 * BCM943907WAE_1 Power Management Application
 *
 * This application snippet demonstrates the power management
 * api for the BCM943907WAE_1 board. This app initializes the
 * Maxim17040 and Maxim8971 on-board chips and monitors the
 * charger and battery status. This app can be used to generate
 * comma-delimited data which can be exported to excel and used
 * to generate power curves. Power curve data can provide useful
 * information on the discharge characteristics of various
 * batteries as well as the power consumption and runtime of
 * various applications.
 *
 * Features demonstrated
 *  - Power Management API
 *    - Maxim17040 chip functionality
 *    - Maxim8971  chip functionality
 *
 * Application Instructions
 *   Setup:
 *     1. This application requires console - Connect a debug board and usb cable.
 *     2. Connect a battery to the battery port on a BCM943907WAE_1 board.
 *     3. Connect a usb cable to the usb port.
 *     4. Set the max runtime of the application (if known) in the init function.
 *   Charge the Battery:
 *     1. If necessary, wait until the battery is fully charged.
 *        You can monitor the charging status by observing the
 *        state of charge (SOC), checking the update function's
 *        return flag field, or enabling printing and watching
 *        for the done charging message.
 *   Collect Data:
 *     1. Unplug the battery and power usb.
 *     2. Open a console using PuTTY and enable:
 *        Change Settings > Session > Logging > All session output
 *     3. Plug in the fully charged battery.
 *        The PuTTY terminal should show comma-delimited data.
 *        Data should be collected approximately once a minute.
 *   Export Data:
 *     1. Wait until the battery dies and the board powers off.
 *     2. Open the log file and copy/paste all data into excel.
 *     3. Highlight the entire column and follow the instructions in:
 *        Data > Text to Columns
 *        to break the data into individual columns.
 *     4. The data should be of the form:
 *        <minutes elapsed> <volts> <SOC> <minutes remaining>
 *     5. You can graph Voltage vs. Time to generate a power curve.
 *
 * NOTE:
 *  - This application ONLY works on the BCM943907WAE_1 board. It will
 *    build on other boards, but they lack the Maxim hardware and
 *    are NOT supported by this power management api.
 */

#include "power_management.h"
#include <stdio.h>

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
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    unsigned int minutes_elapsed = 0;
    wiced_result_t result;
    power_management_status_t status;

    /* Set the number of minutes it takes to run from 100% to 0%
     * for a more accurate remaining runtime calculation. */
    result = power_management_init(390);
    if (result == WICED_ERROR)
    {
        printf("Power management initialization failure\n");
    }
    printf("\n");

    while (1)
    {
        /* FAST fast charging requires 1.5+ Amp input current.
         * Lower current input should use MEDIUM (1.0) or SLOW (0.5). */
        result = power_management_update(&status, MAXIM_CHARGE_SPEED_FAST, 0x00);

        if (result == WICED_SUCCESS)
        {
            printf("%u,%.2f,%d,%.0f\n", minutes_elapsed, status.voltage, status.state_of_charge, status.time_remaining);
        }
        else if (result == WICED_UNSUPPORTED)
        {
            printf("Power management is only supported on the BCM943907WAE_1 platform\n");
        }

        wiced_rtos_delay_milliseconds(60000);
        minutes_elapsed++;
    }
}
