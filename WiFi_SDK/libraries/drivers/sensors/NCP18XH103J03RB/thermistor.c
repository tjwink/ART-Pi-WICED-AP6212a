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
 */

#include "thermistor.h"
#include "wiced_platform.h"
#include "math.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t thermistor_take_sample(wiced_adc_t adc, int16_t* celcius_degree_tenths )
{
    uint16_t sample_value;
    wiced_result_t result = wiced_adc_take_sample(adc, &sample_value);

    /* Thermistor is Murata NCP18XH103J03RB  (Digi-key 490-2436-1-ND )
     *
     * Part Number details:
     * NC  : NTC Chip Thermistor
     * P   : Plated termination
     * 18  : Size 0603
     * XH  : Temperature Characteristics : Nominal B-Constant 3350-3399K
     * 103 : Resistance 10k
     * J   : Tolerance   +/- 5%
     * 03  : Individual Specs: Standard
     * RB  : Paper Tape 4mm pitch, 4000pcs
     *
     *
     * It has a 43K feed resistor from 3V3
     *
     * Thermistor Voltage    = V_supply * ADC_value / 4096
     * Thermistor Resistance = R_feed / ( ( V_supply / V_thermistor ) - 1 )
     * Temp in kelvin = 1 / ( ( ln( R_thermistor / R_0 ) / B ) + 1 / T_0 )
     * Where: R_feed = 43k, V_supply = 3V3, R_0 = 10k, B = 3375, T_0 = 298.15°K (25°C)
     */
    if (result == WICED_SUCCESS)
    {
        double thermistor_resistance = 43000.0 / ( ( 4096.0 / (double) sample_value ) - 1 );
        double logval = log( thermistor_resistance / 10000.0 );
        double temperature = 1.0 / ( logval / 3380.0 + 1.0 / 298.15 ) - 273.15;

        *celcius_degree_tenths = (int16_t)(temperature*10);
        return WICED_SUCCESS;
    }
    else
    {
        *celcius_degree_tenths = 0;
        return result;
    }
}
