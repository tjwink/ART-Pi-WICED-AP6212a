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

/** @file Maxim17040 Library Functions
 *
 */

#include "max17040.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RETRIES                 5
#define VOLTS_PER_VCELL_BIT     0.00125
#define VCELL_REGISTER_ADDRESS  0x02
#define SOC_REGISTER_ADDRESS    0x04
#define UPPER_VOLTAGE_LIMIT     4.20
#define LOWER_VOLTAGE_LIMIT     3.40

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint32_t max_runtime = 0;

static wiced_i2c_device_t maxim17040 =
{
    .port          = WICED_I2C_1,
    .address       = 0x36,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .flags         = 0,
    .speed_mode    = I2C_HIGH_SPEED_MODE,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t max17040_initialize_i2c_device( void )
{
    if ( ( wiced_i2c_init( &maxim17040 ) != WICED_SUCCESS ) ||
         ( wiced_i2c_probe_device(&maxim17040, RETRIES) != WICED_TRUE ) )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Sequence to read data from specified registers on maxim chips.
 *
 * This consists of [Start] [Write (slave address) (register address)] [Repeat-Start] [Read] [Stop].
 */
static wiced_result_t maxim_i2c_reg_read(uint8_t* write_buffer, uint8_t write_buffer_size, uint8_t* read_buffer, uint8_t read_buffer_size )
{
    /* Write slave address of maxim and register address to read from */
    if (wiced_i2c_write(&maxim17040, WICED_I2C_START_FLAG, write_buffer, write_buffer_size) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Read bytes from maxim register address into read_buffer */
    if (wiced_i2c_read(&maxim17040, WICED_I2C_REPEATED_START_FLAG | WICED_I2C_STOP_FLAG, read_buffer, read_buffer_size) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Maxim17040 reports voltage in two bytes.
 * All 8 bits of the first byte are used, but only the upper 4 bits
 * of the second byte are used for a total of 12 bits.
 */
static float calculate_voltage( uint8_t register_value[2] )
{
    uint32_t voltage = 0;

    voltage |= register_value[0];
    voltage <<= 4;
    voltage |= register_value[1] >> 4;

    return (float) voltage * VOLTS_PER_VCELL_BIT;
}

float max17040_get_vcell_voltage( void )
{
    uint8_t vcell_reg = VCELL_REGISTER_ADDRESS;
    uint8_t bytes_read[2] = {0, 0};

    maxim_i2c_reg_read(&vcell_reg, 1, bytes_read, 2);

    return calculate_voltage(bytes_read);
}

uint8_t max17040_get_soc_percent( void )
{
    uint8_t soc_reg = SOC_REGISTER_ADDRESS;
    uint8_t bytes_read[2] = {0, 0};

    maxim_i2c_reg_read(&soc_reg, 1, bytes_read, 2);

    return bytes_read[0];
}

void max17040_set_max_runtime( uint32_t minutes )
{
    max_runtime = minutes;
}

float max17040_get_time_remaining( void )
{
    return ( max17040_get_vcell_voltage() - LOWER_VOLTAGE_LIMIT) * max_runtime / (UPPER_VOLTAGE_LIMIT - LOWER_VOLTAGE_LIMIT );
}
