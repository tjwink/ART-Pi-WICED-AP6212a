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
 * STM32H7xx I2C implementation
 */
#include "platform_peripheral.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* I2C bus frequency in Hz based on speed mode */
#define I2C_LOW_SPEED_MODE_FREQ_HZ      (10000)
#define I2C_STANDARD_SPEED_MODE_FREQ_HZ (100000)
#define I2C_HIGH_SPEED_MODE_FREQ_HZ     (400000)

/*
 * The I2C TIMING register configuration based on the bus speed.
 * The register value is obtained using the STM32CubeMX tool as below:
 * 1. I2C clock source is set to HSI.
 * 2. HSI frequency is 64MHz (from HSI_VALUE in stm32h7xx_hal_conf.h).
 * 3. Rise time is set to 100ns. Fall time is set to 100ns.
 * 4. The desired speed mode (Standard Mode, Fast Mode).
 * 5. The desired speed frequency (10KHz, 100KHz, 400KHz).
 */
#define I2C_LOW_SPEED_MODE_TIMING      (0xE011A8FF)
#define I2C_STANDARD_SPEED_MODE_TIMING (0x10B17DB5)
#define I2C_HIGH_SPEED_MODE_TIMING     (0x00C12166)

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

typedef enum
{
    PLATFORM_I2C1, PLATFORM_I2C2, PLATFORM_I2C3, PLATFORM_I2C4, PLATFORM_I2C_MAX,
} platform_i2c_type_t;

void hal_rcc_i2c1_clock_enable( void )
{
    RCC_PeriphCLKInitTypeDef rcc_i2c1_clock;

    /* Configure the I2C clock source */
    rcc_i2c1_clock.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    rcc_i2c1_clock.I2c123ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&rcc_i2c1_clock);

    /* Enable the I2C clock */
    __HAL_RCC_I2C1_CLK_ENABLE();
}

void hal_rcc_i2c2_clock_enable( void )
{
    RCC_PeriphCLKInitTypeDef rcc_i2c2_clock;

    /* Configure the I2C clock source */
    rcc_i2c2_clock.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    rcc_i2c2_clock.I2c123ClockSelection = RCC_I2C2CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&rcc_i2c2_clock);

    /* Enable the I2C clock */
    __HAL_RCC_I2C2_CLK_ENABLE();
}

void hal_rcc_i2c3_clock_enable( void )
{
    RCC_PeriphCLKInitTypeDef rcc_i2c3_clock;

    /* Configure the I2C clock source */
    rcc_i2c3_clock.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    rcc_i2c3_clock.I2c123ClockSelection = RCC_I2C3CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&rcc_i2c3_clock);

    /* Enable the I2C clock */
    __HAL_RCC_I2C3_CLK_ENABLE();
}

void hal_rcc_i2c4_clock_enable( void )
{
    RCC_PeriphCLKInitTypeDef rcc_i2c4_clock;

    /* Configure the I2C clock source */
    rcc_i2c4_clock.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
    rcc_i2c4_clock.I2c4ClockSelection = RCC_I2C4CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&rcc_i2c4_clock);

    /* Enable the I2C clock */
    __HAL_RCC_I2C4_CLK_ENABLE();
}

void hal_rcc_i2c1_clock_disable( void )
{
    /* Disable the I2C clock */
    __HAL_RCC_I2C1_CLK_DISABLE();
}

void hal_rcc_i2c2_clock_disable( void )
{
    /* Disable the I2C clock */
    __HAL_RCC_I2C2_CLK_DISABLE();
}

void hal_rcc_i2c3_clock_disable( void )
{
    /* Disable the I2C clock */
    __HAL_RCC_I2C3_CLK_DISABLE();
}

void hal_rcc_i2c4_clock_disable( void )
{
    /* Disable the I2C clock */
    __HAL_RCC_I2C4_CLK_DISABLE();
}

const platform_i2c_clock_enable_function_t i2c_clock_enable_function[ NUMBER_OF_I2C_PORTS ] =
{
    [PLATFORM_I2C1] = hal_rcc_i2c1_clock_enable,
    [PLATFORM_I2C2] = hal_rcc_i2c2_clock_enable,
    [PLATFORM_I2C3] = hal_rcc_i2c3_clock_enable,
    [PLATFORM_I2C4] = hal_rcc_i2c4_clock_enable,
};

const platform_i2c_clock_disable_function_t i2c_clock_disable_function[ NUMBER_OF_I2C_PORTS ] =
{
    [PLATFORM_I2C1] = hal_rcc_i2c1_clock_disable,
    [PLATFORM_I2C2] = hal_rcc_i2c2_clock_disable,
    [PLATFORM_I2C3] = hal_rcc_i2c3_clock_disable,
    [PLATFORM_I2C4] = hal_rcc_i2c4_clock_disable,
};

const uint8_t i2c_alternate_functions[ NUMBER_OF_I2C_PORTS ] =
{
    [PLATFORM_I2C1] = GPIO_AF4_I2C1,
    [PLATFORM_I2C2] = GPIO_AF4_I2C2,
    [PLATFORM_I2C3] = GPIO_AF4_I2C3,
    [PLATFORM_I2C4] = GPIO_AF4_I2C4,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

uint8_t platform_i2c_get_port_number( platform_i2c_port_t* i2c_port )
{
    if ( i2c_port == I2C1 )
    {
        return PLATFORM_I2C1;
    }
    else if ( i2c_port == I2C2 )
    {
        return PLATFORM_I2C2;
    }
    else if ( i2c_port == I2C3 )
    {
        return PLATFORM_I2C3;
    }
    else if ( i2c_port == I2C4 )
    {
        return PLATFORM_I2C4;
    }
    else
    {
        return INVALID_I2C_PORT_NUMBER;
    }
}

uint32_t platform_i2c_get_port_timing( platform_i2c_port_t* i2c_port, uint32_t bus_freq )
{
    uint32_t timing;

    if ( bus_freq < I2C_STANDARD_SPEED_MODE_FREQ_HZ )
    {
        /* I2C bus in low speed mode */
        timing = I2C_LOW_SPEED_MODE_TIMING;
    }
    else if ( bus_freq < I2C_HIGH_SPEED_MODE_FREQ_HZ )
    {
        /* I2C bus in standard speed mode */
        timing = I2C_STANDARD_SPEED_MODE_TIMING;
    }
    else
    {
        /* I2C bus in high speed mode */
        timing = I2C_HIGH_SPEED_MODE_TIMING;
    }

    return timing;
}
