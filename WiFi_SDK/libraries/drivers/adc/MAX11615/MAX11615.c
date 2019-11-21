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
 *  MAX11615 - 6 Channel Single-Ended 12bit ADC Implementation
 */

#include "platform_peripheral.h"
#include "wwd_assert.h"
#include "wiced_platform.h"
#include "MAX11615.h"

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

static wiced_i2c_device_t MAX11615_I2C =
{
        .port = WICED_I2C_1,
        .address = MAX11615_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/* Using ADC will initialize I2C bus at 100kHz, change the speed in the MAX11615_I2C structure if needed */
/* https://datasheets.maximintegrated.com/en/ds/MAX11612-MAX11617.pdf */

platform_result_t max11615_init( const platform_adc_t* adc, uint32_t sample_cycle )
{
    UNUSED_PARAMETER( sample_cycle );
    uint8_t setup = MAX11615_SETUP_BYTE;

    /* Initialize I2C */
    WPRINT_APP_DEBUG( ( "I2C Initialization\n" ) );
    if ( wiced_i2c_init( &MAX11615_I2C ) != WICED_SUCCESS )
    {
        WPRINT_APP_ERROR( ( "I2C Initialization Failed\n" ) );
        return PLATFORM_ERROR;
    }

    /* Probe I2C bus for ADC */
    WPRINT_APP_DEBUG( ( "I2C Device Probe\n" ) );
    if( wiced_i2c_probe_device( &MAX11615_I2C, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        WPRINT_APP_ERROR( ( "Failed to detect ADC device; addr:0x%02x\n", MAX11615_I2C.address ) );
        return PLATFORM_ERROR;
    }
    WPRINT_APP_DEBUG( ( "I2C Device detected at address: 0x%02x\n", MAX11615_I2C.address ) );

    /* Write ADC Setup Byte */
    WPRINT_APP_DEBUG( ( "Sending ADC setup byte\r\n" ) );
    if ( wiced_i2c_write( &MAX11615_I2C, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &setup, 1 ) != WICED_SUCCESS )
    {
        WPRINT_APP_ERROR( ( "Failed to setup ADC\r\n" ) );
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;

}

platform_result_t max11615_deinit( const platform_adc_t* adc )
{
    UNUSED_PARAMETER( adc );
    return PLATFORM_UNSUPPORTED;
}

platform_result_t max11615_take_sample( const platform_adc_t* adc, uint16_t* output )
{
    uint8_t cmd = 0b01100001; /* [0 11 XXXX 1] [configbyte readonechannel channelselect single-ended] */
    cmd |= ( adc->channel << 1);
    WPRINT_APP_DEBUG( ("Command Byte: 0x%x\n", cmd) );
    uint8_t data[2];
    wiced_result_t result;

    /* Write ADC Configuration Byte */
    result = wiced_i2c_write( &MAX11615_I2C, WICED_I2C_START_FLAG, &cmd, 1 );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_ERROR( ( "Failed to send ADC read request.\r\n" ) );
        return PLATFORM_ERROR;
    }

    /* Read ADC Response Bytes */
    result = wiced_i2c_read( &MAX11615_I2C, WICED_I2C_REPEATED_START_FLAG|WICED_I2C_STOP_FLAG, data, 2 );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_ERROR( ( "Failed to send ADC read request.\r\n" ) );
        return PLATFORM_ERROR;
    }

    *output = ((uint16_t)(data[0]&0x0F)<<8 | (uint16_t)data[1]);
    return PLATFORM_SUCCESS;
}

platform_result_t max11615_take_sample_stream( const platform_adc_t* adc, void* buffer, uint16_t buffer_length )
{
    UNUSED_PARAMETER( adc );
    UNUSED_PARAMETER( buffer );
    UNUSED_PARAMETER( buffer_length );
    return PLATFORM_UNSUPPORTED;
}
