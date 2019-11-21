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
 * STM32 HAL based I2C implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "wwd_rtos.h"
#include "wwd_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define I2C_TRANSMIT_TIMEOUT ( 1000 )
#define I2C_RECEIVE_TIMEOUT  ( 1000 )

/* I2C bus frequency in Hz based on speed mode */
#define I2C_LOW_SPEED_MODE_FREQ_HZ      (10000)
#define I2C_STANDARD_SPEED_MODE_FREQ_HZ (100000)
#define I2C_HIGH_SPEED_MODE_FREQ_HZ     (400000)

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

static platform_result_t i2c_transmit_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length );
static platform_result_t i2c_receive_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length );

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern const platform_i2c_clock_enable_function_t i2c_clock_enable_function[ ];
extern const platform_i2c_clock_disable_function_t i2c_clock_disable_function[ ];
extern const uint8_t i2c_alternate_functions[ ];

/* Mapping of I2C port number to I2C_Handle */
static I2C_HandleTypeDef i2c_handles[NUMBER_OF_I2C_PORTS];

/******************************************************
 *               Function Definitions
 ******************************************************/

static platform_result_t platform_i2c_pin_init( const platform_i2c_t* i2c, FunctionalState state )
{
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );

    if ( state == ENABLE )
    {
        if ( i2c->pin_scl != NULL )
        {
            platform_gpio_set_alternate_function( i2c->pin_scl->port, i2c->pin_scl->pin_number, GPIO_MODE_AF_OD, GPIO_NOPULL, i2c_alternate_functions[ i2c_number ] );
        }

        if ( i2c->pin_sda != NULL )
        {
            platform_gpio_set_alternate_function( i2c->pin_sda->port, i2c->pin_sda->pin_number, GPIO_MODE_AF_OD, GPIO_NOPULL, i2c_alternate_functions[ i2c_number ] );
        }
    }
    else
    {
        if ( i2c->pin_scl != NULL )
        {
            HAL_GPIO_DeInit( i2c->pin_scl->port, (uint32_t)(1 << i2c->pin_scl->pin_number) );
        }

        if ( i2c->pin_sda != NULL )
        {
            HAL_GPIO_DeInit( i2c->pin_sda->port, (uint32_t)(1 << i2c->pin_sda->pin_number) );
        }
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init( const platform_i2c_t* i2c, const platform_i2c_config_t* config )
{
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;

    wiced_assert( "bad argument", ( i2c != NULL ) && ( config != NULL ) );

    platform_mcu_powersave_disable( );

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[i2c_number];

    /* Enable I2C peripheral pins */
    platform_i2c_pin_init( i2c, ENABLE );

    /* Enable I2C peripheral clock */
    i2c_clock_enable_function[ i2c_number ]( );

    /* Initialize the I2C InitStruct */
    i2c_handle->Instance             = i2c->port;
    i2c_handle->Init.OwnAddress1     = 0x00;
    i2c_handle->Init.OwnAddress2     = 0x00;
    i2c_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if ( config->speed_mode == I2C_LOW_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_LOW_SPEED_MODE_FREQ_HZ );
    }
    else if ( config->speed_mode == I2C_STANDARD_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_STANDARD_SPEED_MODE_FREQ_HZ );
    }
    else if ( config->speed_mode == I2C_HIGH_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_HIGH_SPEED_MODE_FREQ_HZ );
    }

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    }
    else if ( config->address_width == I2C_ADDRESS_WIDTH_10BIT )
    {
        i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    }

    /* Enable and initialize the I2C bus */
    if ( HAL_I2C_Init( i2c_handle ) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    platform_mcu_powersave_enable( );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_deinit( const platform_i2c_t* i2c, const platform_i2c_config_t* config )
{
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;

    wiced_assert( "bad argument", ( i2c != NULL ) && ( config != NULL ) );

    platform_mcu_powersave_disable( );

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[i2c_number];

    HAL_I2C_DeInit( i2c_handle );

    /* Disable I2C peripheral clock */
    i2c_clock_disable_function[ i2c_number ]( );

    /* Disable I2C peripheral pins */
    platform_i2c_pin_init( i2c, DISABLE );

    platform_mcu_powersave_enable( );

    return PLATFORM_SUCCESS;
}

static platform_result_t i2c_transmit_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length )
{
    uint16_t address;
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[i2c_number];

    address = config->address;

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        address <<= 1;
    }

    if ( HAL_I2C_Master_Transmit(i2c_handle, address, buffer, length, I2C_TRANSMIT_TIMEOUT) == HAL_OK )
    {
        return PLATFORM_SUCCESS;
    }
    else
    {
        return PLATFORM_ERROR;
    }
}

static platform_result_t i2c_receive_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length )
{
    uint16_t address;
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[i2c_number];

    address = config->address;

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        address <<= 1;
    }

    if ( HAL_I2C_Master_Receive(i2c_handle, address, buffer, length, I2C_RECEIVE_TIMEOUT) == HAL_OK )
    {
        return PLATFORM_SUCCESS;
    }
    else
    {
        return PLATFORM_ERROR;
    }
}

platform_result_t platform_i2c_init_tx_message( platform_i2c_message_t* message, const void* tx_buffer, uint16_t tx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "bad argument", ( message != NULL ) && ( tx_buffer != NULL ) && ( tx_buffer_length != 0 ) );

    if ( disable_dma == WICED_FALSE )
    {
        return PLATFORM_UNSUPPORTED;
    }

    memset( message, 0x00, sizeof( *message ) );

    message->tx_buffer = tx_buffer;
    message->retries   = retries;
    message->tx_length = tx_buffer_length;
    message->flags     = 0;

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init_rx_message( platform_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "bad argument", ( message != NULL ) && ( rx_buffer != NULL ) && ( rx_buffer_length != 0 ) );

    if ( disable_dma == WICED_FALSE )
    {
        return PLATFORM_UNSUPPORTED;
    }

    memset( message, 0x00, sizeof( *message ) );

    message->rx_buffer = rx_buffer;
    message->retries   = retries;
    message->rx_length = rx_buffer_length;
    message->flags     = 0;

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init_combined_message( platform_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "Not implemented", 0 );
    return PLATFORM_UNSUPPORTED;
}

wiced_bool_t platform_i2c_probe_device( const platform_i2c_t* i2c, const platform_i2c_config_t* config, int retries )
{
    uint32_t    i;
    uint8_t     dummy[2];
    platform_result_t result;

    /* Read two bytes from the addressed-slave. The slave-address won't be
     * acknowledged if it isn't on the I2C bus. The read won't trigger
     * a NACK from the slave (unless of error), since only the receiver can do that.
     */
    for ( i = 0; i < retries; i++ )
    {
        result = i2c_receive_data( i2c, config, dummy, sizeof dummy );

        if (  result == PLATFORM_SUCCESS )
        {
            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

platform_result_t platform_i2c_transfer( const platform_i2c_t* i2c, const platform_i2c_config_t* config, platform_i2c_message_t* messages, uint16_t number_of_messages )
{
    platform_result_t result = PLATFORM_SUCCESS;
    uint32_t          message_count;
    uint32_t          retries;

    /* Check for message validity. Combo message is unsupported */
    for ( message_count = 0; message_count < number_of_messages; message_count++ )
    {
        if ( messages[message_count].rx_buffer != NULL && messages[message_count].tx_buffer != NULL )
        {
            return PLATFORM_UNSUPPORTED;
        }

        if ( ( messages[message_count].tx_buffer == NULL && messages[message_count].tx_length != 0 )  ||
             ( messages[message_count].tx_buffer != NULL && messages[message_count].tx_length == 0 )  ||
             ( messages[message_count].rx_buffer == NULL && messages[message_count].rx_length != 0 )  ||
             ( messages[message_count].rx_buffer != NULL && messages[message_count].rx_length == 0 )  )
        {
            return PLATFORM_BADARG;
        }

        if ( messages[message_count].tx_buffer == NULL && messages[message_count].rx_buffer == NULL )
        {
            return PLATFORM_BADARG;
        }
    }

    platform_mcu_powersave_disable();

    for ( message_count = 0; message_count < number_of_messages; message_count++ )
    {
        for ( retries = 0; retries < messages[message_count].retries; retries++ )
        {
            if ( messages[message_count].tx_length != 0 )
            {
                result = i2c_transmit_data( i2c, config, (uint8_t*) messages[message_count].tx_buffer, messages[message_count].tx_length );

                if ( result == PLATFORM_SUCCESS )
                {
                    /* Transaction successful. Break from the inner loop and continue with the next message */
                    break;
                }
            }
            else if ( messages[message_count].rx_length != 0 )
            {
                result = i2c_receive_data( i2c, config, (uint8_t*) messages[message_count].rx_buffer, messages[message_count].rx_length );

                if ( result == PLATFORM_SUCCESS )
                {
                    /* Transaction successful. Break from the inner loop and continue with the next message */
                    break;
                }
            }
        }

        /* Check if retry is maxed out. If yes, return immediately */
        if ( retries == messages[message_count].retries && result != PLATFORM_SUCCESS )
        {
            result = PLATFORM_ERROR;
            break;
        }
    }

    platform_mcu_powersave_enable();

    return result;
}

platform_result_t platform_i2c_write( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint16_t flags, const void* buffer, uint16_t buffer_length )
{
    platform_result_t result;

    result = i2c_transmit_data( i2c, config, (uint8_t*) buffer, buffer_length );

    return result;
}

platform_result_t platform_i2c_read( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint16_t flags, void* buffer, uint16_t buffer_length )
{
    platform_result_t result;

    result = i2c_receive_data( i2c, config, (uint8_t*) buffer, buffer_length );

    return result;
}
