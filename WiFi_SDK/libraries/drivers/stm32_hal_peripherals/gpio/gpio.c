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
 * STM32 HAL based common GPIO implementation
 */
#include "stdint.h"
#include "string.h"
#include "platform_peripheral.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "wwd_rtos.h"
#include "wwd_assert.h"

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

/* Structure of runtime GPIO IRQ data */
typedef struct
{
    wiced_bool_t                 enabled;    // Active status of the GPIO IRQ
    platform_gpio_port_t*        owner_port; // GPIO port owning the IRQ line (line is shared across all GPIO ports)
    platform_gpio_irq_callback_t handler;    // User callback
    void*                        arg;        // User argument to be passed to the callbackA
} platform_gpio_irq_data_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern const platform_gpio_clock_enable_function_t platform_gpio_clock_enable_function[ ];
extern const platform_gpio_clock_disable_function_t platform_gpio_clock_disable_function[ ];

/* Runtime GPIO IRQ data */
static volatile platform_gpio_irq_data_t gpio_irq_data[NUMBER_OF_GPIO_IRQ_LINES];

/******************************************************
 *            Platform Function Definitions
 ******************************************************/

platform_result_t platform_gpio_init( const platform_gpio_t* gpio, platform_pin_config_t config )
{
    GPIO_InitTypeDef  gpio_init_structure;
    uint8_t port_number;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    port_number = platform_gpio_get_port_number( gpio->port );

    if ( port_number == INVALID_GPIO_PORT_NUMBER )
    {
        return PLATFORM_ERROR;
    }

    platform_mcu_powersave_disable();

    /* Enable peripheral clock for this port */
    platform_gpio_clock_enable_function[ port_number ]( );

    gpio_init_structure.Pin = (uint32_t) ( 1 << gpio->pin_number );
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if ( ( config == INPUT_PULL_UP ) || ( config == INPUT_PULL_DOWN ) || ( config == INPUT_HIGH_IMPEDANCE ) )
    {
        gpio_init_structure.Mode = GPIO_MODE_INPUT;
    }
    else if ( config == OUTPUT_PUSH_PULL )
    {
        gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    }
    else
    {
        gpio_init_structure.Mode = GPIO_MODE_OUTPUT_OD;
    }

    if ( ( config == INPUT_PULL_UP ) || ( config == OUTPUT_OPEN_DRAIN_PULL_UP ) )
    {
        gpio_init_structure.Pull = GPIO_PULLUP;
    }
    else if ( config == INPUT_PULL_DOWN )
    {
        gpio_init_structure.Pull = GPIO_PULLDOWN;
    }
    else
    {
        gpio_init_structure.Pull = GPIO_NOPULL;
    }

    HAL_GPIO_Init( gpio->port, &gpio_init_structure );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_deinit( const platform_gpio_t* gpio )
{
    uint8_t port_number;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    port_number = platform_gpio_get_port_number( gpio->port );

    if ( port_number == INVALID_GPIO_PORT_NUMBER )
    {
        return PLATFORM_ERROR;
    }

    platform_mcu_powersave_disable();

    HAL_GPIO_DeInit( gpio->port, (uint32_t)(1 << gpio->pin_number) );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_high( const platform_gpio_t* gpio )
{
    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable( );

    HAL_GPIO_WritePin(gpio->port, (uint32_t) ( 1 << gpio->pin_number ), GPIO_PIN_SET);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_low( const platform_gpio_t* gpio )
{
    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    HAL_GPIO_WritePin(gpio->port, (uint32_t) ( 1 << gpio->pin_number ), GPIO_PIN_RESET);

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

wiced_bool_t platform_gpio_input_get( const platform_gpio_t* gpio )
{
    wiced_bool_t result = WICED_FALSE;
    GPIO_PinState pin_state;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    pin_state = HAL_GPIO_ReadPin(gpio->port, (uint32_t) ( 1 << gpio->pin_number ));

    if (pin_state == GPIO_PIN_SET)
    {
        result = WICED_TRUE;
    }

    platform_mcu_powersave_enable();

    return result;
}

platform_result_t platform_gpio_irq_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger, platform_gpio_irq_callback_t handler, void* arg )
{
    GPIO_InitTypeDef  gpio_init_structure;
    IRQn_Type         interrupt_vector;
    uint32_t          interrupt_line;
    uint8_t           port_number;

    wiced_assert( "bad argument", ( gpio != NULL ) && ( handler != NULL ) );

    port_number = platform_gpio_get_port_number( gpio->port );

    if ( port_number == INVALID_GPIO_PORT_NUMBER )
    {
        return PLATFORM_ERROR;
    }

    interrupt_line = (uint32_t) ( 1 << gpio->pin_number );

    if ( ( interrupt_line & 0x001F ) != 0 )
    {
        /* Line 0 to 4 */
        interrupt_vector = (IRQn_Type) ( EXTI0_IRQn + gpio->pin_number );
    }
    else if ( ( interrupt_line & 0x03E0 ) != 0 )
    {
        /* Line 5 to 9 */
        interrupt_vector = EXTI9_5_IRQn;
    }
    else if ( ( interrupt_line & 0xFC00 ) != 0 )
    {
        /* Line 10 to 15 */
        interrupt_vector = EXTI15_10_IRQn;
    }
    else
    {
        return PLATFORM_BADARG;
    }

    switch ( trigger )
    {
        case IRQ_TRIGGER_RISING_EDGE:
        {
            gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
            break;
        }
        case IRQ_TRIGGER_FALLING_EDGE:
        {
            gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
            break;
        }
        case IRQ_TRIGGER_BOTH_EDGES:
        {
            gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
            break;
        }
        default:
        {
            return PLATFORM_BADARG;
        }
    }

    platform_mcu_powersave_disable();

    /* Enable peripheral clock for this port */
    platform_gpio_clock_enable_function[ port_number ]( );

    /*
     * Note that the GPIO Pull setting is configured as GPIO_NOPULL.
     * The GPIO pin should have the external pull-up/pull-down setup
     * appropriately in order for interrupt triggering to work properly.
     */
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Pin  = interrupt_line;

    gpio_irq_data[gpio->pin_number].enabled    = WICED_TRUE;
    gpio_irq_data[gpio->pin_number].owner_port = gpio->port;
    gpio_irq_data[gpio->pin_number].handler    = handler;
    gpio_irq_data[gpio->pin_number].arg        = arg;

    HAL_GPIO_Init( gpio->port, &gpio_init_structure );

    NVIC_EnableIRQ( interrupt_vector );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_irq_disable( const platform_gpio_t* gpio )
{
    wiced_bool_t      interrupt_line_used;
    IRQn_Type         interrupt_vector;
    uint32_t          interrupt_line;
    uint8_t           pin_number;
    uint8_t           port_number;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    port_number = platform_gpio_get_port_number( gpio->port );

    if ( port_number == INVALID_GPIO_PORT_NUMBER )
    {
        return PLATFORM_ERROR;
    }

    interrupt_line = (uint32_t) ( 1 << gpio->pin_number );

    if ( ( interrupt_line & 0x001F ) != 0 )
    {
        /* Line 0 to 4 */
        interrupt_vector = (IRQn_Type) ( EXTI0_IRQn + gpio->pin_number );
    }
    else if ( ( interrupt_line & 0x03E0 ) != 0 )
    {
        /* Line 5 to 9 */
        interrupt_vector = EXTI9_5_IRQn;
    }
    else if ( ( interrupt_line & 0xFC00 ) != 0 )
    {
        /* Line 10 to 15 */
        interrupt_vector = EXTI15_10_IRQn;
    }
    else
    {
        return PLATFORM_BADARG;
    }

    platform_mcu_powersave_disable();

    if ( gpio_irq_data[gpio->pin_number].owner_port == gpio->port )
    {
        interrupt_line_used = WICED_FALSE;

        if ( interrupt_vector == EXTI9_5_IRQn )
        {
            for ( pin_number = 5 ; pin_number < 10 ; pin_number++ )
            {
                if ( (pin_number != gpio->pin_number) && (gpio_irq_data[pin_number].enabled == WICED_TRUE) )
                {
                    interrupt_line_used = WICED_TRUE;
                }
            }
        }
        else if ( interrupt_vector == EXTI15_10_IRQn )
        {
            for ( pin_number = 10 ; pin_number < 16 ; pin_number++ )
            {
                if ( (pin_number != gpio->pin_number) && (gpio_irq_data[pin_number].enabled == WICED_TRUE) )
                {
                    interrupt_line_used = WICED_TRUE;
                }
            }
        }

        if ( interrupt_line_used == WICED_FALSE )
        {
            NVIC_DisableIRQ( interrupt_vector );
        }

        HAL_GPIO_DeInit( gpio->port, (uint32_t)(1 << gpio->pin_number) );

        gpio_irq_data[gpio->pin_number].enabled    = WICED_FALSE;
        gpio_irq_data[gpio->pin_number].owner_port = 0;
        gpio_irq_data[gpio->pin_number].handler    = 0;
        gpio_irq_data[gpio->pin_number].arg        = 0;
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_irq_manager_init( void )
{
    memset( (void*)gpio_irq_data, 0, sizeof( gpio_irq_data ) );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_set_alternate_function( platform_gpio_port_t* gpio_port, uint8_t pin_number, uint32_t mode, uint32_t pull_up_down_type, uint32_t alternation_function )
{
    GPIO_InitTypeDef    gpio_init_structure;
    uint8_t port_number;

    port_number = platform_gpio_get_port_number( gpio_port );

    if ( port_number == INVALID_GPIO_PORT_NUMBER )
    {
        return PLATFORM_ERROR;
    }

    platform_mcu_powersave_disable();

    gpio_init_structure.Pin       = (uint32_t) ( 1 << pin_number );
    gpio_init_structure.Mode      = mode;
    gpio_init_structure.Pull      = pull_up_down_type;
    gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_structure.Alternate = alternation_function;

    /* Enable peripheral clock for this port */
    platform_gpio_clock_enable_function[ port_number ]( );

    HAL_GPIO_Init( gpio_port, &gpio_init_structure );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

/******************************************************
 *               IRQ Handler Definitions
 ******************************************************/

/* Common IRQ handler for all GPIOs */
void platform_gpio_irq( void )
{
    uint32_t active_interrupt_vector = (uint32_t) ( ( SCB->ICSR & 0x3fU ) - 16 );
    uint32_t gpio_number;
    uint32_t interrupt_line;

    switch ( active_interrupt_vector )
    {
        case EXTI0_IRQn:
            interrupt_line = GPIO_PIN_0;
            gpio_number = 0;
            break;
        case EXTI1_IRQn:
            interrupt_line = GPIO_PIN_1;
            gpio_number = 1;
            break;
        case EXTI2_IRQn:
            interrupt_line = GPIO_PIN_2;
            gpio_number = 2;
            break;
        case EXTI3_IRQn:
            interrupt_line = GPIO_PIN_3;
            gpio_number = 3;
            break;
        case EXTI4_IRQn:
            interrupt_line = GPIO_PIN_4;
            gpio_number = 4;
            break;
        case EXTI9_5_IRQn:
            interrupt_line = GPIO_PIN_5;
            for ( gpio_number = 5; gpio_number < 10 && __HAL_GPIO_EXTI_GET_IT(interrupt_line) == 0; gpio_number++ )
            {
                interrupt_line <<= 1;
            }
            break;
        case EXTI15_10_IRQn:
            interrupt_line = GPIO_PIN_10;
            for ( gpio_number = 10; gpio_number < 16 && __HAL_GPIO_EXTI_GET_IT(interrupt_line) == 0; gpio_number++ )
            {
                interrupt_line <<= 1;
            }
            break;
        default:
            return;
    }

    /* Clear interrupt flag */
    __HAL_GPIO_EXTI_CLEAR_IT(interrupt_line);

    /* Call the respective GPIO interrupt handler/callback */
    if ( gpio_irq_data[gpio_number].handler != NULL )
    {
        void* arg = gpio_irq_data[gpio_number].arg; /* Avoids undefined order of access to volatiles */
        gpio_irq_data[gpio_number].handler( arg );
    }
}
