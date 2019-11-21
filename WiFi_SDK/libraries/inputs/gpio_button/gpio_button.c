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
 * GPIO-button implementation
 *
 */

#include "wwd_assert.h"
#include "wiced_platform.h"
#include "gpio_button.h"

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

static void gpio_button_interrupt_handler( void* args );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static gpio_button_state_change_callback_t button_state_change_callback;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t gpio_button_init( const gpio_button_t* button )
{
    wiced_assert( "Bad args", (button != NULL) );
    return wiced_gpio_init( button->gpio, ( button->polarity == WICED_ACTIVE_HIGH )? INPUT_PULL_UP: INPUT_PULL_DOWN );
}

wiced_result_t gpio_button_deinit( const gpio_button_t* button )
{
    return wiced_gpio_deinit( button->gpio );
}

wiced_result_t gpio_button_register_state_change_callback( gpio_button_state_change_callback_t callback )
{
    if ( !callback )
    {
        return WICED_BADARG;
    }

    button_state_change_callback = callback;

    return WICED_SUCCESS;
}

wiced_result_t gpio_button_enable( const gpio_button_t* button )
{
    wiced_gpio_irq_trigger_t trigger;

    wiced_assert( "Bad args", (button != NULL) );

    if ( button->trigger == 0 )
    {
        trigger = ( ( button->polarity == WICED_ACTIVE_LOW ) ? IRQ_TRIGGER_RISING_EDGE : IRQ_TRIGGER_FALLING_EDGE );
    }
    else
    {
        trigger = button->trigger;
    }

    return wiced_gpio_input_irq_enable( button->gpio, trigger, gpio_button_interrupt_handler, (void*)button );
}

wiced_result_t gpio_button_disable( const gpio_button_t* button )
{
    wiced_assert( "Bad args", (button != NULL) );

    return wiced_gpio_input_irq_disable( button->gpio );
}

wiced_bool_t gpio_button_get_value( const gpio_button_t* button )
{
    wiced_bool_t value;

    wiced_assert( "Bad args", (button != NULL) );

    value = wiced_gpio_input_get( button->gpio );
    return value;
}


static void gpio_button_interrupt_handler( void* args )
{
    const gpio_button_t* button = (const gpio_button_t*)args;
    wiced_bool_t   gpio_state;
    wiced_bool_t   is_pressed;

    if( !button_state_change_callback || !button )
    {
        return;
    }

    gpio_state = wiced_gpio_input_get( button->gpio );

    is_pressed = ( button->polarity == WICED_ACTIVE_HIGH ) ? ( (gpio_state  == WICED_FALSE ) ? WICED_TRUE : WICED_FALSE ) : ( (gpio_state == WICED_FALSE ) ? WICED_FALSE : WICED_TRUE );

    button_state_change_callback( (void*)button, is_pressed );
}
