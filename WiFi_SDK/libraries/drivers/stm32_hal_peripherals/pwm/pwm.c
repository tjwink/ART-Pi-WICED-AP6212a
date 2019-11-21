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
 * STM32 HAL based PWM implementation
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

extern const platform_pwm_clock_enable_function_t pwm_clock_enable_function[ ];
extern const uint8_t pwm_alternate_functions[ ];

/* Mapping of PWM port number to TIM Handle */
static TIM_HandleTypeDef tim_handles[NUMBER_OF_PWM_PORTS];

/* Timer Output Compare configurations */
TIM_OC_InitTypeDef tim_oc_configs[NUMBER_OF_PWM_PORTS];

/******************************************************
 *               Function Definitions
 ******************************************************/

static platform_result_t platform_pwm_pin_init( const platform_pwm_t* pwm, FunctionalState state )
{
    uint8_t pwm_number;

    pwm_number = platform_pwm_get_port_number( pwm->port );

    if ( state == ENABLE )
    {
        if ( pwm->pin != NULL )
        {
            platform_gpio_set_alternate_function( pwm->pin->port, pwm->pin->pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, pwm_alternate_functions[ pwm_number ] );
        }
    }
    else
    {
        if ( pwm->pin != NULL )
        {
            HAL_GPIO_DeInit( pwm->pin->port, (uint32_t)(1 << pwm->pin->pin_number) );
        }
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_pwm_init( const platform_pwm_t* pwm, uint32_t frequency, float duty_cycle )
{
    uint8_t pwm_number;
    TIM_HandleTypeDef* tim_handle;
    TIM_OC_InitTypeDef* tim_oc_config;
    uint32_t tim_period;
    uint32_t tim_prescaler;

    wiced_assert( "bad argument", ( pwm != NULL ) && ( duty_cycle >= 0 ) && ( duty_cycle <= 100 ) );

    platform_mcu_powersave_disable( );

    pwm_number = platform_pwm_get_port_number( pwm->port );
    tim_handle = &tim_handles[pwm_number];
    tim_oc_config = &tim_oc_configs[pwm_number];

    /* Get PWM divider settings */
    if ( platform_pwm_get_divider(pwm->port, frequency, &tim_period, &tim_prescaler) != PLATFORM_SUCCESS )
    {
        return PLATFORM_UNSUPPORTED;
    }

    /* Enable PWM peripheral pins */
    platform_pwm_pin_init( pwm, ENABLE );

    /* Enable PWM peripheral clock */
    pwm_clock_enable_function[ pwm_number ]( );

    /* Configure the TIM peripheral */
    tim_handle->Instance               = pwm->port;
    tim_handle->Init.Prescaler         = tim_prescaler;
    tim_handle->Init.Period            = tim_period;
    tim_handle->Init.ClockDivision     = 0;
    tim_handle->Init.CounterMode       = TIM_COUNTERMODE_UP;
    tim_handle->Init.RepetitionCounter = 0;

    /* Configure the PWM channels */
    tim_oc_config->OCMode       = TIM_OCMODE_PWM1;
    tim_oc_config->OCPolarity   = TIM_OCPOLARITY_HIGH;
    tim_oc_config->OCFastMode   = TIM_OCFAST_DISABLE;
    tim_oc_config->OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    tim_oc_config->OCNIdleState = TIM_OCNIDLESTATE_RESET;
    tim_oc_config->OCIdleState  = TIM_OCIDLESTATE_RESET;

    if ( duty_cycle == 0 )
    {
        tim_oc_config->Pulse = 0;
    }
    else
    {
        tim_oc_config->Pulse = (((tim_period + 1) * ((uint32_t)duty_cycle)) / 100) - 1;
    }

    /* Initialize the TIM peripheral */
    if ( HAL_TIM_PWM_Init(tim_handle) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    /* Configure the TIM channel */
    if ( HAL_TIM_PWM_ConfigChannel(tim_handle, tim_oc_config, pwm->channel) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    platform_mcu_powersave_enable( );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_pwm_start( const platform_pwm_t* pwm )
{
    uint8_t pwm_number;
    TIM_HandleTypeDef* tim_handle;

    wiced_assert( "bad argument", ( pwm != NULL ) );

    pwm_number = platform_pwm_get_port_number( pwm->port );
    tim_handle = &tim_handles[pwm_number];

    /* Start PWM signals generation */
    if ( HAL_TIM_PWM_Start(tim_handle, pwm->channel) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_pwm_stop( const platform_pwm_t* pwm )
{
    uint8_t pwm_number;
    TIM_HandleTypeDef* tim_handle;

    wiced_assert( "bad argument", ( pwm != NULL ) );

    pwm_number = platform_pwm_get_port_number( pwm->port );
    tim_handle = &tim_handles[pwm_number];

    /* Stop PWM signals generation */
    if ( HAL_TIM_PWM_Stop(tim_handle, pwm->channel) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
}
