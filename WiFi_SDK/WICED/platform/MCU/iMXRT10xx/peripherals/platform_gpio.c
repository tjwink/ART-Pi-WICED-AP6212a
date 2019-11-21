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
 * common GPIO implementation
 */
#include "string.h"
#include "platform_peripheral.h"
//#include "chip.h"
#include "wwd_assert.h"
#include "wwd_rtos_isr.h"

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
    platform_gpio_t                 gpio;    /* gpio used for interrupt */
    platform_gpio_irq_handler_t     handler; /* User callback   */
    void*                           arg;     /* User argument to be passed to the callbackA */
    platform_gpio_irq_trigger_t     trigger;
} platform_gpio_irq_data_t;


/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* Runtime GPIO IRQ data */
static platform_gpio_irq_data_t gpio_irq_data[NUMBER_OF_GPIO_IRQ_LINES];

/******************************************************
 *               Function Definitions
 ******************************************************/
platform_result_t platform_gpio_irq_manager_init( void )
{

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_init( const platform_gpio_t* gpio, platform_pin_config_t config )
{
//    uint16_t mode;
//    uint8_t  direction;

//    switch(config)
//    {
//        case INPUT_PULL_UP:
//        {
//            mode = SCU_MODE_PULLUP;
//            direction =LPC_PIN_INPUT;
//            break;
//        }
//        case INPUT_PULL_DOWN:
//        {
//            mode = SCU_MODE_PULLDOWN;
//            direction =LPC_PIN_INPUT;
//            break;
//        }
//        case INPUT_HIGH_IMPEDANCE:
//        {
//            mode = SCU_MODE_INACT;
//            direction =LPC_PIN_INPUT;
//            break;
//        }
//        case OUTPUT_PUSH_PULL:
//        {
//            mode = SCU_MODE_INACT;
//            direction =LPC_PIN_OUTPUT;
//            break;
//        }
//        case OUTPUT_OPEN_DRAIN_NO_PULL:
//        {
//            mode = SCU_MODE_INACT;
//            direction =LPC_PIN_OUTPUT;
//            break;
//        }
//        case OUTPUT_OPEN_DRAIN_PULL_UP:
//        {
//            mode = SCU_MODE_PULLUP;
//            direction =LPC_PIN_OUTPUT;
//            break;
//        }
//        default:
//        {
//            return PLATFORM_UNSUPPORTED;
//        }
//    }
//    Chip_SCU_PinMux( gpio->hw_pin.group, gpio->hw_pin.pin, mode, gpio->hw_pin.mode_function );
//    Chip_GPIO_SetPinDIR( LPC_GPIO_PORT, gpio->gpio_port, gpio->gpio_pin, direction );
    return PLATFORM_SUCCESS;
}

platform_result_t platform_pin_set_alternate_function( const platform_pin_t* hw_pin )
{
//    Chip_SCU_PinMuxSet( hw_pin->group, hw_pin->pin, hw_pin->mode_function );
    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_high( const platform_gpio_t* gpio)
{
//    Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio->gpio_port, gpio->gpio_pin, 1);
    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_output_low( const platform_gpio_t* gpio)
{
//    Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio->gpio_port, gpio->gpio_pin, 0);
    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_input_get( const platform_gpio_t* gpio )
{
    UNUSED_PARAMETER( gpio );
    return PLATFORM_SUCCESS;
}

/******************************************************
 *               IRQ Handler Definitions
 ******************************************************/
//static int8_t platform_gpio_irq_get_next_unused( void )
//{
//    /* Searches for the next empty IRQ to use. */
//    uint32_t index;
//    for ( index = 0; index < NUMBER_OF_GPIO_IRQ_LINES; index++ )
//    {
//        if ( gpio_irq_data[ index ].handler == NULL )
//        {
//            return index;
//        }
//    }
//    return -1;
//}

platform_result_t platform_gpio_irq_init( void )
{
    memset( (void*)gpio_irq_data, 0, sizeof( gpio_irq_data ) );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_gpio_irq_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger, platform_gpio_irq_handler_t handler, void* arg )
{
//    IRQn_Type IRQn;
//    int8_t irq_number = platform_gpio_irq_get_next_unused( );

//    if ( irq_number >= 0 )
//    {
//        gpio_irq_data[ irq_number ].handler = handler;
//        gpio_irq_data[ irq_number ].arg = arg;
//        gpio_irq_data[ irq_number ].trigger = trigger;
//        gpio_irq_data[ irq_number ].gpio = *gpio;

//        Chip_SCU_PinMuxSet( gpio->hw_pin.group, gpio->hw_pin.pin, ( SCU_MODE_INBUFF_EN | SCU_MODE_INACT | gpio->hw_pin.mode_function ) );
//        /* Configure GPIO pin as input */
//        Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, gpio->gpio_port, gpio->gpio_pin );
//        Chip_SCU_GPIOIntPinSel( irq_number, gpio->gpio_port, gpio->gpio_pin );
//        Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//        Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//        switch ( trigger )
//        {
//            case IRQ_TRIGGER_RISING_EDGE:
//                Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//                break;
//            case IRQ_TRIGGER_FALLING_EDGE:
//                Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//                break;
//            default:
//                //wiced_assert( "Unsupported Trigger", trigger == IRQ_TRIGGER_BOTH_EDGES);
//                Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//        }

//        switch ( irq_number )
//        {
//            case 0:
//                IRQn = PIN_INT0_IRQn;
//                break;
//            case 1:
//                IRQn = PIN_INT1_IRQn;
//                break;
//            case 2:
//                IRQn = PIN_INT2_IRQn;
//                break;
//            case 3:
//                IRQn = PIN_INT3_IRQn;
//                break;
//            case 4:
//                IRQn = PIN_INT4_IRQn;
//                break;
//            case 5:
//                IRQn = PIN_INT5_IRQn;
//                break;
//            case 6:
//                IRQn = PIN_INT6_IRQn;
//                break;
//            case 7:
//                IRQn = PIN_INT7_IRQn;
//                break;
//            default:
//                break;
//        }
//        NVIC_DisableIRQ( IRQn );
//        NVIC_ClearPendingIRQ( IRQn );
//        NVIC_SetPriority( IRQn, 0xE );
//        NVIC_EnableIRQ(IRQn );


//        return PLATFORM_SUCCESS;
//    }

    return PLATFORM_NO_EFFECT;
}

platform_result_t platform_gpio_irq_disable( const platform_gpio_t* gpio )
{
//    uint8_t irq_number;
//    IRQn_Type IRQn;

//    for ( irq_number = 0; irq_number < NUMBER_OF_GPIO_IRQ_LINES; irq_number++ )
//    {
//        if ( ( gpio_irq_data[ irq_number ].gpio.gpio_port == gpio->gpio_port ) &&
//             ( gpio_irq_data[ irq_number ].gpio.gpio_pin == gpio->gpio_pin )
//           )
//        {
//            switch ( irq_number )
//            {
//                case 0:
//                    IRQn = PIN_INT0_IRQn;
//                    break;
//                case 1:
//                    IRQn = PIN_INT1_IRQn;
//                    break;
//                case 2:
//                    IRQn = PIN_INT2_IRQn;
//                    break;
//                case 3:
//                    IRQn = PIN_INT3_IRQn;
//                    break;
//                case 4:
//                    IRQn = PIN_INT4_IRQn;
//                    break;
//                case 5:
//                    IRQn = PIN_INT5_IRQn;
//                    break;
//                case 6:
//                    IRQn = PIN_INT6_IRQn;
//                    break;
//                case 7:
//                    IRQn = PIN_INT7_IRQn;
//                    break;
//                default:
//                    break;
//            }
//            NVIC_DisableIRQ( IRQn );

//            switch ( gpio_irq_data[ irq_number ].trigger )
//            {
//                case IRQ_TRIGGER_RISING_EDGE:
//                    Chip_PININT_DisableIntHigh( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//                    break;
//                case IRQ_TRIGGER_FALLING_EDGE:
//                    Chip_PININT_DisableIntLow( LPC_GPIO_PIN_INT, PININTCH(irq_number) );
//                    break;
//                default:
//                    wiced_assert( "Unsupported Trigger", gpio_irq_data[ irq_number ].trigger == IRQ_TRIGGER_BOTH_EDGES);
//            }
//            gpio_irq_data[ irq_number ].handler  = 0;
//            gpio_irq_data[ irq_number ].arg      = 0;
//            gpio_irq_data[ irq_number ].trigger  = 0;

//            return PLATFORM_SUCCESS;
//        }
//    }
    return PLATFORM_NO_EFFECT;
}

/* ***************************************************
 * Common IRQ handler for all GPIOs.
 * Add checks for interrupt GPIO here as required.
 * Currently only check for SPI irq is performed
 *****************************************************/
WWD_RTOS_DEFINE_ISR( gpio_irq )
{
//    uint32_t irq_mask;
//    uint32_t irq_number;

//    irq_mask = Chip_PININT_GetIntStatus( LPC_GPIO_PIN_INT );
//    if (irq_mask != 1)
//    {
//        irq_mask = 1;
//    }
//    /* Clear interrupt flag */
//    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, irq_mask );
//    for ( irq_number = 0; irq_number < NUMBER_OF_GPIO_IRQ_LINES; irq_number++ )
//    {
//        if ( irq_mask & PININTCH(irq_number) )
//        {
//            /* Call the respective GPIO interrupt handler/callback */
//            if ( gpio_irq_data[ irq_number ].handler != NULL )
//            {
//                void * arg = gpio_irq_data[ irq_number ].arg; /* Avoids undefined order of access to volatiles */
//                gpio_irq_data[ irq_number ].handler( arg );
//            }
//        }
//    }
}

/******************************************************
 *               IRQ Handler Mapping
 ******************************************************/

WWD_RTOS_MAP_ISR( gpio_irq , GPIO0_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO1_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO2_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO3_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO4_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO5_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO6_irq )
WWD_RTOS_MAP_ISR( gpio_irq , GPIO7_irq )
