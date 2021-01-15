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
 * STM32H7xx GPIO implementation
 */
#include "stdint.h"
#include "string.h"
#include "platform_peripheral.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "wwd_rtos_isr.h"
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
    platform_gpio_port_t*        owner_port; // GPIO port owning the IRQ line (line is shared across all GPIO ports)
    platform_gpio_irq_callback_t handler;    // User callback
    void*                        arg;        // User argument to be passed to the callbackA
} platform_gpio_irq_data_t;


/* Runtime GPIO IRQ data */
static volatile platform_gpio_irq_data_t gpio_irq_data[NUMBER_OF_GPIO_IRQ_LINES];

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

void hal_rcc_gpioa_clock_enable( void )
{
    __HAL_RCC_GPIOA_CLK_ENABLE( );
}

void hal_rcc_gpiob_clock_enable( void )
{
    __HAL_RCC_GPIOB_CLK_ENABLE( );
}

void hal_rcc_gpioc_clock_enable( void )
{
    __HAL_RCC_GPIOC_CLK_ENABLE( );
}

void hal_rcc_gpiod_clock_enable( void )
{
    __HAL_RCC_GPIOD_CLK_ENABLE( );
}

void hal_rcc_gpioe_clock_enable( void )
{
    __HAL_RCC_GPIOE_CLK_ENABLE( );
}

void hal_rcc_gpiof_clock_enable( void )
{
    __HAL_RCC_GPIOF_CLK_ENABLE( );
}

void hal_rcc_gpiog_clock_enable( void )
{
    __HAL_RCC_GPIOG_CLK_ENABLE( );
}

void hal_rcc_gpioh_clock_enable( void )
{
    __HAL_RCC_GPIOH_CLK_ENABLE( );
}

void hal_rcc_gpioi_clock_enable( void )
{
    __HAL_RCC_GPIOI_CLK_ENABLE( );
}

void hal_rcc_gpioj_clock_enable( void )
{
    __HAL_RCC_GPIOJ_CLK_ENABLE( );
}

void hal_rcc_gpiok_clock_enable( void )
{
    __HAL_RCC_GPIOK_CLK_ENABLE( );
}

void hal_rcc_gpioa_clock_disable( void )
{
    __HAL_RCC_GPIOA_CLK_DISABLE( );
}

void hal_rcc_gpiob_clock_disable( void )
{
    __HAL_RCC_GPIOB_CLK_DISABLE( );
}

void hal_rcc_gpioc_clock_disable( void )
{
    __HAL_RCC_GPIOC_CLK_DISABLE( );
}

void hal_rcc_gpiod_clock_disable( void )
{
    __HAL_RCC_GPIOD_CLK_DISABLE( );
}

void hal_rcc_gpioe_clock_disable( void )
{
    __HAL_RCC_GPIOE_CLK_DISABLE( );
}

void hal_rcc_gpiof_clock_disable( void )
{
    __HAL_RCC_GPIOF_CLK_DISABLE( );
}

void hal_rcc_gpiog_clock_disable( void )
{
    __HAL_RCC_GPIOG_CLK_DISABLE( );
}

void hal_rcc_gpioh_clock_disable( void )
{
    __HAL_RCC_GPIOH_CLK_DISABLE( );
}

void hal_rcc_gpioi_clock_disable( void )
{
    __HAL_RCC_GPIOI_CLK_DISABLE( );
}

void hal_rcc_gpioj_clock_disable( void )
{
    __HAL_RCC_GPIOJ_CLK_DISABLE( );
}

void hal_rcc_gpiok_clock_disable( void )
{
    __HAL_RCC_GPIOK_CLK_DISABLE( );
}

/*GPIO外围时钟启用功能 */
const platform_gpio_clock_enable_function_t platform_gpio_clock_enable_function[ NUMBER_OF_GPIO_PORTS ] =
{
    [0]  = hal_rcc_gpioa_clock_enable,
    [1]  = hal_rcc_gpiob_clock_enable,
    [2]  = hal_rcc_gpioc_clock_enable,
    [3]  = hal_rcc_gpiod_clock_enable,
    [4]  = hal_rcc_gpioe_clock_enable,
    [5]  = hal_rcc_gpiof_clock_enable,
    [6]  = hal_rcc_gpiog_clock_enable,
    [7]  = hal_rcc_gpioh_clock_enable,
    [8]  = hal_rcc_gpioi_clock_enable,
    [9]  = hal_rcc_gpioj_clock_enable,
    [10] = hal_rcc_gpiok_clock_enable,
};
/*
根据索引开启时钟
*/
void gpio_peripheral_clocks(int index)
{
	switch(index)
	{
		case 0:		hal_rcc_gpioa_clock_enable();		break;
		case 1:		hal_rcc_gpiob_clock_enable();		break;
		case 2:		hal_rcc_gpioc_clock_enable();		break;
		case 3:		hal_rcc_gpiod_clock_enable();		break;
		case 4:		hal_rcc_gpioe_clock_enable();		break;
		case 5:		hal_rcc_gpiof_clock_enable();		break;
		case 6:		hal_rcc_gpiog_clock_enable();		break;
		case 7:		hal_rcc_gpioh_clock_enable();		break;
		case 8:		hal_rcc_gpioi_clock_enable();		break;
		case 9:		hal_rcc_gpioj_clock_enable();		break;
	  case 10:	hal_rcc_gpiok_clock_enable();		break;
	
	}
}


/*GPIO外围时钟禁用功能 */
const platform_gpio_clock_disable_function_t platform_gpio_clock_disable_function[ NUMBER_OF_GPIO_PORTS ] =
{
    [0]  = hal_rcc_gpioa_clock_disable,
    [1]  = hal_rcc_gpiob_clock_disable,
    [2]  = hal_rcc_gpioc_clock_disable,
    [3]  = hal_rcc_gpiod_clock_disable,
    [4]  = hal_rcc_gpioe_clock_disable,
    [5]  = hal_rcc_gpiof_clock_disable,
    [6]  = hal_rcc_gpiog_clock_disable,
    [7]  = hal_rcc_gpioh_clock_disable,
    [8]  = hal_rcc_gpioi_clock_disable,
    [9]  = hal_rcc_gpioj_clock_disable,
    [10] = hal_rcc_gpiok_clock_disable,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

uint8_t platform_gpio_get_port_number( platform_gpio_port_t * gpio_port )
{
    uint8_t port_index = ( (uint32_t) gpio_port - GPIOA_BASE ) / 0x400;

    if ( ( (uint32_t) gpio_port < GPIOA_BASE ) || ( (uint32_t) gpio_port > GPIOK_BASE ) )
    {
        return INVALID_GPIO_PORT_NUMBER;
    }

    return ( port_index );
}

/******************************************************
 *            Platform Function Definitions
 ******************************************************/
/*
IO初始化
*/
platform_result_t platform_gpio_init( const platform_gpio_t* gpio, platform_pin_config_t config )
{
    GPIO_InitTypeDef  gpio_init_structure;
    uint8_t           port_number;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    port_number = platform_gpio_get_port_number( gpio->port );

    /*使能时钟端口*/
   // platform_gpio_clock_enable_function[port_number];
		gpio_peripheral_clocks(port_number);

    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Mode  = ( ( config == INPUT_PULL_UP ) || ( config == INPUT_PULL_DOWN ) || ( config == INPUT_HIGH_IMPEDANCE ) ) ? 0x00 : 0x01;
    	
//    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
//    gpio_init_structure.Mode  = ( ( config == INPUT_PULL_UP ) || ( config == INPUT_PULL_DOWN ) || ( config == INPUT_HIGH_IMPEDANCE ) ) ? GPIO_Mode_IN : GPIO_Mode_OUT;
//    gpio_init_structure.OType = ( config == OUTPUT_PUSH_PULL ) ? GPIO_OType_PP : GPIO_OType_OD;

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

    gpio_init_structure.Pin = (uint32_t) ( 1 << gpio->pin_number );

    HAL_GPIO_Init( gpio->port, &gpio_init_structure );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}


platform_result_t platform_gpio_deinit( const platform_gpio_t* gpio )
{
    GPIO_InitTypeDef  gpio_init_structure;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    /* Set to Input high-impedance */
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Pin   = (uint32_t) ( 1 << gpio->pin_number );

    HAL_GPIO_DeInit( gpio->port, gpio->pin_number );

    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

/*
拉高
*/
platform_result_t platform_gpio_output_high( const platform_gpio_t* gpio )
{
//    wiced_assert( "bad argument", ( gpio != NULL ) );

//    platform_mcu_powersave_disable( );

//    gpio->port->BSRR = (uint16_t) ( 1 << gpio->pin_number );

//    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

/*
拉低
*/
platform_result_t platform_gpio_output_low( const platform_gpio_t* gpio )
{
//    wiced_assert( "bad argument", ( gpio != NULL ) );

//    platform_mcu_powersave_disable();

//    gpio->port->BSRR = (uint16_t) ( 1 << gpio->pin_number );

//    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}

/*
获取引脚输入
*/
wiced_bool_t platform_gpio_input_get( const platform_gpio_t* gpio )
{
    wiced_bool_t result;

    wiced_assert( "bad argument", ( gpio != NULL ) );

    platform_mcu_powersave_disable();

    result = ( ( gpio->port->IDR & (uint32_t) ( 1 << gpio->pin_number ) ) != 0 ) ? WICED_TRUE : WICED_FALSE;

    platform_mcu_powersave_enable();

    return result;
}

/*
使能中断引脚
*/
platform_result_t platform_gpio_irq_enable( const platform_gpio_t* gpio, platform_gpio_irq_callback_t handler, void* arg )
{
	GPIO_InitTypeDef GPIO_InitStructure; 
  uint32_t            interrupt_line;//中断线
	interrupt_line = (uint32_t) ( 1 << gpio->pin_number );//根据IO端口算出中断线
	__GPIOC_CLK_ENABLE();


	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;//边缘触发中断	

	GPIO_InitStructure.Pull = GPIO_NOPULL;
	    
	GPIO_InitStructure.Pin = (uint32_t) ( 1 << gpio->pin_number );
	
	gpio_irq_data[gpio->pin_number].owner_port = gpio->port;
	gpio_irq_data[gpio->pin_number].handler    = handler;
	gpio_irq_data[gpio->pin_number].arg        = arg;

	HAL_GPIO_Init(gpio->port, &GPIO_InitStructure);      

	HAL_NVIC_SetPriority(interrupt_line, 0, 0);

	HAL_NVIC_EnableIRQ(interrupt_line);
}




platform_result_t platform_gpio_irq_disable( const platform_gpio_t* gpio )
{
//未使用
}


platform_result_t platform_gpio_deepsleep_wakeup_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger )
{
    return PLATFORM_UNSUPPORTED;
}

platform_result_t platform_gpio_irq_manager_init( void )
{
    memset( (void*)gpio_irq_data, 0, sizeof( gpio_irq_data ) );

    /*打开SYSCFG外围时钟以允许写入SYSCFG寄存器*/
    //RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE );
		//先注释

    return PLATFORM_SUCCESS;
}

				
//					 if ( PLATFORM_SUCCESS != platform_gpio_set_alternate_function( 
//					wifi_sdio_pins[ a ].port, 
//					wifi_sdio_pins[ a ].pin_number, 
//					GPIO_MODE_AF_PP, 
//					GPIO_PULLUP, 
//					//fuyong   			GPIO_AF12_SDIO1 ) )
				
platform_result_t platform_gpio_set_alternate_function
																											(                                		
																											platform_gpio_port_t* gpio_port, 		
																											uint8_t pin_number,              		
																											uint32_t output_type,            		
																											uint32_t pull_up_down_type,      		
																											uint8_t alternation_function )			
{
    GPIO_InitTypeDef  gpio_init_structure;
		/*获取端口号*/
    uint8_t           port_number = platform_gpio_get_port_number( gpio_port );

    platform_mcu_powersave_disable();

    /*启用端口外设时钟  */
		//platform_gpio_clock_enable_function[port_number];
		gpio_peripheral_clocks(port_number);
		/*配置引脚功能*/
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Mode  = output_type;
    gpio_init_structure.Alternate = alternation_function;
    gpio_init_structure.Pull  = pull_up_down_type;
    gpio_init_structure.Pin   = (uint32_t) ( 1 << pin_number );
		/*初始化引脚*/
    HAL_GPIO_Init( gpio_port, &gpio_init_structure);
	
    platform_mcu_powersave_enable();

    return PLATFORM_SUCCESS;
}



/******************************************************
 *               IRQ Handler Definitions
 ******************************************************/

/* Common IRQ handler for all GPIOs */
//WWD_RTOS_DEFINE_ISR( gpio_irq )
//{

//}

