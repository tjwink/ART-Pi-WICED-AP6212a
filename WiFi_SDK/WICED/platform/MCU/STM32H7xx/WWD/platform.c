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
 * Defines board support package for BCM943340WCD1 board
 */
#include "platform.h"
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "wwd_platform_common.h"
//#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "platform_bluetooth.h"
#include "platform_button.h"
#include "gpio_button.h"


/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_gpio_t platform_gpio_pins[] =
{
    [WICED_GPIO_1]  = { GPIOA, 15 },
    [WICED_GPIO_2]  = { GPIOB,  3 },
    [WICED_GPIO_3]  = { GPIOB,  4 },
    [WICED_GPIO_4]  = { GPIOG, 14 },
    [WICED_GPIO_5]  = { GPIOG,  9 },
    [WICED_GPIO_6]  = { GPIOG, 12 },
    [WICED_GPIO_7]  = { GPIOG, 13 },
    [WICED_GPIO_8]  = { GPIOB,  5 },
    [WICED_GPIO_9]  = { GPIOB,  6 },
    [WICED_GPIO_10] = { GPIOA,  1 },
    [WICED_GPIO_11] = { GPIOA,  2 },
    [WICED_GPIO_12] = { GPIOA,  3 },
    [WICED_GPIO_13] = { GPIOD,  5 },
    [WICED_GPIO_14] = { GPIOD,  6 },
    [WICED_GPIO_15] = { GPIOD,  3 },
    [WICED_GPIO_16] = { GPIOD,  4 },
    [WICED_GPIO_17] = { GPIOA,  6 },
    [WICED_GPIO_18] = { GPIOA,  7 },
    [WICED_GPIO_19] = { GPIOA,  5 },
    [WICED_GPIO_20] = { GPIOA,  0 },
    [WICED_GPIO_21] = { GPIOA,  4 },
    [WICED_GPIO_22] = { GPIOB, 10 },
    [WICED_GPIO_23] = { GPIOB,  9 },
    [WICED_GPIO_24] = { GPIOC,  3 },
    [WICED_GPIO_25] = { GPIOC,  6 },
    [WICED_GPIO_26] = { GPIOH,  5 }, /* Or GPIOH6 if using SMBus */
    [WICED_GPIO_27] = { GPIOH,  4 },
 };


//const platform_gpio_t wifi_control_pins[] =
//{
//    /* Reset pin unavailable */
////    [WWD_PIN_POWER      ] = { GPIOB,  13 },
////    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
////    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  1 },
////    [WWD_PIN_BOOTSTRAP_1] = { GPIOC, 10 },
//	  [WWD_PIN_RESET      ] = { GPIOB,  13 },
////    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
////    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  1 },
////    [WWD_PIN_BOOTSTRAP_1] = { GPIOC, 10 },
//};

const platform_gpio_t wifi_control_pins[] =
{
	  [WWD_PIN_RESET      ] = { GPIOC,  2 },
};

const platform_gpio_t wifi_sdio_pins[] =
{

	  [WWD_PIN_SDIO_OOB_IRQ] = { GPIOI, 11},
		
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },//
    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },//
		
    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },//
    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },//
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },//
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },//
};


/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{


	NVIC_SetPriority( SDMMC1_IRQn        ,  6 ); /* WLAN SDIO           */
//	NVIC_SetPriority( DMA2_Stream3_IRQn,  6 ); /* WLAN SDIO DMA       */

//	NVIC_SetPriority( SDIO_IRQn        ,  5 ); /* WLAN SDIO           */
	NVIC_SetPriority( DMA2_Stream3_IRQn,  6 ); /* WLAN SDIO DMA       */

	
//	NVIC_SetPriority( DMA2_Stream6_IRQn,  3 ); /* WLAN SPI DMA        */ //WLAN用的是SPI接口驱动方式
//	NVIC_SetPriority( USART1_IRQn      ,  6 ); /* MICO_UART_1         */
//	NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* MICO_UART_1 TX DMA  */
//	NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* MICO_UART_1 RX DMA  */
//	NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
//	NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
	
}


/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/
#define WWD_RTOS_DEFINE_ISR( function ) \
        void function( void ); \
        __attribute__(( used )) void function( void )  
					

/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

/* These DMA assignments can be found STM32F4xx datasheet DMA section */
#define WWD_RTOS_MAP_ISR(  function, irq_handler  ) \
        extern void irq_handler( void ); \
        __attribute__(( alias( #function ))) void irq_handler ( void );


