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

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port               = USART6,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_4],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_5],
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream6,
            .channel        = DMA_Channel_5,
            .irq_vector     = DMA2_Stream6_IRQn,
            .complete_flags = DMA_HISR_TCIF6,
            .error_flags    = ( DMA_HISR_TEIF6 | DMA_HISR_FEIF6 ),
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream1,
            .channel        = DMA_Channel_5,
            .irq_vector     = DMA2_Stream1_IRQn,
            .complete_flags = DMA_LISR_TCIF1,
            .error_flags    = ( DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 ),
        },
    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};
#endif

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c
 * SDIO: WWD_PIN_BOOTSTRAP[1:0] = b'X0
 * gSPI: WWD_PIN_BOOTSTRAP[1:0] = b'01
 */
/*

    WWD_PIN_POWER,
    WWD_PIN_RESET,
    WWD_PIN_32K_CLK,
    WWD_PIN_BOOTSTRAP_0,
    WWD_PIN_BOOTSTRAP_1,
    WWD_PIN_CONTROL_MAX,
*/
const platform_gpio_t wifi_control_pins[] =
{
    /* Reset pin unavailable */
//    [WWD_PIN_POWER      ] = { GPIOB,  13 },
//    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
//    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  1 },
//    [WWD_PIN_BOOTSTRAP_1] = { GPIOC, 10 },
	  [WWD_PIN_RESET      ] = { GPIOB,  13 },
//    [WWD_PIN_32K_CLK    ] = { GPIOA,  8 },
//    [WWD_PIN_BOOTSTRAP_0] = { GPIOB,  1 },
//    [WWD_PIN_BOOTSTRAP_1] = { GPIOC, 10 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
//    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOB, 11 },
//    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },//
//    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },//
//    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },//
//    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },//
//    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },//
//    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },//
	  [WWD_PIN_SDIO_OOB_IRQ] = { GPIOA, 0},
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },//
    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },//
    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },//
    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },//
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },//
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },//
};

/* Wi-Fi gSPI bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SPI.c */
const platform_gpio_t wifi_spi_pins[] =
{
    [WWD_PIN_SPI_IRQ ] = { GPIOC,  9 },
    [WWD_PIN_SPI_CS  ] = { GPIOC, 11 },
    [WWD_PIN_SPI_CLK ] = { GPIOB, 13 },
    [WWD_PIN_SPI_MOSI] = { GPIOB, 15 },
    [WWD_PIN_SPI_MISO] = { GPIOB, 14 },
};

/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = { GPIOF,  8 },
    [WICED_BT_PIN_HOST_WAKE  ] = { GPIOB,  0 },
    [WICED_BT_PIN_DEVICE_WAKE] = { GPIOF,  7 }
};
const platform_gpio_t* wiced_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = &internal_bt_control_pins[WICED_BT_PIN_POWER      ],
    [WICED_BT_PIN_HOST_WAKE  ] = &internal_bt_control_pins[WICED_BT_PIN_HOST_WAKE  ],
    [WICED_BT_PIN_DEVICE_WAKE] = &internal_bt_control_pins[WICED_BT_PIN_DEVICE_WAKE],
    [WICED_BT_PIN_RESET]       = NULL,
};

/* Bluetooth UART pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = { GPIOA,  9 },
    [WICED_BT_PIN_UART_RX ] = { GPIOA, 10 },
    [WICED_BT_PIN_UART_CTS] = { GPIOA, 11 },
    [WICED_BT_PIN_UART_RTS] = { GPIOA, 12 },
};
const platform_gpio_t* wiced_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    [WICED_BT_PIN_UART_RX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    [WICED_BT_PIN_UART_CTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    [WICED_BT_PIN_UART_RTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
};

/* Bluetooth UART peripheral and runtime driver. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_uart_t internal_bt_uart_peripheral =
{
    .port               = USART1,
    .tx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    .rx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    .cts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    .rts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
    .tx_dma_config =
    {
        .controller     = DMA2,
        .stream         = DMA2_Stream7,
        .channel        = DMA_Channel_4,
        .irq_vector     = DMA2_Stream7_IRQn,
        .complete_flags = DMA_HISR_TCIF7,
        .error_flags    = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 ),
    },
    .rx_dma_config =
    {
        .controller     = DMA2,
        .stream         = DMA2_Stream2,
        .channel        = DMA_Channel_4,
        .irq_vector     = DMA2_Stream2_IRQn,
        .complete_flags = DMA_LISR_TCIF2,
        .error_flags    = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
    },
};
static platform_uart_driver_t internal_bt_uart_driver;
const platform_uart_t*        wiced_bt_uart_peripheral = &internal_bt_uart_peripheral;
platform_uart_driver_t*       wiced_bt_uart_driver     = &internal_bt_uart_driver;

/* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_config_t wiced_bt_uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};

/*BT chip specific configuration information*/
const platform_bluetooth_config_t wiced_bt_config =
{
    .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
    .patchram_download_baud_rate = 115200,
    .featured_baud_rate          = 115200
};

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON2,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
};

const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
{
     [WICED_LED_INDEX_1] = WICED_LED1,
     [WICED_LED_INDEX_2] = WICED_LED2,
};

/* I2C peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_i2c_t platform_i2c_peripherals[] =
{
    [WICED_I2C_1] =
    {
        .port                    = I2C2,
        .pin_scl                 = &platform_gpio_pins[WICED_GPIO_27],
        .pin_sda                 = &platform_gpio_pins[WICED_GPIO_26],
        .peripheral_clock_reg    = RCC_APB1Periph_I2C2,
        .tx_dma                  = DMA1,
        .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
        .tx_dma_stream           = DMA1_Stream7,
        .rx_dma_stream           = DMA1_Stream0,
        .tx_dma_stream_id        = 7,
        .rx_dma_stream_id        = 0,
        .tx_dma_channel          = DMA_Channel_1,
        .rx_dma_channel          = DMA_Channel_1,
        .gpio_af                 = GPIO_AF_I2C2
    },
    [WICED_I2C_2] =
    {
        .port                    = I2C1,
        .pin_scl                 = 0,
        .pin_sda                 = 0,
        .peripheral_clock_reg    = RCC_APB1Periph_I2C1,
        .tx_dma                  = DMA1,
        .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
        .tx_dma_stream           = DMA1_Stream7,
        .rx_dma_stream           = DMA1_Stream0,
        .tx_dma_stream_id        = 7,
        .rx_dma_stream_id        = 0,
        .tx_dma_channel          = DMA_Channel_1,
        .rx_dma_channel          = DMA_Channel_1,
        .gpio_af                 = GPIO_AF_I2C1
    }
};

/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] = {ADC1, ADC_Channel_1, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_10]},
    [WICED_ADC_2] = {ADC1, ADC_Channel_2, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_11]},
    [WICED_ADC_3] = {ADC1, ADC_Channel_3, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_12]},
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {TIM4, 1, RCC_APB1Periph_TIM4, GPIO_AF_TIM4, &platform_gpio_pins[WICED_GPIO_9]},
    [WICED_PWM_2]  = {TIM3, 2, RCC_APB1Periph_TIM3, GPIO_AF_TIM3, &platform_gpio_pins[WICED_GPIO_8] },
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
////    /* Interrupt priority setup. Called by WICED/platform/MCU/STM32F4xx/platform_init.c */
////    NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
//    NVIC_SetPriority( SDIO_IRQn        ,  2 ); /* WLAN SDIO           */
//    NVIC_SetPriority( DMA2_Stream3_IRQn,  3 ); /* WLAN SDIO DMA       */
////    NVIC_SetPriority( DMA1_Stream3_IRQn,  3 ); /* WLAN gSPI DMA       */
////    NVIC_SetPriority( USART6_IRQn      ,  6 ); /* WICED_UART_1        */
////    NVIC_SetPriority( USART1_IRQn      ,  6 ); /* Bluetooth UART      */
////    NVIC_SetPriority( DMA2_Stream6_IRQn,  7 ); /* WICED_UART_1 TX DMA */
////    NVIC_SetPriority( DMA2_Stream1_IRQn,  7 ); /* WICED_UART_1 RX DMA */
////    NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* Bluetooth TX DMA    */
////    NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* Bluetooth RX DMA    */
////    NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
////    NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
	
		/* Interrupt priority setup. Called by MiCO/platform/MCU/STM32F2xx/platform_init.c */
//	NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
	NVIC_SetPriority( SDIO_IRQn        ,  5 ); /* WLAN SDIO           */
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

/* LEDs on this platform are active HIGH */
platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
    {
        switch (off_on)
        {
            case WICED_LED_OFF:
                platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
            case WICED_LED_ON:
                platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
        }
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
}

/* Initialize LEDs and turn off */
void platform_init_leds( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );

    platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);
    platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);
}

void platform_init_external_devices( void )
{
    platform_init_leds();

    /* Initialise buttons to input by default */
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_PULL_UP );
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON2], INPUT_PULL_UP );

#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time )
{
    int             button_gpio;
    uint32_t        button_press_timer = 0;
    int             led_state = 0;

    /* Initialize input */
    button_gpio     = platform_gpio_buttons[button_index].gpio;
    platform_gpio_init( &platform_gpio_pins[ button_gpio ], INPUT_PULL_UP );

    while ( (PLATFORM_BUTTON_PRESSED_STATE == platform_gpio_input_get(&platform_gpio_pins[ button_gpio ])) )
    {
        /* wait a bit */
        host_rtos_delay_milliseconds( PLATFORM_BUTTON_PRESS_CHECK_PERIOD );

        /* Toggle LED */
        platform_led_set_state(led_index, (led_state == 0) ? WICED_LED_OFF : WICED_LED_ON);
        led_state ^= 0x01;

        /* keep track of time */
        button_press_timer += PLATFORM_BUTTON_PRESS_CHECK_PERIOD;
        if ((max_time > 0) && (button_press_timer >= max_time))
        {
            break;
        }
    }

     /* turn off the LED */
    platform_led_set_state(led_index, WICED_LED_OFF );

    return button_press_timer;
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    return platform_get_button_press_time ( PLATFORM_FACTORY_RESET_BUTTON_INDEX, PLATFORM_RED_LED_INDEX, max_time );
}

/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/
#define WWD_RTOS_DEFINE_ISR( function ) \
        void function( void ); \
        __attribute__(( used )) void function( void )  
					
//WWD_RTOS_DEFINE_ISR( usart6_irq )
//{
//    platform_uart_irq( &platform_uart_drivers[WICED_UART_1] );
//}

//WWD_RTOS_DEFINE_ISR( usart6_tx_dma_irq )
//{
//    platform_uart_tx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
//}

//WWD_RTOS_DEFINE_ISR( usart6_rx_dma_irq )
//{
//    platform_uart_rx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
//}

//WWD_RTOS_DEFINE_ISR( bt_uart_irq )
//{
//    platform_uart_irq( wiced_bt_uart_driver );
//}

//WWD_RTOS_DEFINE_ISR( bt_uart_tx_dma_irq )
//{
//    platform_uart_tx_dma_irq( wiced_bt_uart_driver );
//}

//WWD_RTOS_DEFINE_ISR( bt_uart_rx_dma_irq )
//{
//    platform_uart_rx_dma_irq( wiced_bt_uart_driver );
//}

/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

/* These DMA assignments can be found STM32F4xx datasheet DMA section */
#define WWD_RTOS_MAP_ISR(  function, irq_handler  ) \
        extern void irq_handler( void ); \
        __attribute__(( alias( #function ))) void irq_handler ( void );

//WWD_RTOS_MAP_ISR( usart6_irq         , USART6_irq       )
//WWD_RTOS_MAP_ISR( usart6_tx_dma_irq  , DMA2_Stream6_irq )
//WWD_RTOS_MAP_ISR( usart6_rx_dma_irq  , DMA2_Stream1_irq )
//WWD_RTOS_MAP_ISR( bt_uart_irq        , USART1_irq       )
//WWD_RTOS_MAP_ISR( bt_uart_tx_dma_irq , DMA2_Stream7_irq )
//WWD_RTOS_MAP_ISR( bt_uart_rx_dma_irq , DMA2_Stream2_irq )
