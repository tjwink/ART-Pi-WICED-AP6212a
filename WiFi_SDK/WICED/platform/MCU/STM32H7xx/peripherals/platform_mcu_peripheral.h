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
 * Defines STM32H7xx common peripheral structures, macros, constants and declares STM32H7xx peripheral API
 */
#pragma once
#include "platform_cmsis.h"
#include "platform_constants.h"

#include "RTOS/wwd_rtos_interface.h"
#include "ring_buffer.h"

#include "stm32h7xx.h"
//#include "stm32h753xx.h"
#include "stm32h750xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_i2c_ex.h"
#include "stm32h7xx_hal_rtc.h"
#include "stm32h7xx_hal_rtc_ex.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_tim_ex.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio_ex.h"
#include "stm32h7xx_hal_iwdg.h"


#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LSI_FREQ_HZ               (32000)
#define LSE_FREQ_HZ               (32768)

/* GPIOA to K */
#define NUMBER_OF_GPIO_PORTS      (11)

/* Interrupt line 0 to 15. Each line is shared among the same numbered pins across all GPIO ports */
#define NUMBER_OF_GPIO_IRQ_LINES  (16)

/* USART1 to 8 */
#define NUMBER_OF_UART_PORTS      (8)

/* Invalid UART port number */
#define INVALID_UART_PORT_NUMBER  (0xff)

/* Invalid GPIO port number */
#define INVALID_GPIO_PORT_NUMBER  (0xff)

/* Invalid I2C port number */
#define INVALID_I2C_PORT_NUMBER   (0xff)

/* Invalid PWM port number */
#define INVALID_PWM_PORT_NUMBER   (0xff)

/* WatchDog configuration */
#define WATCHDOG_INSTANCE         (IWDG1)
#define WATCHDOG_PRESCALER        (IWDG_PRESCALER_256)
#define WATCHDOG_RESET_FLAG       (RCC_FLAG_IWDG1RST)
#define PRESCALAR                 (256)

/* RTC configuration */
#define RTC_CLOCK_SOURCE          (RCC_RTCCLKSOURCE_LSI)
#define RTC_PRESCALER_A           (127)
#define RTC_PRESCALER_S           (249)

/* Default STDIO buffer size */
#ifndef STDIO_BUFFER_SIZE
#define STDIO_BUFFER_SIZE         (1024)
#endif

/* SPI1 to SPI6 */
#define NUMBER_OF_SPI_PORTS       (6)

/* I2C1 to I2C4 */
#define NUMBER_OF_I2C_PORTS       (4)

/* TIM1, TIM2, TIM3, TIM4, TIM5, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17 */
#define NUMBER_OF_PWM_PORTS       (12)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* GPIO port */
typedef GPIO_TypeDef  platform_gpio_port_t;

/* UART port */
typedef USART_TypeDef platform_uart_port_t;

/* SPI port */
typedef SPI_TypeDef   platform_spi_port_t;

/* I2C port */
typedef I2C_TypeDef   platform_i2c_port_t;

/* PWM port */
typedef TIM_TypeDef   platform_pwm_port_t;

/* GPIO alternate function */
typedef uint8_t       platform_gpio_alternate_function_t;

/* Peripheral clock function */
typedef void (*platform_peripheral_clock_function_t)(uint32_t clock, FunctionalState state );

typedef DMA_TypeDef     dma_registers_t;
typedef FunctionalState functional_state_t;
typedef uint32_t        peripheral_clock_t;
typedef IRQn_Type       irq_vector_t;

typedef void (*platform_uart_clock_enable_function_t)(void);
typedef void (*platform_uart_clock_disable_function_t)(void);

typedef void (*platform_gpio_clock_enable_function_t)(void);
typedef void (*platform_gpio_clock_disable_function_t)(void);

typedef void (*platform_i2c_clock_enable_function_t)(void);
typedef void (*platform_i2c_clock_disable_function_t)(void);

typedef void (*platform_pwm_clock_enable_function_t)(void);
typedef void (*platform_pwm_clock_disable_function_t)(void);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    DMA_TypeDef*        controller;
    DMA_Stream_TypeDef* stream;
    uint32_t            channel;
    IRQn_Type           irq_vector;
    uint32_t            complete_flags;
    uint32_t            error_flags;
} platform_dma_config_t;

typedef struct
{
    platform_gpio_port_t* port;       /* GPIO port. platform_gpio_port_t is defined in <WICED-SDK>/MCU/<MCU>/platform_mcu_interface.h */
    uint8_t               pin_number; /* pin number. Valid range is defined in <WICED-SDK>/MCU/<MCU>/platform_mcu_interface.h         */
} platform_gpio_t;

typedef struct
{
    ADC_TypeDef*           port;
    uint8_t                channel;
    uint32_t               adc_peripheral_clock;
    uint8_t                rank;
    const platform_gpio_t* pin;
} platform_adc_t;

typedef struct
{
    platform_pwm_port_t*   port;
    uint32_t               channel;
    const platform_gpio_t* pin;
} platform_pwm_t;

/* DMA can be enabled by setting SPI_USE_DMA */
typedef struct
{
    platform_spi_port_t*                 port;
    uint8_t                              gpio_af;
    uint32_t                             peripheral_clock_reg;
    platform_peripheral_clock_function_t peripheral_clock_func;
    const platform_gpio_t*               pin_mosi;
    const platform_gpio_t*               pin_miso;
    const platform_gpio_t*               pin_clock;
    platform_dma_config_t                tx_dma;
    platform_dma_config_t                rx_dma;
} platform_spi_t;

typedef struct
{
    uint8_t unimplemented;
} platform_spi_slave_driver_t;

typedef struct
{
    platform_i2c_port_t*   port;
    const platform_gpio_t* pin_scl;
    const platform_gpio_t* pin_sda;
} platform_i2c_t;

typedef struct
{
    platform_uart_port_t*  port;
    const platform_gpio_t* tx_pin;
    const platform_gpio_t* rx_pin;
    const platform_gpio_t* cts_pin;
    const platform_gpio_t* rts_pin;
    platform_dma_config_t  tx_dma_config;
    platform_dma_config_t  rx_dma_config;
} platform_uart_t;

typedef struct
{
    platform_uart_t*           peripheral;
    wiced_ring_buffer_t*       rx_buffer;
    host_semaphore_type_t      rx_complete;
    host_semaphore_type_t      tx_complete;
    volatile uint32_t          tx_size;
    volatile uint32_t          rx_size;
    volatile platform_result_t last_receive_result;
    volatile platform_result_t last_transmit_result;
} platform_uart_driver_t;

typedef struct
{
    DMA_TypeDef*                         dma_register;
    DMA_Stream_TypeDef*                  stream;
    uint32_t                             channel;
    peripheral_clock_t                   peripheral_clock;
    platform_peripheral_clock_function_t peripheral_clock_func;
    irq_vector_t                         irq;
} platform_dma_t;

typedef struct
{
    SPI_TypeDef*                         spi;
    uint8_t                              gpio_af;
    unsigned                             is_master   : 1;
    unsigned                             enable_mclk : 1;
    peripheral_clock_t                   peripheral_clock;
    platform_peripheral_clock_function_t peripheral_clock_func;
    const platform_gpio_t*               pin_ck;
    const platform_gpio_t*               pin_sd;
    const platform_gpio_t*               pin_ws;
    const platform_gpio_t*               pin_mclk;
    platform_dma_t                       tx_dma;
    platform_dma_t                       rx_dma;
} platform_i2s_t;

typedef enum
{
    WICED_QSPI_PIN_CS,
    WICED_QSPI_PIN_CLK,
    WICED_QSPI_PIN_D0,
    WICED_QSPI_PIN_D1,
    WICED_QSPI_PIN_D2,
    WICED_QSPI_PIN_D3,
    WICED_QSPI_PIN_MAX,
} wiced_qspi_pin_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

void rcc_apb2_clock_enable(uint32_t spi_clk_value, FunctionalState state);
void rcc_apb1_clock_enable(uint32_t spi_clk_value, FunctionalState state);

platform_result_t platform_gpio_irq_manager_init      ( void );
uint8_t           platform_gpio_get_port_number       ( platform_gpio_port_t* gpio_port );
extern platform_result_t platform_gpio_set_alternate_function
																											( 
																												platform_gpio_port_t* gpio_port, 
																												uint8_t pin_number, 
																												uint32_t output_type, 
																												uint32_t pull_up_down_type, 
																												uint8_t alternation_function );
//platform_result_t platform_watchdog_init              ( void );
//platform_result_t platform_mcu_powersave_init         ( void );

//platform_result_t platform_rtc_init                   ( void );
platform_result_t platform_rtc_enter_powersave        ( void );
platform_result_t platform_rtc_abort_powersave        ( void );
platform_result_t platform_rtc_exit_powersave         ( uint32_t requested_sleep_time, uint32_t *cpu_sleep_time );

uint8_t           platform_uart_get_port_number       ( platform_uart_port_t* uart );
void              platform_uart_irq                   ( platform_uart_driver_t* driver );
void              platform_uart_tx_dma_irq            ( platform_uart_driver_t* driver );
void              platform_uart_rx_dma_irq            ( platform_uart_driver_t* driver );

uint8_t           platform_i2c_get_port_number        ( platform_i2c_port_t* i2c_port );
uint32_t          platform_i2c_get_port_timing        ( platform_i2c_port_t* i2c_port, uint32_t bus_freq );

uint8_t           platform_pwm_get_port_number        ( platform_pwm_port_t* pwm_port );
platform_result_t platform_pwm_get_divider            ( platform_pwm_port_t* pwm_port, uint32_t frequency, uint32_t* div_period, uint32_t* div_prescaler );

void              platform_i2s_irq                    ( uint32_t i2s );
void              platform_i2s_tx_dma_irq             ( uint32_t i2s );

platform_result_t platform_filesystem_init            ( void );
uint8_t           platform_spi_get_port_number        ( platform_spi_port_t* spi );

void              platform_gpio_irq                   ( void );

#ifdef __cplusplus
} /* extern "C" */
#endif
