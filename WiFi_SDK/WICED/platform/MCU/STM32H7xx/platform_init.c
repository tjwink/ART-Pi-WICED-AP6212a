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
 * Define default STM32H7xx initialization functions
 */
#include "stdio.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_config.h"
#include "platform_toolchain.h"
#include "platform/wwd_platform_interface.h"

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
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void error_handler(void)
{
    /* User may add here some code to deal with this error */
    while(1)
    {
    }
}
static int32_t       stm32f2_clock_needed_counter = 0;
#define WICED_ENABLE_INTERRUPTS()   __asm("CPSIE i")  /**< Enable interrupts to start task switching in MICO RTOS. */
#define WICED_DISABLE_INTERRUPTS()  __asm("CPSID i")  /**< Disable interrupts to stop task switching in MICO RTOS. */

platform_result_t platform_mcu_powersave_disable( void )
{
#ifndef WICED_DISABLE_MCU_POWERSAVE

    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

platform_result_t platform_mcu_powersave_enable( void )
{
#ifndef WICED_DISABLE_MCU_POWERSAVE
    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            TODO : Fill the table below
  *            System Clock source            =
  *            SYSCLK(Hz)                     =
  *            HCLK(Hz)                       =
  *            AHB Prescaler                  =
  *            APB1 Prescaler                 =
  *            APB2 Prescaler                 =
  *            HSE Frequency(Hz)              =
  *            PLL_M                          =
  *            PLL_N                          =
  *            PLL_P                          =
  *            PLL_Q                          =
  *            VDD(V)                         =
  *            Main regulator output voltage  =
  *            Flash Latency(WS)              =
  * @param  None
  * @retval None
  */
static void system_clock_config(void)
{
    RCC_ClkInitTypeDef rcc_clock_init_struct;
    RCC_OscInitTypeDef rcc_oscillator_init_struct;
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
    uint32_t flash_latency;

    /* Enable use of Floating point instructions */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif

    /*!< Supply configuration update enable */
    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {}

    HAL_RCC_GetOscConfig(&rcc_oscillator_init_struct);

    if(rcc_oscillator_init_struct.HSEState != RCC_HSE_ON || rcc_oscillator_init_struct.PLL.PLLSource != RCC_PLLSOURCE_HSE)
    {
        /* Enable HSE Oscillator and activate PLL with HSE as source */
        rcc_oscillator_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        rcc_oscillator_init_struct.HSEState       = RCC_HSE_ON;
        rcc_oscillator_init_struct.LSIState       = RCC_LSI_ON;
        rcc_oscillator_init_struct.LSEState       = RCC_LSE_ON;
        rcc_oscillator_init_struct.HSIState       = RCC_HSI_DIV1;
        rcc_oscillator_init_struct.PLL.PLLState   = RCC_PLL_ON;
        rcc_oscillator_init_struct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
        rcc_oscillator_init_struct.PLL.PLLM       = 5;
        rcc_oscillator_init_struct.PLL.PLLN       = 160;
        rcc_oscillator_init_struct.PLL.PLLP       = 2;
        rcc_oscillator_init_struct.PLL.PLLR       = 2;
        rcc_oscillator_init_struct.PLL.PLLQ       = 4;
        rcc_oscillator_init_struct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;
        rcc_oscillator_init_struct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
        rcc_oscillator_init_struct.PLL.PLLFRACN   = 0;
        rcc_oscillator_init_struct.HSICalibrationValue = 16;

        if(HAL_RCC_OscConfig(&rcc_oscillator_init_struct) != HAL_OK)
        {
            /* Initialization Error */
            error_handler();
        }
    }

    HAL_RCC_GetClockConfig(&rcc_clock_init_struct, &flash_latency);
    if(rcc_clock_init_struct.SYSCLKSource   != RCC_SYSCLKSOURCE_PLLCLK)
    {
        /**Initializes the CPU, AHB and APB busses clocks
        */
        rcc_clock_init_struct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                    |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;

        rcc_clock_init_struct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
        rcc_clock_init_struct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
        rcc_clock_init_struct.AHBCLKDivider  = RCC_HCLK_DIV2;
        rcc_clock_init_struct.APB3CLKDivider = RCC_APB3_DIV2;
        rcc_clock_init_struct.APB1CLKDivider = RCC_APB1_DIV2;
        rcc_clock_init_struct.APB2CLKDivider = RCC_APB2_DIV2;
        rcc_clock_init_struct.APB4CLKDivider = RCC_APB4_DIV2;

        if(HAL_RCC_ClockConfig(&rcc_clock_init_struct, FLASH_LATENCY_4) != HAL_OK)
        {
            /* Initialization Error */
            error_handler();
        }

        RCC_PeriphClkInit.PeriphClockSelection      = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2;
        RCC_PeriphClkInit.Usart16ClockSelection     = RCC_USART16CLKSOURCE_HSI;
        RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_HSI;
        HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
    }
    else
    {
        /* HAL_RCC_ClockConfig() is not called, call below two functions to update
         * STM32 HAL Global variables.
         */

        /* Update SystemCoreClock in STM32 HAL */
        SystemCoreClockUpdate();

        /* Update SystemD2Clock in STM32 HAL */
        HAL_RCC_GetHCLKFreq();
    }

    /* Configure the Systick interrupt time */
    HAL_SYSTICK_Config(SystemCoreClock/1000);

    /* Configure the Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* define MEASURE_CLOCK_FREQUENCY to measure clock frequency
     * on RCC_MCO1 and RCC_MCO2 pins.
     */
#ifdef MEASURE_CLOCK_FREQUENCY
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
    HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_10);
#endif
}
/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/* STM32F2 common clock initialization function
 * This brings up enough clocks to allow the processor to run quickly while initializing memory.
 * Other platform specific clock init can be done in init_platform() or init_architecture()
 */
WEAK void platform_init_system_clocks( void )
{
#ifndef BOOTLOADER
    /* Initialize watchdog */
//    platform_watchdog_init( );
#endif
    system_clock_config();
}


WEAK void platform_init_memory( void )
{

}

void platform_init_connectivity_module( void )
{
    host_platform_init( );
}

WEAK void platform_init_external_devices( void )
{

}

void rcc_apb2_clock_enable(uint32_t rcc_apb2_peripheral, FunctionalState new_state)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(new_state));

    if (new_state != DISABLE)
    {
        RCC->APB2ENR |= rcc_apb2_peripheral;
    }
    else
    {
        RCC->APB2ENR &= ~rcc_apb2_peripheral;
    }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as WT for D3 SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


void SD_LowLevel_Init(void)
{
		int i;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能 SDMMC 时钟 */
    __HAL_RCC_SDMMC1_CLK_ENABLE();
  
    /* 使能 GPIOs 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚为不需要上、下拉模式*/  
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /*设置引脚速率为高速 */      
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    /*设置为SDIO1复用 */  
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/  
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    /*设置引脚的输出类型为推挽输出*/    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚为不需要上、下拉模式*/  
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /*设置引脚速率为高速 */ 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    /*设置为SDIO1复用 */  
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/ 
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    //启用WIFI模块

    /*使能引脚时钟*/	
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /*选择要控制的GPIO引脚*/															   
    GPIO_InitStruct.Pin = GPIO_PIN_2;	
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      
    /*设置引脚为上拉模式*/
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    /*设置引脚速率为高速 */   
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);	
    /*禁用WiFi模块*/
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);  
		for(i=0;i<0xffff;i++)
		{
			__nop();
		}
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);  

}



void platform_init_mcu_infrastructure( void )
{
    uint8_t i;

#ifdef INTERRUPT_VECTORS_IN_RAM
    SCB->VTOR = 0x24000000; /* Change the vector table to point to start of SRAM */
#endif /* ifdef INTERRUPT_VECTORS_IN_RAM */

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    MPU_Config();

    /* Enable I-Cache */
//  SCB_EnableICache();

    /* Enable D-Cache */
//    SCB_EnableDCache();

    /* 初始化中断优先级*/
    for ( i = 0; i < 100; i++ )
    {
        HAL_NVIC_SetPriority( i, 0xf, 0);
    }
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    platform_init_rtos_irq_priorities();
    platform_init_peripheral_irq_priorities();

    /*初始化GPIO IRQ管理器 */
//    platform_gpio_irq_manager_init();


#ifndef WICED_DISABLE_MCU_POWERSAVE
    /* Initialize MCU powersave */
//    platform_mcu_powersave_init( );
//    platform_mcu_powersave_disable( );

    /* Initialize RTC */
//    platform_rtc_init( );
#endif /* ifndef WICED_DISABLE_MCU_POWERSAVE */
}

void platform_mcu_reset( void )
{
    NVIC_SystemReset( );

    /* Loop forever */
    while ( 1 )
    {
    }
}
