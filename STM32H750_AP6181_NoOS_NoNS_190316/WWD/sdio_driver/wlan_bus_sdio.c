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
 * Defines WWD SDIO functions for STM32H7xx MCU
 */
#include "stm32h7xx.h"
#include <string.h> /* For memcpy */
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_constants.h"
#include "wwd_rtos_isr.h"

/******************************************************
 *             Constants
 ******************************************************/

#define COMMAND_FINISHED_CMD52_TIMEOUT_LOOPS (100000)
#define COMMAND_FINISHED_CMD53_TIMEOUT_LOOPS (100000)
#define SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS    (100000)
#define SDIO_DMA_TIMEOUT_LOOPS               (1000000)
#define MAX_TIMEOUTS                         (30)
#define SDIO_ERROR_MASK                      ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_CTIMEOUT | SDMMC_STA_DTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR )
#define SDIO_IRQ_CHANNEL                     (0x31)
#define BUS_LEVEL_MAX_RETRIES                (5)
#define SDIO_ENUMERATION_TIMEOUT_MS          (500)

/******************************************************
 *             Structures
 ******************************************************/

typedef struct
{
    /*@shared@*//*@null@*/uint8_t* data;
    uint16_t length;
} sdio_dma_segment_t;

/******************************************************
 *             Variables
 ******************************************************/

static const uint32_t bus_direction_mapping[ ] =
{
    [BUS_READ] = SDMMC_TRANSFER_DIR_TO_SDMMC,
    [BUS_WRITE] = SDMMC_TRANSFER_DIR_TO_CARD
};

__align(4) static uint8_t temp_dma_buffer[ MAX( 2 * 1024, WICED_LINK_MTU+ 64 ) ];
static uint8_t*               user_data;
static uint32_t               user_data_size;
static uint8_t*               dma_data_source;
static uint32_t               dma_transfer_size;
static host_semaphore_type_t  sdio_transfer_finished_semaphore;
static wiced_bool_t           sdio_transfer_failed;

/******************************************************
 *             Static Function Declarations
 ******************************************************/

static uint32_t sdio_get_blocksize_dctrl( sdio_block_size_t block_size );
static sdio_block_size_t find_optimal_block_size( uint32_t data_size );
static void sdio_prepare_data_transfer( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/;

/******************************************************
 *             Function definitions
 ******************************************************/

#ifndef  WICED_DISABLE_MCU_POWERSAVE
static void sdio_oob_irq_handler( void* arg )
{
    UNUSED_PARAMETER( arg );
    WWD_BUS_STATS_INCREMENT_VARIABLE( oob_intrs );
    platform_mcu_powersave_exit_notify( );
    wwd_thread_notify_irq( );
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

static void sdio_enable_bus_irq( void )
{
    SDMMC1->MASK |= SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR | SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND;
}

static void sdio_disable_bus_irq( void )
{
    SDMMC1->MASK = 0;
}

#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt( void )
{
    /* Set GPIO_B[1:0] to input. One of them will be re-purposed as OOB interrupt */
    platform_gpio_init( &wifi_sdio_pins[ WWD_PIN_SDIO_OOB_IRQ ], INPUT_HIGH_IMPEDANCE );
    platform_gpio_irq_enable( &wifi_sdio_pins[ WWD_PIN_SDIO_OOB_IRQ ], IRQ_TRIGGER_RISING_EDGE, sdio_oob_irq_handler, 0 );
    return WWD_SUCCESS;
}

uint8_t host_platform_get_oob_interrupt_pin( void )
{
    return WICED_WIFI_OOB_IRQ_GPIO_PIN;
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
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

wwd_result_t host_platform_bus_init( void )
{
    SDMMC_InitTypeDef sdio_init_structure;
    wwd_result_t result;

    HAL_StatusTypeDef HAL_return;

//    platform_mcu_powersave_disable( );

    /* Reset SDIO Block */
    SDMMC_PowerState_OFF( SDMMC1 );
    __HAL_RCC_SDMMC1_FORCE_RESET( );
    __HAL_RCC_SDMMC1_RELEASE_RESET( );

    /* Enable the SDIO Clock */
    __HAL_RCC_SDMMC1_CLK_ENABLE( );

    result = host_rtos_init_semaphore( &sdio_transfer_finished_semaphore );
    if ( result != WWD_SUCCESS )
    {
        return result;
    }

    /* Clear all SDIO interrupts */
    SDMMC1->ICR = (uint32_t) 0xffffffff;

    /* Turn on SDIO IRQ */
    /* Must be lower priority than the value of configMAX_SYSCALL_INTERRUPT_PRIORITY */
    /* otherwise FreeRTOS will not be able to mask the interrupt */
    /* keep in mind that ARMCM7 interrupt priority logic is inverted, the highest value */
    /* is the lowest priority */
    HAL_NVIC_EnableIRQ( (IRQn_Type) SDIO_IRQ_CHANNEL );

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
    /* Set GPIO_B[1:0] to 00 to put WLAN module into SDIO mode */
    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0], OUTPUT_PUSH_PULL );
    platform_gpio_output_low( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
#endif
#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1], OUTPUT_PUSH_PULL );
    platform_gpio_output_low( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
#endif

    /* Setup GPIO pins for SDIO data & clock */
    SD_LowLevel_Init();

    /* SDMMC Initialization Frequency (400KHz max) for IP CLK 200MHz*/
    sdio_init_structure.ClockDiv            = SDMMC_INIT_CLK_DIV;
    sdio_init_structure.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    sdio_init_structure.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    sdio_init_structure.BusWide             = SDMMC_BUS_WIDE_1B;
    sdio_init_structure.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    HAL_return                              = SDMMC_Init( SDMMC1, sdio_init_structure );
    HAL_return                             |= SDMMC_PowerState_ON( SDMMC1 );
    HAL_return                             |= SDMMC_SetSDMMCReadWaitMode( SDMMC1, SDMMC_READ_WAIT_MODE_CLK );

    if ( HAL_return |= HAL_OK )
    {
        return (~WWD_SUCCESS);
    }

//    platform_mcu_powersave_enable( );
    return WWD_SUCCESS;
}

wwd_result_t host_platform_sdio_enumerate( void )
{
    wwd_result_t result;

    uint32_t loop_count;
    uint32_t data = 0;

    loop_count = 0;
    do
    {
        /* Send CMD0 to set it to idle state */
        result = host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_0, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );

        /* CMD5. */
        result = host_platform_sdio_transfer( BUS_READ, SDIO_CMD_5, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );

        /* Send CMD3 to get RCA. */
        result = host_platform_sdio_transfer( BUS_READ, SDIO_CMD_3, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, RESPONSE_NEEDED, &data );

        loop_count++;
        if ( loop_count >= (uint32_t) SDIO_ENUMERATION_TIMEOUT_MS )
        {

            return WWD_TIMEOUT;
        }
    } while ( ( result != WWD_SUCCESS ));// && ( host_rtos_delay_milliseconds( (uint32_t) 1 ), ( 1 == 1 ) ) );
    /* If you're stuck here, check the platform matches your hardware */

    /* Send CMD7 with the returned RCA to select the card */
    result = host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_7, SDIO_BYTE_MODE, SDIO_1B_BLOCK, data, 0, 0, RESPONSE_NEEDED, NULL );

    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_deinit( void )
{

    wwd_result_t result = WWD_SUCCESS;

    result = host_rtos_deinit_semaphore( &sdio_transfer_finished_semaphore );

//    platform_mcu_powersave_disable( );

    sdio_disable_bus_irq( );

    SDMMC_PowerState_OFF( SDMMC1 );
    __HAL_RCC_SDMMC1_CLK_DISABLE( );


#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
#endif
#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
#endif

    /* Turn off SDIO IRQ */
    NVIC_DisableIRQ( (IRQn_Type) SDIO_IRQ_CHANNEL );

//    platform_mcu_powersave_enable( );
    return result;
}

wwd_result_t host_platform_sdio_transfer( wwd_bus_transfer_direction_t direction, sdio_command_t command, sdio_transfer_mode_t mode, sdio_block_size_t block_size, uint32_t argument, /*@null@*/uint32_t* data, uint16_t data_size, sdio_response_needed_t response_expected, /*@out@*//*@null@*/uint32_t* response )
{
    uint32_t loop_count = 0;
    wwd_result_t result = WWD_UNSUPPORTED;
    uint16_t attempts = 0;

    wiced_assert( "Bad args", !((command == SDIO_CMD_53) && (data == NULL)) );

    if ( response != NULL )
    {
        *response = 0;
    }

//    platform_mcu_powersave_disable( );

    restart: SDMMC1->ICR = (uint32_t) 0xFFFFFFFF;
    sdio_transfer_failed = WICED_FALSE;
    ++attempts;

    /* Check if we've tried too many times */
    if ( attempts >= (uint16_t) BUS_LEVEL_MAX_RETRIES )
    {
        result = WWD_SDIO_RETRIES_EXCEEDED;
        goto exit;
    }

    /* Prepare the data transfer register */
    if ( command == SDIO_CMD_53 )
    {
        sdio_enable_bus_irq( );

        /* Dodgy STM32 hack to set the CMD53 byte mode size to be the same as the block size */
        if ( mode == SDIO_BYTE_MODE )
        {
            block_size = find_optimal_block_size( data_size );
            if ( block_size < SDIO_512B_BLOCK )
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) ) | block_size;
            }
            else
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) );
            }
        }

        /* Prepare the SDIO for a data transfer */
        sdio_prepare_data_transfer( direction, block_size, (uint8_t*) data, data_size );

        /* Send the command */
        SDMMC1->ARG = argument;
        SDMMC1->CMD = (uint32_t) ( command | SDMMC_RESPONSE_SHORT | SDMMC_WAIT_NO | SDMMC_CPSM_ENABLE | SDMMC_CMD_CMDTRANS );

//        /* Wait for the whole transfer to complete */
        result = host_rtos_get_semaphore( &sdio_transfer_finished_semaphore, (uint32_t) 50, WICED_TRUE );
        if ( result != WWD_SUCCESS )
        {
            goto exit;
        }

        if ( sdio_transfer_failed == WICED_TRUE )
        {
            goto restart;
        }

        /* Check if there were any SDIO errors */
        if ( ( SDMMC1->STA & ( SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT ) ) != 0 )
        {
            goto restart;
        }
        else if ( ( ( SDMMC1->STA & ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR ) ) != 0 ) )
        {
            wiced_assert( "SDIO communication failure", 0 );
            goto restart;
        }

        /* Wait till complete */
        loop_count = (uint32_t) SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS;
        do
        {
            loop_count--;
            if ( loop_count == 0 || ( ( SDMMC1->STA & SDIO_ERROR_MASK ) != 0 ) )
            {

                goto restart;
            }
        } while ( ( SDMMC1->STA & ( SDMMC_STA_CPSMACT | SDMMC_STA_DPSMACT ) ) != 0 );

        if ( direction == BUS_READ )
        {
            memcpy( user_data, dma_data_source, (size_t) user_data_size );
        }

    }
    /* ALl SDIO CMD other than CMD53 */
    else
    {
        uint32_t temp_sta;

        /* Send the command */
        SDMMC1->ARG = argument;
        SDMMC1->CMD = (uint32_t) ( command | SDMMC_RESPONSE_SHORT | SDMMC_WAIT_NO | SDMMC_CPSM_ENABLE );

        loop_count = (uint32_t) COMMAND_FINISHED_CMD52_TIMEOUT_LOOPS;
        do
        {
            temp_sta = SDMMC1->STA;
            loop_count--;
            if ( loop_count == 0 || ( ( response_expected == RESPONSE_NEEDED ) && ( ( temp_sta & SDIO_ERROR_MASK ) != 0 ) ) )
            {
                goto restart;
            }
        } while ( ( temp_sta & ( SDMMC_STA_CPSMACT | SDMMC_STA_DPSMACT ) ) != 0 );
    }

    if ( response != NULL )
    {
        *response = SDMMC1->RESP1;
    }
    result = WWD_SUCCESS;
    SDMMC1->CMD = 0;

    exit: //platform_mcu_powersave_enable( );
    return result;
}

static void sdio_prepare_data_transfer( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/
{
    /* Setup a single transfer using the temp buffer */
    user_data = data;
    user_data_size = data_size;
    dma_transfer_size = (uint32_t) ( ( ( data_size + (uint16_t) block_size - 1 ) / (uint16_t) block_size ) * (uint16_t) block_size );

    if ( direction == BUS_WRITE )
    {
        dma_data_source = data;
    }
    else
    {
        dma_data_source = temp_dma_buffer;
    }

    SDMMC1->DTIMER    = (uint32_t) SDMMC_DATATIMEOUT;
    SDMMC1->DLEN      = dma_transfer_size;
    SDMMC1->DCTRL     = (uint32_t) sdio_get_blocksize_dctrl( block_size ) | bus_direction_mapping[ (int) direction ] | SDMMC_TRANSFER_MODE_BLOCK | SDMMC_DPSM_DISABLE | SDMMC_DCTRL_SDIOEN;

    SDMMC1->IDMACTRL  = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
    SDMMC1->IDMABASE0 = (uint32_t) dma_data_source;
}

void host_platform_enable_high_speed_sdio( void )
{
    SDMMC_InitTypeDef sdio_init_structure;

    /* bus_clock = input_clock / ( 2 * Clockdiv) */
#ifdef SLOW_SDIO_CLOCK
    sdio_init_structure.ClockDiv       = (uint8_t) 10; /* 10 = 10 MHz if SDIO clock = 200MHz */
#else
    sdio_init_structure.ClockDiv       = SDMMC_NSpeed_CLK_DIV; /* 4 = 25MHz if SDIO clock = 200MHz */
#endif
    sdio_init_structure.ClockEdge      = SDMMC_CLOCK_EDGE_RISING;
    sdio_init_structure.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
#ifndef SDIO_1_BIT
    sdio_init_structure.BusWide        = SDMMC_BUS_WIDE_4B;
#else
    sdio_init_structure.BusWide        = SDMMC_BUS_WIDE_1B;
#endif
    sdio_init_structure.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;

    SDMMC_Init( SDMMC1, sdio_init_structure );

}

static sdio_block_size_t find_optimal_block_size( uint32_t data_size )
{

    if ( data_size > (uint32_t) 256 )
        return SDIO_512B_BLOCK;
    if ( data_size > (uint32_t) 128 )
        return SDIO_256B_BLOCK;
    if ( data_size > (uint32_t) 64 )
        return SDIO_128B_BLOCK;
    if ( data_size > (uint32_t) 32 )
        return SDIO_64B_BLOCK;
    if ( data_size > (uint32_t) 16 )
        return SDIO_32B_BLOCK;
    if ( data_size > (uint32_t) 8 )
        return SDIO_16B_BLOCK;
    if ( data_size > (uint32_t) 4 )
        return SDIO_8B_BLOCK;
    if ( data_size > (uint32_t) 2 )
        return SDIO_4B_BLOCK;

    return SDIO_4B_BLOCK;
}

static uint32_t sdio_get_blocksize_dctrl( sdio_block_size_t block_size )
{
    switch ( block_size )
    {
        case SDIO_1B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_1B;
        case SDIO_2B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_2B;
        case SDIO_4B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_4B;
        case SDIO_8B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_8B;
        case SDIO_16B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_16B;
        case SDIO_32B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_32B;
        case SDIO_64B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_64B;
        case SDIO_128B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_128B;
        case SDIO_256B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_256B;
        case SDIO_512B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_512B;
        case SDIO_1024B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_1024B;
        case SDIO_2048B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_2048B;
        default:
            return 0;
    }
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    SDMMC1->MASK = SDMMC_MASK_SDIOTIE;
    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    SDMMC1->MASK = 0;
    return WWD_SUCCESS;
}

wwd_result_t host_platform_unmask_sdio_interrupt( void )
{
    SDMMC1->MASK |= SDMMC_MASK_SDIOTIE;
    return WWD_SUCCESS;
}

//void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
//{
//    UNUSED_PARAMETER( direction );
//}

/******************************************************
 *             IRQ Handler Definitions
 ******************************************************/
//WWD_RTOS_DEFINE_ISR( sdio_irq )
void sdio_irq(void)
{
    uint32_t intstatus = SDMMC1->STA;
		
//    WWD_BUS_STATS_INCREMENT_VARIABLE( sdio_intrs );

    if ( ( intstatus & ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR ) ) != 0 )
    {
//        WWD_BUS_STATS_INCREMENT_VARIABLE( error_intrs );
        wiced_assert("sdio error flagged",0);
        sdio_transfer_failed = WICED_TRUE;
        SDMMC1->ICR = (uint32_t) 0xffffffff;
        host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
    }
    else
    {
        if ((intstatus & (SDMMC_STA_CMDREND | SDMMC_STA_CMDSENT)) != 0)
        {
            if ( ( SDMMC1->RESP1 & 0x800 ) != 0 )
            {
                sdio_transfer_failed = WICED_TRUE;
                host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
            }

            /* Clear all command/response interrupts */
            SDMMC1->ICR = (SDMMC_STA_CMDREND | SDMMC_STA_CMDSENT);
        }

        if (intstatus & SDMMC_STA_DATAEND)
        {
            wwd_result_t result;
            SDMMC1->ICR      = SDMMC_STA_DATAEND;
            SDMMC1->DLEN     = 0;
            SDMMC1->DCTRL    = SDMMC_DCTRL_SDIOEN;
            SDMMC1->IDMACTRL = SDMMC_DISABLE_IDMA;
            SDMMC1->CMD      = 0;
            result = host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
            wiced_assert( "failed to set dma semaphore", result == WWD_SUCCESS );
        }

        /* Check whether the external interrupt was triggered */
        if (intstatus & SDMMC_STA_SDIOIT)
        {
            /* Clear the interrupt */
            SDMMC1->ICR   = SDMMC_STA_SDIOIT;
            /* Mask interrupt, to be unmasked later by WICED WWD thread */
            SDMMC1->MASK &= ~(SDMMC_MASK_SDIOTIE);
            /* Inform WICED WWD thread */
            wwd_thread_notify_irq( );
        }
    }
}
void SDMMC1_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  sdio_irq();
}

