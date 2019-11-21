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
 * Defines WWD SDIO functions for STM32F4xx MCU
 */
#include <string.h> /* For memcpy */
#include "wwd_platform_common.h"
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_constants.h"
#include "wwd_rtos_isr.h"
//#include "misc.h"
//#include "platform_cmsis.h"
//#include "platform_peripheral.h"
#include "platform_config.h"
//#include "chip.h"
#include "fsl_debug_console.h"
#include "fsl_sdio.h"
/******************************************************
 *             Constants
 ******************************************************/


/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Variables
 ******************************************************/


/******************************************************
 *             Static Function Declarations
 ******************************************************/
host_semaphore_type_t        sdio_transfer_finished_semaphore;

/******************************************************
 *             Function definitions
 ******************************************************/
#ifndef  WICED_DISABLE_MCU_POWERSAVE
static void sdio_oob_irq_handler( void* arg )
{
    UNUSED_PARAMETER(arg);
    WWD_BUS_STATS_INCREMENT_VARIABLE( oob_intrs );
//    platform_mcu_powersave_exit_notify( );
    wwd_thread_notify_irq( );
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt( void )
{
    /* Set GPIO_B[1:0] to input. One of them will be re-purposed as OOB interrupt */
//    platform_gpio_init( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], INPUT_HIGH_IMPEDANCE );
//    platform_gpio_irq_enable( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], IRQ_TRIGGER_RISING_EDGE, sdio_oob_irq_handler, 0 );
    return WWD_SUCCESS;
}

uint8_t host_platform_get_oob_interrupt_pin( void )
{
    return WICED_WIFI_OOB_IRQ_GPIO_PIN;
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */
sdio_card_t g_sdio;
wwd_result_t host_platform_bus_init( void )
{
		wwd_result_t     result;
		/* 配置为输出模式，默认高电平，不使用中断，后面通过GPIO_PinInit函数加载配置 */
    gpio_pin_config_t Wifi_Reg_On_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
		/* 初始化 Wifi_Reg_On GPIO. */
		GPIO_PinInit(GPIO1, 0, &Wifi_Reg_On_config);
		/*复位WIFI*/
		GPIO_PinWrite(GPIO1 , 0, 0U);
		host_rtos_delay_milliseconds( 100 );
		GPIO_PinWrite(GPIO1 , 0, 1U);	
		
    sdio_card_t *card = &g_sdio;
    card->host.base = MMC_HOST_BASEADDR;
    card->host.sourceClock_Hz = MMC_HOST_CLK_FREQ;

		result = host_rtos_init_semaphore( &sdio_transfer_finished_semaphore );
    if ( result != WWD_SUCCESS )
    {
        return result;
    }
		if(SDIO_Init(card)!=kStatus_Success)
			return WWD_TIMEOUT;
		else
		{
			return WWD_SUCCESS;
		}
}

wwd_result_t host_platform_sdio_enumerate( void )
{
			return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_deinit( void )
{
    return WWD_SUCCESS;
}

wwd_result_t host_platform_sdio_transfer( wwd_bus_transfer_direction_t  direction,
																					wwd_sdio_command_t	  				command, 
																					sdio_transfer_mode_t 					mode, 
																					sdio_block_size_t 						block_size, 
																					uint32_t 											argument, /*@null@*/ 
																					uint32_t* 										data, 
																					uint16_t 											data_size, 
																					sdio_response_needed_t  			response_expected, /*@out@*/ /*@null@*/ 
																					uint32_t* 										response )
{

		uint8_t *txData = NULL;
    uint8_t *rxData = NULL;
		wwd_result_t result;
	
		if(data_size)
		{	
			if(direction == BUS_READ)
			{
				rxData = (uint8_t *)data;
			}
			else
			{
				txData = (uint8_t *)data;
			}
		}
		/* Wait for the whole transfer to complete */
		result = host_rtos_get_semaphore( &sdio_transfer_finished_semaphore, (uint32_t) 50, WICED_TRUE );
		if ( result != WWD_SUCCESS )
		{
				PRINTF("SDIO传送超时，command=%d,argument=%x,block_size=%d,data_size=%d",command,argument,block_size,data_size);
				goto exit;
		}
		wwd_thread_notify_irq( );
		//PRINTF("SDIO得到信号量\r\n");
		if(SDIO_IO_Transfer(&g_sdio, command, argument, block_size, txData, rxData, data_size, response) == kStatus_Fail)
		{
			return WWD_TIMEOUT;
		}
//		if(false ==SDMMCEVENT_Wait(0, SDMMCHOST_TRANSFER_COMPLETE_TIMEOUT))		
//		{
//			PRINTF("SDIO传送超时，command=%d,argument=%x,block_size=%d,data_size=%d",command,argument,block_size,data_size);
//			return WWD_TIMEOUT;
//		}		
		return WWD_SUCCESS;
exit:
//    platform_mcu_powersave_enable();
    return result;		
}


wwd_result_t host_platform_enable_high_speed_sdio( void )
{
		sdio_bus_width_t busWidth = kSDIO_DataBus1Bit;
    /* try to switch high speed */
    if (kStatus_Success != SDIO_SwitchToHighSpeed(&g_sdio))
    {
        return WWD_TIMEOUT;
    }
    /* switch data bus width */
    if ((g_sdio.cccrflags & kSDIO_CCCRSupportHighSpeed) || (g_sdio.cccrflags & kSDIO_CCCRSupportLowSpeed4Bit))
    {
        busWidth = kSDIO_DataBus4Bit;
    }
    if (kStatus_Success != SDIO_SetDataBusWidth(&g_sdio, busWidth))
    {
        return WWD_TIMEOUT;
    }
    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    return  WWD_SUCCESS;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    return  WWD_SUCCESS;
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    UNUSED_PARAMETER( direction );
}
