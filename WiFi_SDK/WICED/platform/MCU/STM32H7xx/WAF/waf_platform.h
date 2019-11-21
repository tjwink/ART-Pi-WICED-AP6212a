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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "platform_dct.h"
#include <string.h>
#include <stdlib.h>
#include "spi_flash.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "wwd_assert.h"
#include "wiced_framework.h"
#include "elf.h"
#include "wiced_apps_common.h"

#define DCT_SFLASH_COPY_BUFFER_SIZE_ON_STACK (2048)

/******************************************************
 *                   Constants
 ******************************************************/

#define PLATFORM_FLASH_SECTOR_0           ((uint16_t)0U)  /* Bank 1, Sector 0 */
#define PLATFORM_FLASH_SECTOR_1           ((uint16_t)1U)  /* Bank 1, Sector 1 */
#define PLATFORM_FLASH_SECTOR_2           ((uint16_t)2U)  /* Bank 1, Sector 2 */
#define PLATFORM_FLASH_SECTOR_3           ((uint16_t)3U)  /* Bank 1, Sector 3 */
#define PLATFORM_FLASH_SECTOR_4           ((uint16_t)4U)  /* Bank 1, Sector 4 */
#define PLATFORM_FLASH_SECTOR_5           ((uint16_t)5U)  /* Bank 1, Sector 5 */
#define PLATFORM_FLASH_SECTOR_6           ((uint16_t)6U)  /* Bank 1, Sector 6 */
#define PLATFORM_FLASH_SECTOR_7           ((uint16_t)7U)  /* Bank 1, Sector 7 */
#define PLATFORM_FLASH_SECTOR_8           ((uint16_t)8U)  /* Bank 2, Sector 0 */
#define PLATFORM_FLASH_SECTOR_9           ((uint16_t)9U)  /* Bank 2, Sector 1 */
#define PLATFORM_FLASH_SECTOR_10          ((uint16_t)10U) /* Bank 2, Sector 2 */
#define PLATFORM_FLASH_SECTOR_11          ((uint16_t)11U) /* Bank 2, Sector 3 */
#define PLATFORM_FLASH_SECTOR_12          ((uint16_t)12U) /* Bank 2, Sector 4 */
#define PLATFORM_FLASH_SECTOR_13          ((uint16_t)13U) /* Bank 2, Sector 5 */
#define PLATFORM_FLASH_SECTOR_14          ((uint16_t)14U) /* Bank 2, Sector 6 */
#define PLATFORM_FLASH_SECTOR_15          ((uint16_t)15U) /* Bank 2, Sector 7 */

#if defined(OTA2_SUPPORT)
#define SECTOR_SIZE         (128*1024)
#define FLASH_ADDR_MASK     (0x01FFFFFF)

/******************************************************
 *                    Macros
 ******************************************************/
#define DCT1_START_ADDR  ((uint32_t)&dct1_start_addr_loc)
#define DCT1_SIZE        ((uint32_t)&dct1_size_loc)
#define DCT2_START_ADDR  ((uint32_t)&dct2_start_addr_loc)
#define DCT2_SIZE        ((uint32_t)&dct2_size_loc)

#define PLATFORM_DCT_COPY1_START_ADDRESS     ( DCT1_START_ADDR & FLASH_ADDR_MASK )
#define PLATFORM_DCT_COPY1_START_SECTOR      ( (uint16_t)(PLATFORM_DCT_COPY1_START_ADDRESS / SECTOR_SIZE) )
#define PLATFORM_DCT_COPY1_END_ADDRESS       ( (uint16_t)(PLATFORM_DCT_COPY1_START_ADDRESS + DCT1_SIZE) )
#define PLATFORM_DCT_COPY1_END_SECTOR        ( (uint16_t)(PLATFORM_DCT_COPY1_END_ADDRESS / SECTOR_SIZE) )  /* one sector ?!?!?! */

#define PLATFORM_DCT_COPY2_START_ADDRESS     ( (uint16_t)(DCT2_START_ADDR & FLASH_ADDR_MASK) )
#define PLATFORM_DCT_COPY2_START_SECTOR      ( (uint16_t)(PLATFORM_DCT_COPY2_START_ADDRESS / SECTOR_SIZE)  )
#define PLATFORM_DCT_COPY2_END_ADDRESS       ( (uint16_t)(PLATFORM_DCT_COPY2_START_ADDRESS + DCT2_SIZE) )
#define PLATFORM_DCT_COPY2_END_SECTOR        ( (uint16_t)(PLATFORM_DCT_COPY2_END_ADDRESS / SECTOR_SIZE) )  /* one sector ?!?!?! */

#define PLATFORM_DEFAULT_LOAD                ( WICED_FRAMEWORK_LOAD_ONCE )

#ifdef CURRENT_APPLICATION_USES_INTERNAL_FLASH
#define PLATFORM_APP_INT_START_SECTOR        ( FLASH_SECTOR_3  )
#define PLATFORM_APP_INT_END_SECTOR          ( FLASH_SECTOR_7 )
#endif

#define PLATFORM_SFLASH_PERIPHERAL_ID        (0)
#else
/******************************************************
 *                    Macros
 ******************************************************/
#define SECTOR_SIZE         ( 128 * 1024 )
#define SECTOR_MASK         ( ( uint32_t ) ( SECTOR_SIZE - 1 ) )

#define DCT1_START_ADDR  ((uint32_t)&dct1_start_addr_loc)
#define DCT1_SIZE        ((uint32_t)&dct1_size_loc)
#define DCT2_START_ADDR  ((uint32_t)&dct2_start_addr_loc)
#define DCT2_SIZE        ((uint32_t)&dct2_size_loc)

#define PLATFORM_DCT_COPY1_START_SECTOR      ( PLATFORM_FLASH_SECTOR_1  )
#define PLATFORM_DCT_COPY1_START_ADDRESS     ( DCT1_START_ADDR )
#define PLATFORM_DCT_COPY1_END_SECTOR        ( PLATFORM_FLASH_SECTOR_1 )
#define PLATFORM_DCT_COPY1_END_ADDRESS       ( DCT1_START_ADDR + DCT1_SIZE )
#define PLATFORM_DCT_COPY2_START_SECTOR      ( PLATFORM_FLASH_SECTOR_2  )
#define PLATFORM_DCT_COPY2_START_ADDRESS     ( DCT2_START_ADDR )
#define PLATFORM_DCT_COPY2_END_SECTOR        ( PLATFORM_FLASH_SECTOR_2 )
#define PLATFORM_DCT_COPY2_END_ADDRESS       ( DCT2_START_ADDR + DCT1_SIZE )

#define PLATFORM_DEFAULT_LOAD                ( WICED_FRAMEWORK_LOAD_ONCE )
#endif
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
 *                 Global Variables
 ******************************************************/
/* These come from the linker script */
extern void* dct1_start_addr_loc;
extern void* dct1_size_loc;
extern void* dct2_start_addr_loc;
extern void* dct2_size_loc;

/******************************************************
 *               Function Declarations
 ******************************************************/

/* WAF platform functions */
void platform_start_app         ( uint32_t entry_point );
void platform_load_app_chunk    ( const image_location_t* app_header_location, uint32_t offset, void* physical_address, uint32_t size );
void platform_erase_app_area    ( uint32_t physical_address, uint32_t size );
platform_result_t  platform_erase_flash       ( uint16_t start_sector, uint16_t end_sector );
platform_result_t  platform_write_flash_chunk ( uint32_t address, const void* data, uint32_t size );

/**
 * Generic Button / flash LED get press time
 *
 * NOTE: This uses wiced_rtos_delay_milliseconds(), and blocks the current thread until:
 * - The button is released
 * - the max_time has expired (ms)
 *
 * NOTE: You must at least call NoOS_setup_timing(); before calling this.
 *
 *        To change the granularity of the timer (default 100us), change
 *           platforms/<platform>/platform.h: PLATFORM_BUTTON_PRESS_CHECK_PERIOD
 *           NOTE: This also changes the "flashing" frequency of the LED)
 *
 *        To change the polarity of the button (default is pressed = low), change
 *           platforms/<platform>/platform.h: PLATFORM_BUTTON_PRESSED_STATE
 *
 * @param   button_index     - platform_button_t (generic button index, see platforms/<platform>/platform.c for table)
 * @param   led_index        - wiced_led_index_t (generic led index, see platfors/<platform>/platform.c for table)
 *
 * @return  ms button was held
 *
 */
uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time );

/* Check length of time the "Factory Reset" button is pressed
 *
 * NOTES: This is used for bootloader (PLATFORM_HAS_OTA) and ota2_bootloader.
 *        You must at least call NoOS_setup_timing(); before calling this.
 *
 *        To change the button used for this test on your platform, change
 *           platforms/<platform>/platform.h: PLATFORM_FACTORY_RESET_BUTTON_INDEX
 *
 *        To change the LED used for this purpose, see the usage of PLATFORM_RED_LED_INDEX
 *           platforms/<platform>/platform.c:
 *
 * USAGE for OTA support (see <Wiced-SDK>/WICED/platform/MCU/wiced_waf_common.c::wiced_waf_check_factory_reset() )
 *            > 5 seconds - initiate Factory Reset
 *
 * USAGE for OTA2 support (Over The Air version 2 see <Wiced-SDK>/apps/waf/ota2_bootloader/ota2_bootloader.c).
 *             ~5 seconds - start SoftAP
 *            ~10 seconds - initiate Factory Reset
 *
 * param    max_time    - maximum time to wait
 *
 * returns  usecs button was held
 *
 */
uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time );

#ifdef __cplusplus
} /* extern "C" */
#endif

