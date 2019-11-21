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
 *
 *This file implements the quad SPI flash interface SPIFI.
 */

#include "platform_peripheral.h"
#include "chip.h"
#include "spifilib_api.h"
#include "RTOS/wwd_rtos_interface.h"
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
#define SPIFLASH_BASE_ADDRESS (0x14000000)

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *                   Variables
 ******************************************************/
static uint32_t lmem[21];
static SPIFI_HANDLE_T *spifi_handle;

/******************************************************
 *               Function Definitions
 ******************************************************/

platform_result_t platform_spifi_init( const platform_spifi_t* spifi )
{
    uint32_t spifiBaseClockRate;
    uint32_t maxSpifiClock;

    /*
    if( spi_ptr->semaphore_is_inited == WICED_FALSE )
    {
        host_rtos_init_semaphore( &spi_ptr->in_use_semaphore );
        spi_ptr->semaphore_is_inited = WICED_TRUE;
    }
    else
    {
        host_rtos_get_semaphore( &spi_ptr->in_use_semaphore, NEVER_TIMEOUT, WICED_FALSE );
    }
    */

    /* Mux the port and pin to direct it to SPIFI */
    platform_pin_set_alternate_function( &spifi->clock );
    platform_pin_set_alternate_function( &spifi->miso  );
    platform_pin_set_alternate_function( &spifi->mosi  );
    platform_pin_set_alternate_function( &spifi->sio2 );
    platform_pin_set_alternate_function( &spifi->sio3  );
    platform_pin_set_alternate_function( &spifi->cs  );

    /* SPIFI base clock will be based on the main PLL rate and a divider */
    spifiBaseClockRate = Chip_Clock_GetClockInputHz(CLKIN_MAINPLL);

    /* Setup SPIFI clock to run around 1Mhz. Use divider E for this, as it allows
     * higher divider values up to 256 maximum)
     */
    Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, ((spifiBaseClockRate / 1000000) + 1));
    Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);

    /* Initialize LPCSPIFILIB library, reset the interface */
    spifiInit( ( uint32_t )spifi->spifi_base, true);

    /* register support for the family(s) we may want to work with */
    spifiRegisterFamily( SPIFI_REG_FAMILY_MacronixMX25L );

    /* Initialize and detect a device and get device context */
    spifi_handle = spifiInitDevice( &lmem, sizeof(lmem), ( uint32_t ) spifi->spifi_base, SPIFLASH_BASE_ADDRESS );

    /* Get some info needed for the application */
    maxSpifiClock = spifiDevGetInfo(spifi_handle, SPIFI_INFO_MAXCLOCK);

    /* Setup SPIFI clock to at the maximum interface rate the detected device
       can use. This should be done after device init. */
    Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, ((spifiBaseClockRate / maxSpifiClock) + 1));

    /* start by unlocking the device */
    if (spifiDevUnlockDevice(spifi_handle) != SPIFI_ERR_NONE) {
        return WICED_ERROR;
    }

    /* Enable quad.  If not supported it will be ignored */
    spifiDevSetOpts(spifi_handle, SPIFI_OPT_USE_QUAD, true);

    /* Enter memMode */
    spifiDevSetMemMode(spifi_handle, true);

    /* Just a test */
    maxSpifiClock = *( (uint32_t *)SPIFLASH_BASE_ADDRESS );

    return spifi_handle == NULL ? WICED_ERROR : WICED_SUCCESS;
}

platform_result_t platform_spifi_deinit( const platform_spifi_t* spifi )
{
    return PLATFORM_UNSUPPORTED;
}
