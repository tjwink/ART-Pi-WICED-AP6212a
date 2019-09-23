/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

#include "wwd_rtos.h"
#include "wwd_assert.h"
#include "internal/wwd_sdpcm.h"
#include "internal/wwd_internal.h"
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "platform_toolchain.h"

/******************************************************
 *             Macros
 ******************************************************/

#ifndef WWD_THREAD_POLL_TIMEOUT
#define WWD_THREAD_POLL_TIMEOUT      (NEVER_TIMEOUT)
#endif

#ifndef WWD_THREAD_POKE_TIMEOUT
#define WWD_THREAD_POKE_TIMEOUT      (100)
#endif

/******************************************************
 *             Global Functions
 ******************************************************/

__weak void wwd_wait_for_wlan_event( host_semaphore_type_t* transceive_semaphore )
{
    wwd_result_t result = WWD_SUCCESS;

    REFERENCE_DEBUG_ONLY_VARIABLE( result );

    /* Check if we have run out of bus credits */
    if ( wwd_sdpcm_get_available_credits( ) == 0 )
    {
        /* Keep poking the WLAN until it gives us more credits */
        result = wwd_bus_poke_wlan( );
        wiced_assert( "Poking failed!", result == WWD_SUCCESS );

        result = host_rtos_get_semaphore( transceive_semaphore, (uint32_t) WWD_THREAD_POKE_TIMEOUT, WICED_FALSE );
    }
    else
    {
        /* Put the bus to sleep and wait for something else to do */
//        if ( WWD_WLAN_MAY_SLEEP( ) )
//        {
//            result = wwd_allow_wlan_bus_to_sleep( );
//            wiced_assert( "Error setting wlan sleep", result == WWD_SUCCESS );
//        }
        result = host_rtos_get_semaphore( transceive_semaphore, (uint32_t) WWD_THREAD_POLL_TIMEOUT, WICED_FALSE );
    }
    wiced_assert("Could not get wwd sleep semaphore\n", (result == WWD_SUCCESS)||(result == WWD_TIMEOUT) );
}
