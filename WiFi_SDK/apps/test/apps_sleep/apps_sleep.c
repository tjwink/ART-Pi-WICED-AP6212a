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
 * Deep sleep test
 *
 */

#include "wiced.h"
#include "wiced_deep_sleep.h"
#include "platform_cache.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define AON_SRAM_PATTERN           0xABABABAB

#define AON_SRAM_TEST_SIZE         2816

#define APP_SLEEP_EACH             10U

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

typedef struct
{
    uint32_t             callback_counters[6];
    uint32_t             test_memory[AON_SRAM_TEST_SIZE / sizeof(uint32_t)];
} aon_memory_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

volatile static aon_memory_t WICED_DEEP_SLEEP_SAVED_VAR( aon );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * We probably are good without this code which puts cores into reset,
 * because in current application we do not actually enable anything.
 * Have this fragment to increase test application robustness.
 * Good test would be to run this application with and without this kind resets.
 */
static uint32_t reset_except_these_wrappers[] =
{
    PLATFORM_SOCSRAM_CH0_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_SOCSRAM_CH1_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_APPSCR4_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_APPSCR4_MASTER_WRAPPER_REGBASE(0x0),
    PLATFORM_CHIPCOMMON_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_CHIPCOMMON_MASTER_WRAPPER_REGBASE(0x0),
    PLATFORM_AON_APB0_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_APPS_APB0_SLAVE_WRAPPER_REGBASE(0x0),
    PLATFORM_APPS_DEFAULT_SLAVE_WRAPPER_REGBASE(0x0),
    0x0
};

static void application_handle_cores( void )
{
    printf( "Core wrappers:\n" );
    platform_backplane_print_cores( ~0x0 );

    if ( ( platform_gci_chipstatus( GCI_CHIPSTATUS_BOOT_MODE_REG ) & GCI_CHIPSTATUS_BOOT_MODE_WCPU_MASK ) == 0 )
    {
        printf( "\nWCPU is booted from ROM.\nApplication may not able to go to sleep as WCPU is up.\nPlease change bootstrap to boot WCPU from RAM.\n\n" );
    }

    platform_backplane_reset_cores( reset_except_these_wrappers, ~0x0 );
}

static void application_initialize_aon_ram( void )
{
    unsigned i;

    for ( i = 0; i < ARRAYSIZE( aon.test_memory ); ++i )
    {
        aon.test_memory[i] = AON_SRAM_PATTERN;
    }
}

static void application_check_aon_ram( void )
{
    unsigned bad_counter = 10;
    unsigned i;

    printf( "Checking always-on SRAM..." );

    for ( i = 0; i < ARRAYSIZE( aon.test_memory ); ++i )
    {
        uint32_t word = aon.test_memory[i];

        if ( word != AON_SRAM_PATTERN )
        {
            printf( "lost word #%u: 0x%x...", i, (unsigned)word );
            --bad_counter;
            if ( bad_counter == 0 )
            {
                break;
            }
            printf("\n");
        }
    }
    printf( ( i < ARRAYSIZE( aon.test_memory ) ) ? "failure\n"  : "success\n" );
}

static void application_handle_powersave( void )
{
    unsigned i;

    for ( i = 0; i < ARRAYSIZE( aon.callback_counters ); ++i )
    {
        printf( "%u counter: %lx\n", i, aon.callback_counters[i]);
    }

    for ( i = 0; ; i++)
    {
        host_rtos_delay_milliseconds( 1000 );

        printf( "%u from %u, curr res 0x%lx\n", i, APP_SLEEP_EACH, PLATFORM_PMU->res_state);

        if ( ( i > 0 ) && ( ( i % APP_SLEEP_EACH ) == 0 ) )
        {
            host_rtos_delay_milliseconds( 100 );

            platform_mcu_powersave_enable( );

            host_rtos_delay_milliseconds( 5000 );

            platform_mcu_powersave_disable( );
        }
    }
}

static void application_deep_sleep_event_handler( wiced_deep_sleep_event_type_t event, int handler )
{
    int offset = 0;

    switch ( event )
    {
        case WICED_DEEP_SLEEP_EVENT_ENTER:
            offset = 0;
            break;

        case WICED_DEEP_SLEEP_EVENT_CANCEL:
            offset = 1;
            break;

        case WICED_DEEP_SLEEP_EVENT_LEAVE:
            offset = 2;
            break;

        default:
            break;
    }

    offset += handler * 3;

    aon.callback_counters[offset] += 1;
}

WICED_DEEP_SLEEP_EVENT_HANDLER( application_deep_sleep_event_handler_1 )
{
    application_deep_sleep_event_handler( event, 0 );
}

WICED_DEEP_SLEEP_EVENT_HANDLER( application_deep_sleep_event_handler_2 )
{
    application_deep_sleep_event_handler( event, 1 );
}

void application_start( void )
{
    const wiced_bool_t is_warmboot = WICED_DEEP_SLEEP_IS_WARMBOOT( );
    wiced_time_t       wtime       = 0;

    host_rtos_delay_milliseconds( 5000 );

    wiced_time_get_time( &wtime );

    printf( "Start: is_warmboot=%d current_time=%u\n" , (int)is_warmboot, (unsigned)wtime );

    if ( is_warmboot )
    {
        application_check_aon_ram( );
    }

    application_initialize_aon_ram( );

    if ( !is_warmboot )
    {
        /*
         * Do it only during cold boot.
         * After warm boot used resource mask should be sufficient to keep down all unneeded cores and resources.
         * Furthemore, accessing them to print may trigger bus hang.
         */
        application_handle_cores( );
    }

    application_handle_powersave( );
}
