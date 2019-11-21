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

#include "wwd_debug.h"
#include "wwd_wifi.h"
#include "wiced_framework.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_low_power.h"
#include "wiced_time.h"
#include "wiced_crypto.h"
#include "wiced_management.h"
#include "platform/wwd_bus_interface.h"
#include "command_console.h"
#include "wiced_log.h"
#include "spi_flash.h"



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

/******************************************************
 *               Function Definitions
 ******************************************************/

int reboot_console_command( int argc, char *argv[] )
{
    WPRINT_APP_INFO(( "Rebooting...\n" ));

    wiced_rtos_delay_milliseconds( 1000 );

    wiced_framework_reboot();

    /* Never reached */
    return ERR_CMD_OK;
}

int get_time_console_command( int argc, char *argv[] )
{
    wiced_time_t curr_time;

    wiced_time_get_time( &curr_time );

    WPRINT_APP_INFO(( "Time: %u\n", (unsigned)curr_time ));

    return ERR_CMD_OK;
}

int sleep_console_command( int argc, char *argv[] )
{
    wiced_rtos_delay_milliseconds( atoi( argv[ 1 ] ) );
    return ERR_CMD_OK;
}

typedef void (*prng_print_function_t)( char value, uint32_t byte_number );
#define BYTES_TO_BITS( num )              ( 8 * ( num ) )
#define BYTE_HAS_BIT_SET( num, index )    ( ( ( num ) & ( ( uint8_t )1 << ( index )  ) ) != 0 )

static void prng_bit_printer( uint8_t value, uint32_t byte_number )
{
    UNUSED_PARAMETER( byte_number );

    for ( uint8_t bit_index = 0 ; bit_index < BYTES_TO_BITS( sizeof( value ) ); bit_index++ )
    {
        if ( BYTE_HAS_BIT_SET( value, bit_index ) )
        {
            WPRINT_APP_INFO(("1"));
        }
        else
        {
            WPRINT_APP_INFO(("0"));
        }
    }
}

static void prng_hex_printer( uint8_t value, uint32_t byte_number )
{
    WPRINT_APP_INFO(( "0x%02x ", (unsigned)value ));
    if ( ( byte_number % 8 ) == 7 )
    {
        WPRINT_APP_INFO(( "\n" ));
    }
}

static int prng_common_command_handler( int argc, char *argv[], prng_print_function_t print_function )
{
    uint16_t bytes_needed = atoi( argv[ 1 ] );
    uint16_t byte_number  = 0;
    wiced_result_t result;

    do
    {
        uint8_t value;
        result = wiced_crypto_get_random( &value, sizeof( value ) );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(( "Failed to get random number: %d\n", (int)result ));
        }
        else
        {
            print_function( value, byte_number++ );
        }

        bytes_needed -= sizeof( value );
    }
    while ( bytes_needed > 0 );
    WPRINT_APP_INFO(("\n"));
    return ERR_CMD_OK;
}

int prng_get_algorithms_command( int argc, char* argv[] )
{
    WPRINT_APP_INFO(("well512\n"));
#ifdef WICED_SECURE_PRNG_FORTUNA_ENABLE
    WPRINT_APP_INFO(("fortuna\n"));
#endif /* WICED_SECURE_PRNG_FORTUNA_ENABLE */
    return ERR_CMD_OK;
}

int prng_set_algorithm_command( int argc, char* argv[] )
{
    int err = ERR_CMD_OK;
    wiced_result_t result = WICED_ERROR;
    const int algorithm_name_index = 1;
    if ( 0 == strncasecmp( argv[algorithm_name_index], "well512", strlen("well512") ) )
    {
        result = wiced_crypto_use_default_prng( );
    }
#ifdef WICED_SECURE_PRNG_FORTUNA_ENABLE
    else if ( 0 == strncasecmp( argv[algorithm_name_index], "fortuna", strlen("fortuna") ) )
    {
        result = wiced_crypto_use_secure_prng( );
    }
    else
    {
        WPRINT_APP_INFO(( "Invalid prng %s\n", argv[algorithm_name_index] ));
    }
#else
    else
    {
        WPRINT_APP_INFO(( "Unknown prng type %s. Need to define WICED_SECURE_PRNG_FORTUNA_ENABLE in build?\n", argv[algorithm_name_index] ));
    }
#endif /* WICED_SECURE_PRNG_FORTUNA_ENABLE */

    if ( WICED_ERROR == result )
    {
        WPRINT_APP_INFO(( "Error %d\n", result ));
        err = ERR_UNKNOWN;
    }

    return err;

}

int prng_bit_dump_command( int argc, char* argv[] )
{
    return prng_common_command_handler( argc, argv, ( prng_print_function_t ) prng_bit_printer );
}

int prng_console_command( int argc, char* argv[] )
{
    return prng_common_command_handler( argc, argv, ( prng_print_function_t ) prng_hex_printer );
}

int mcu_powersave_console_command( int argc, char *argv[] )
{
    int enable = atoi( argv[ 1 ] );

    if ( enable )
    {
        wiced_platform_mcu_enable_powersave();
    }
    else
    {
        wiced_platform_mcu_disable_powersave();
    }

    return ERR_CMD_OK;
}

int wiced_init_console_command( int argc, char *argv[] )
{
    int init = atoi( argv[ 1 ] );

    if ( init )
    {
        wiced_init();
    }
    else
    {
        wiced_deinit();
    }

    return ERR_CMD_OK;
}

int wiced_log_set_level_console_command( int argc, char *argv[] )
{
    WICED_LOG_FACILITY_T facility;
    WICED_LOG_LEVEL_T loglevel;

    if ( argc == 3)
    {
        facility =  (WICED_LOG_LEVEL_T)atoi(argv[1]);
        loglevel = (WICED_LOG_LEVEL_T)atoi(argv[2]);
    }
    else if(argc == 2)
    {
        /*legacy support for 1 parameter*/
        facility =  WLF_TEST;
        loglevel = (WICED_LOG_LEVEL_T)atoi(argv[1]);
    }
    else
    {
        return ERR_BAD_ARG;
    }

    if (wiced_log_set_facility_level(facility, loglevel) != WICED_SUCCESS)
    {
        return ERR_BAD_ARG;
    }
    return ERR_CMD_OK;
}

int wiced_log_get_level_console_command( int argc, char *argv[] )
{
    for (WICED_LOG_FACILITY_T facility = WLF_DEF; facility < WLF_MAX; facility++)
    {
        WPRINT_APP_INFO(("facility %d, level 0x%08x\n", (unsigned int)facility, (unsigned int)wiced_log_get_facility_level(facility)));
    }
    return ERR_CMD_OK;
}

#ifdef WICED_PLATFORM_INCLUDES_SPI_FRAM

#define FRAM_BLOCK_SIZE (1024)
#define MY_BUFF_SIZE    (32 * 1024)
#define FRAM_ACCESS_CYCLES (100)

int fram_write(int argc, char* argv[])
{
    unsigned int addr,loopcount;
    sflash_handle_t fram;
    unsigned int writekbytes, writebytes;
    wiced_time_t timevar1,timevar2;
    char *fram_txdata = malloc(MY_BUFF_SIZE);
    hw_interface_t hw_interface = INTERFACE_4390x_SPI_0;

    init_sflash( &fram, &hw_interface, 1);

    for (loopcount = 0 ; loopcount < MY_BUFF_SIZE; loopcount++)
    {
        fram_txdata[loopcount] = loopcount % 128;
    }

    addr  = ((strtoul( argv[1], (char **)NULL, 16 )) & 0x00FFFFFF);
    writekbytes=  (unsigned int)   (strtoul( argv[2], (char **)NULL, 16 ));
    writebytes = writekbytes * FRAM_BLOCK_SIZE;


    wiced_time_get_time(&timevar1);

    for(loopcount = 0; loopcount < FRAM_ACCESS_CYCLES; loopcount++)
    {
        sflash_write( &fram, addr, fram_txdata, writebytes);
    }

    wiced_time_get_time(&timevar2);

    wiced_log_printf("Average time taken for Write of %d Kbytes = %d ms\n\r",writekbytes,(timevar2 - timevar1)/FRAM_ACCESS_CYCLES);


    deinit_sflash( &fram);
    return ERR_CMD_OK;
}


int fram_read(int argc, char* argv[])
{
    unsigned int addr,loopcount;
    char *fram_rxdata = malloc(MY_BUFF_SIZE);
    wiced_time_t timevar1,timevar2;
    int errorcount = 0;
    sflash_handle_t fram;
    unsigned int readbytes,readkbytes;
    hw_interface_t hw_interface = INTERFACE_4390x_SPI_0;

    init_sflash( &fram, &hw_interface, 1);

    for (loopcount = 0; loopcount < MY_BUFF_SIZE; loopcount++)
    {
        fram_rxdata[loopcount] = 0;
    }

    addr = ((strtoul( argv[1], (char **)NULL, 16 )) & 0x00FFFFFF);
    readkbytes=  (unsigned int)   (strtoul( argv[2], (char **)NULL, 16 ));
    readbytes = readkbytes * FRAM_BLOCK_SIZE;

    wiced_time_get_time(&timevar1);

    for(loopcount = 0; loopcount < FRAM_ACCESS_CYCLES; loopcount++)
    {
        sflash_read( &fram, addr, fram_rxdata, readbytes);
    }

    wiced_time_get_time(&timevar2);

    wiced_log_printf("Average time taken for Read of %d Kbytes = %d ms\n\r",readkbytes,(timevar2 - timevar1)/FRAM_ACCESS_CYCLES);

   for (loopcount = 0; loopcount < readbytes;loopcount++)
   {
       if (fram_rxdata[loopcount] != loopcount % 128)
           errorcount++;
   }

   if(errorcount == 0)
       wiced_log_printf("Data Write Read matched\n\r");
   else
       wiced_log_printf("Total Errors: %d\n\r", errorcount);

    deinit_sflash( &fram);
    return ERR_CMD_OK;
}
#endif


#if defined(MCU_BCM920739) || defined(BCM43909)
int mcu_powersave_mode_console_command( int argc, char *argv[] )
{
    int mcu_mode_par = atoi( argv[ 1 ] );
    platform_mcu_powersave_mode_t mcu_mode = mcu_mode_par;

    platform_mcu_powersave_set_mode( mcu_mode );

    return ERR_CMD_OK;
}
#endif

#ifdef BCM43909

int mcu_powersave_clock_console_command( int argc, char *argv[] )
{
    int request_par = atoi( argv[ 1 ] );
    int clock_par = atoi( argv[ 2 ] );
    platform_mcu_powersave_clock_t clock = clock_par;

    if ( request_par )
    {
        platform_mcu_powersave_request_clock( clock );
    }
    else
    {
        platform_mcu_powersave_release_clock( clock );
    }

    return ERR_CMD_OK;
}

int mcu_powersave_tick_console_command( int argc, char *argv[] )
{
    int tick_mode_par = atoi( argv[ 1 ] );
    platform_tick_powersave_mode_t tick_mode = tick_mode_par;

    platform_mcu_powersave_set_tick_mode( tick_mode );

    return ERR_CMD_OK;
}

int mcu_powersave_freq_console_command( int argc, char *argv[] )
{
    int freq_mode_par = atoi( argv[ 1 ] );
    platform_cpu_clock_frequency_t freq_mode = freq_mode_par;
    wiced_bool_t result;

    platform_tick_stop();
    result = platform_cpu_clock_init( freq_mode );
    platform_tick_start();

    if ( result != WICED_TRUE )
    {
        WPRINT_APP_INFO(( "Failed to set %d frequency mode\n", freq_mode_par ));
    }

    return ERR_CMD_OK;
}

void WEAK NEVER_INLINE mcu_powersave_sleep_event_handler( wiced_bool_t before )
{
    static uint32_t WICED_DEEP_SLEEP_SAVED_VAR( rxbeaconmbss_saved );
    static uint32_t WICED_DEEP_SLEEP_SAVED_VAR( rxbeaconobss_saved );

    static wiced_counters_t counters; /* large object, keep it static */

    uint32_t rxbeaconmbss = 0;
    uint32_t rxbeaconobss = 0;

    if ( wwd_wifi_get_counters( WWD_STA_INTERFACE, &counters ) == WWD_SUCCESS )
    {
        rxbeaconmbss = counters.rxbeaconmbss;
        rxbeaconobss = counters.rxbeaconobss;
    }

    if ( before )
    {
        rxbeaconmbss_saved = rxbeaconmbss;
        rxbeaconobss_saved = rxbeaconobss;
    }
    else
    {
        uint32_t time_since_deep_sleep_enter = wiced_deep_sleep_ticks_since_enter(); /* call before any prints to not skew results by delays due to printing */

        WPRINT_APP_INFO(( "\n%s boot\n", WICED_DEEP_SLEEP_IS_WARMBOOT() ? "WARM" : "COLD" ));
        WPRINT_APP_INFO(( "BEACONS:\n" ));
        WPRINT_APP_INFO(( "    before sleep: from BSS %u from not BSS %u\n", (unsigned)rxbeaconmbss_saved, (unsigned)rxbeaconobss_saved ));
        WPRINT_APP_INFO(( "    after  sleep: from BSS %u from not BSS %u\n", (unsigned)rxbeaconmbss, (unsigned)rxbeaconobss ));
        WPRINT_APP_INFO(( "    during sleep: from BSS %u from not BSS %u\n", (unsigned)(rxbeaconmbss - rxbeaconmbss_saved), (unsigned)(rxbeaconobss - rxbeaconobss_saved) ));
        WPRINT_APP_INFO(( "TIME since deep-sleep enter: %u\n", (unsigned)time_since_deep_sleep_enter ));
    }
}

int mcu_powersave_sleep_console_command( int argc, char *argv[] )
{
    int mode = atoi( argv[ 1 ] );
    int sleep_ms = atoi( argv[ 2 ] );

    mcu_powersave_sleep_event_handler( WICED_TRUE );

    wiced_platform_mcu_enable_powersave();

    wwd_thread_notify_irq();

    if ( mode == 0 )
    {
        wiced_rtos_delay_milliseconds( sleep_ms );
    }
    else
    {
        int i;

        i = 0;
        while ( 1  )
        {
            uint32_t flags;

            WICED_SAVE_INTERRUPTS( flags );

            if ( PLATFORM_WLAN_POWERSAVE_IS_RES_UP() )
            {
                WICED_RESTORE_INTERRUPTS( flags );

                if ( ++i > 1000 )
                {
                    WPRINT_APP_INFO(( "Refuse to sleep due to WLAN resources are still requested\n" ));
                    break;
                }

                wiced_rtos_delay_milliseconds( 1 );
            }
            else
            {
                platform_mcu_powersave_sleep( sleep_ms, PLATFORM_TICK_SLEEP_FORCE_INTERRUPTS_WLAN_ON );

                WICED_RESTORE_INTERRUPTS( flags );

                break;
            }
        }
    }

    wiced_platform_mcu_disable_powersave();

    mcu_powersave_sleep_event_handler( WICED_FALSE );

    return ERR_CMD_OK;
}

int mcu_powersave_info_console_command( int argc, char *argv[] )
{
    int i;

    if ( WICED_DEEP_SLEEP_IS_ENABLED() )
    {
        WPRINT_APP_INFO(( "Deep-sleep enabled. " ));

        if ( WICED_DEEP_SLEEP_IS_WARMBOOT() )
        {
            WPRINT_APP_INFO(( "Ticks since enter deep-sleep: %u.\n", (unsigned)wiced_deep_sleep_ticks_since_enter()));
        }
        else
        {
            WPRINT_APP_INFO(( "This is cold boot.\n" ));
        }
    }
    else
    {
        WPRINT_APP_INFO(( "Deep-sleep disabled or not supported.\n" ));
    }

    if ( platform_hibernation_is_returned_from( ) )
    {
        WPRINT_APP_INFO(( "Returned from hibernation where spent %u ticks\n", (unsigned)platform_hibernation_get_ticks_spent( ) ));
    }

    WPRINT_APP_INFO(( "MCU powersave is %s now.\n",  platform_mcu_powersave_is_permitted() ? "enabled" : "disabled"));

    WPRINT_APP_INFO(( "MCU powersave mode is %d now.\n",  (int)platform_mcu_powersave_get_mode() ));

    WPRINT_APP_INFO(( "MCU powersave tick mode is %d now.\n",  (int)platform_mcu_powersave_get_tick_mode() ));

    for ( i = 0; i < PLATFORM_MCU_POWERSAVE_CLOCK_MAX; i++ )
    {
        WPRINT_APP_INFO(( "MCU clock %d requested %lu times.\n", i, (unsigned long)platform_mcu_powersave_get_clock_request_counter( (platform_mcu_powersave_clock_t)i ) ));
    }

    return ERR_CMD_OK;
}

int mcu_powersave_gpio_wakeup_enable_console_command( int argc, char *argv[] )
{
    int                                          gpio_wakeup_config_par  = atoi( argv[ 1 ] );
    int                                          gpio_wakeup_trigger_par = atoi( argv[ 2 ] );
    platform_mcu_powersave_gpio_wakeup_config_t  gpio_wakeup_config      = gpio_wakeup_config_par;
    platform_mcu_powersave_gpio_wakeup_trigger_t gpio_wakeup_trigger     = gpio_wakeup_trigger_par;
    platform_result_t                            result;

    result = platform_mcu_powersave_gpio_wakeup_enable( gpio_wakeup_config, gpio_wakeup_trigger );
    if ( result != PLATFORM_SUCCESS )
    {
        WPRINT_APP_INFO(( "Enabling failure: %d\n",  result ));
    }

    return ERR_CMD_OK;
}

int mcu_powersave_gpio_wakeup_disable_console_command( int argc, char *argv[] )
{
    platform_mcu_powersave_gpio_wakeup_disable();
    return ERR_CMD_OK;
}

int mcu_powersave_gpio_wakeup_ack_console_command( int argc, char *argv[] )
{
    platform_mcu_powersave_gpio_wakeup_ack();
    return ERR_CMD_OK;
}

int mcu_powersave_gci_gpio_wakeup_enable_console_command( int argc, char *argv[] )
{
    int                                           gpio_pin_par           = atoi( argv[ 1 ] );
    int                                           gpio_wakeup_config_par = atoi( argv[ 2 ] );
    int                                           gpio_trigger_par       = atoi( argv[ 3 ] );
    platform_pin_t                                gpio_pin               = gpio_pin_par;
    platform_mcu_powersave_gpio_wakeup_config_t   gpio_wakeup_config     = gpio_wakeup_config_par;
    platform_gci_gpio_irq_trigger_t               gpio_trigger           = gpio_trigger_par;
    platform_result_t                             result;

    result = platform_mcu_powersave_gci_gpio_wakeup_enable( gpio_pin, gpio_wakeup_config, gpio_trigger );
    if ( result != PLATFORM_SUCCESS )
    {
        WPRINT_APP_INFO(( "Enabling failure: %d\n",  result ));
    }

    return ERR_CMD_OK;
}

int mcu_powersave_gci_gpio_wakeup_disable_console_command( int argc, char *argv[] )
{
    int            gpio_pin_par = atoi( argv[ 1 ] );
    platform_pin_t gpio_pin     = gpio_pin_par;

    platform_mcu_powersave_gci_gpio_wakeup_disable( gpio_pin );

    return ERR_CMD_OK;
}

int mcu_powersave_gci_gpio_wakeup_ack_console_command( int argc, char *argv[] )
{
    int            gpio_pin_par = atoi( argv[ 1 ] );
    platform_pin_t gpio_pin     = gpio_pin_par;
    wiced_bool_t   res          = platform_mcu_powersave_gci_gpio_wakeup_ack( gpio_pin );

    WPRINT_APP_INFO(( "Ack %s\n", res ? "SUCCEEDED" : "FAILED" ));

    return ERR_CMD_OK;
}

int hibernation_console_command( int argc, char *argv[] )
{
    uint32_t          freq            = platform_hibernation_get_clock_freq();
    uint32_t          max_ticks       = platform_hibernation_get_max_ticks();
    platform_result_t result          = PLATFORM_BADARG;
    uint32_t          ms_to_wakeup    = atoi( argv[ 1 ] );
    uint32_t          ticks_to_wakeup = freq * ms_to_wakeup / 1000;

    WPRINT_APP_INFO(( "\n\n*** To make it work please make sure that application is flashed if not done so! ***\n\n" ));
    WPRINT_APP_INFO(( "Frequency %u maximum ticks 0x%x\n", (unsigned)freq, (unsigned)max_ticks ));

    if ( ticks_to_wakeup > max_ticks )
    {
        WPRINT_APP_INFO(( "Scheduled ticks number %u is too big\n", (unsigned)ticks_to_wakeup ));
    }
    else
    {
        WPRINT_APP_INFO(( "After short sleep will hibernate for %u ms (or %u ticks)\n", (unsigned)ms_to_wakeup, (unsigned)ticks_to_wakeup ));

        host_rtos_delay_milliseconds( 1000 );

        result = platform_hibernation_start( ticks_to_wakeup );
    }

    WPRINT_APP_INFO(( "Hibernation failure: %d\n", (int)result ));

    return ERR_CMD_OK;
}

int mcu_wlan_powersave_stats_console_command( int argc, char *argv[] )
{
#if PLATFORM_WLAN_POWERSAVE
    uint32_t     call_num     = platform_wlan_powersave_get_stats( PLATFORM_WLAN_POWERSAVE_STATS_CALL_NUM );
    uint32_t     up_time      = platform_wlan_powersave_get_stats( PLATFORM_WLAN_POWERSAVE_STATS_UP_TIME );
    uint32_t     wait_up_time = platform_wlan_powersave_get_stats( PLATFORM_WLAN_POWERSAVE_STATS_WAIT_UP_TIME );
    wiced_bool_t is_res_up    = platform_wlan_powersave_is_res_up();

    WPRINT_APP_INFO(( "call_num=%lu up_time=%lu wait_up_time=%lu is_res_up=%d\n", (unsigned long)call_num, (unsigned long)up_time, (unsigned long)wait_up_time, is_res_up ));
#else
    WPRINT_APP_INFO(( "WLAN powersave is not compiled-in\n" ));
#endif /* PLATFORM_WLAN_POWERSAVE */

    return ERR_CMD_OK;
}

WICED_SLEEP_EVENT_HANDLER( deep_sleep_event_handler )
{
    if ( event == WICED_SLEEP_EVENT_WLAN_RESUME )
    {
        mcu_powersave_sleep_event_handler( WICED_FALSE );
    }
}

#endif /* BCM43909 */
