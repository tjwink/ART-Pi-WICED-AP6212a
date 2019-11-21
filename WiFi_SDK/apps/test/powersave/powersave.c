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

#include "command_console_commands.h"

#include "wiced_management.h"
#include "wiced_low_power.h"
#include "wiced_power_logger.h"

/******************************************************
 *                      Macros
 ******************************************************/
// To give enough time for the PAD tool to connect and demo the capabilities.
#ifdef WICED_POWER_LOGGER_ENABLE
#define POWERSAVE_DEEP_SLEEP_TIME_MS                              "5000"
#define POWERSAVE_DEEP_SLEEP_SHORT_TIME_MS                        "5000"
#define POWERSAVE_ITERATION_DELAY_MS                              "20000"
#else
#define POWERSAVE_DEEP_SLEEP_TIME_MS                              "120000"
#define POWERSAVE_DEEP_SLEEP_SHORT_TIME_MS                        "5000"
#define POWERSAVE_ITERATION_DELAY_MS                              "3000"
#endif

#define POWERSAVE_ITERATION_SLEEP_COMMAND                         "sleep "POWERSAVE_ITERATION_DELAY_MS
#define POWERSAVE_JOIN_COMMAND                                    "join test123 open PASSWORD 192.168.1.100 192.168.1.255 192.168.1.1"
#define POWERSAVE_WLAN_POWERSAVE_ENABLE_MODE_CONFIG_COMMAND       "wifi_powersave 1"
#define POWERSAVE_WLAN_POWERSAVE_DISABLE_MODE_CONFIG_COMMAND      "wifi_powersave 0"
#define POWERSAVE_APPS_FREQ_60MHZ_COMMAND                         "mcu_powersave_freq 2"
#define POWERSAVE_APPS_ALP_AVAILABLE_CLOCK_REQUEST_COMMAND        "mcu_powersave_clock 1 0"
#define POWERSAVE_APPS_HT_AVAILABLE_CLOCK_REQUEST_COMMAND         "mcu_powersave_clock 1 1"
#define POWERSAVE_APPS_BACKPLANE_ON_ILP_CLOCK_REQUEST_COMMAND     "mcu_powersave_clock 1 2"
#define POWERSAVE_APPS_BACKPLANE_ON_ALP_CLOCK_REQUEST_COMMAND     "mcu_powersave_clock 1 3"
#define POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND               "mcu_powersave_tick 0"
#define POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND              "mcu_powersave_mode 0"
#define POWERSAVE_APPS_SLEEP_MODE_CONFIG_COMMAND                  "mcu_powersave_mode 1"
#define POWERSAVE_APPS_POWERSAVE_ENABLE_COMMAND                   "mcu_powersave 1"
#define POWERSAVE_APPS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND       "mcu_powersave_sleep 1 "POWERSAVE_DEEP_SLEEP_TIME_MS
#define POWERSAVE_APPS_SHORT_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND "mcu_powersave_sleep 1 "POWERSAVE_DEEP_SLEEP_SHORT_TIME_MS
#define POWERSAVE_APPS_RTOS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND  "mcu_powersave_sleep 0 "POWERSAVE_DEEP_SLEEP_TIME_MS
#define POWERSAVE_IPERF_UDP_SERVER                                "iperf -s -u -i 1"
#define POWERSAVE_START_SEQUENCE_AGAIN_COMMAND                    NULL

#define NETWORK_INTERFACE                    WICED_STA_INTERFACE
#define NETWORK_INTERFACE_ADDRESS_ASSIGNING  WICED_USE_EXTERNAL_DHCP_SERVER_RESTORE
#define NETWORK_INTERFACE_ADDRESS            NULL

/********************************************************************************
 *  List of stanadlone tests. Please change one of them to 1 to enable that test.
 ********************************************************************************/

/*
 * WLAN is not associated with AP.
 * APPS go to deep-sleep for long period, wake-up for short period, then again go to deep-sleep for long period.
 */
#define POWERSAVE_STANDALONE_TEST_DEEPSLEEP_NO_ASSOC              0

/*
 * WLAN is associated with AP.
 * APPS go to deep-sleep for long period, wake-up for short period, then again go to deep-sleep for long period.
 */
#define POWERSAVE_STANDALONE_TEST_DEEPSLEEP_ASSOC                 0

/*
 * WLAN is associated with AP.
 * APPS is waiting packets from WLAN. APPS is powered on but no clocks running.
 * When WLAN pass packet to APPS the APPS wake-up and HT clock requested, after packet handled clocks dropped and wait started again.
 */
#define POWERSAVE_STANDALONE_TEST_WAIT_FOR_WLAN                   0

/*
 * WLAN is associated with AP.
 * APPS is waiting packets from WLAN.
 * APPS is powered on, ALP clock available, bus is on ILP clock (so UART and interrupts are working).
 * When WLAN pass packet to APPS the APPS wake-up and HT clock requested, after packet handled HT clock dropped.
 */
#define POWERSAVE_STANDALONE_TEST_ACTIVE_WAIT_FOR_WLAN            0

/*
 * WLAN is associated with AP.
 * APPS is running iPerf UDP server.
 * APPS is powered on, HT clock available, bus is running on ALP clock.
 * APPS backplane and CPU frequency reduced to 80MHZ.
 */
#define POWERSAVE_STANDALONE_TEST_LOW_POWER_NETWORKING            0

/*
 * WLAN is not associated with AP.
 * APPS go to deep-sleep for short period, wake up and immediately go to back to deep-sleep, repeat this cycle continuously.
 * For more precise profiling may worth to go to UART driver and make TX function (e.g. uart_slow_transmit_bytes()) do nothing.
 */
#define POWERSAVE_STANDALONE_TEST_WAKEUP_FROM_DEEP_SLEEP_PROFILE  0

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

static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static wiced_bool_t wprint_permission = WICED_FALSE;

static const command_t commands[] =
{
    ALL_COMMANDS
    CMD_TABLE_END
};

static void startup_commands( void )
{
    static char* cold_boot_commands[] =
    {
#if POWERSAVE_STANDALONE_TEST_WAKEUP_FROM_DEEP_SLEEP_PROFILE
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SHORT_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_DEEPSLEEP_NO_ASSOC
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_DEEPSLEEP_ASSOC
        POWERSAVE_JOIN_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_WLAN_POWERSAVE_ENABLE_MODE_CONFIG_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_WAIT_FOR_WLAN
        POWERSAVE_JOIN_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_WLAN_POWERSAVE_ENABLE_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_ACTIVE_WAIT_FOR_WLAN
        POWERSAVE_JOIN_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_WLAN_POWERSAVE_ENABLE_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_ALP_AVAILABLE_CLOCK_REQUEST_COMMAND,
        POWERSAVE_APPS_BACKPLANE_ON_ILP_CLOCK_REQUEST_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_RTOS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_LOW_POWER_NETWORKING
        POWERSAVE_JOIN_COMMAND,
        POWERSAVE_WLAN_POWERSAVE_DISABLE_MODE_CONFIG_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_APPS_FREQ_60MHZ_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_HT_AVAILABLE_CLOCK_REQUEST_COMMAND,
        POWERSAVE_APPS_BACKPLANE_ON_ALP_CLOCK_REQUEST_COMMAND,
        POWERSAVE_APPS_POWERSAVE_ENABLE_COMMAND,
        POWERSAVE_IPERF_UDP_SERVER
#endif
    };
    static char* warm_boot_commands[] =
    {
#if POWERSAVE_STANDALONE_TEST_DEEPSLEEP_ASSOC || POWERSAVE_STANDALONE_TEST_DEEPSLEEP_NO_ASSOC
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_ITERATION_SLEEP_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#elif POWERSAVE_STANDALONE_TEST_WAKEUP_FROM_DEEP_SLEEP_PROFILE
        POWERSAVE_APPS_TICKLESS_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_DEEPSLEEP_MODE_CONFIG_COMMAND,
        POWERSAVE_APPS_SHORT_SLEEP_WITH_POWERSAVE_ENABLED_COMMAND,
        POWERSAVE_START_SEQUENCE_AGAIN_COMMAND
#endif
    };

    char** commands;
    int commands_num;
    int i;

    if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE() )
    {
        commands = warm_boot_commands;
        commands_num = ARRAYSIZE( warm_boot_commands );
    }
    else
    {
        commands = cold_boot_commands;
        commands_num = ARRAYSIZE( cold_boot_commands );
    }

    i = 0;
    while ( i < commands_num )
    {
        if ( commands[ i ] == POWERSAVE_START_SEQUENCE_AGAIN_COMMAND )
        {
            i = 0;
        }
        else
        {
            console_parse_cmd( commands[ i ] );
            i++;
        }
    }
}

void application_start( void )
{
    wiced_result_t result;

    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_PROFILING, EVENT_DESC_FUNC_TIME );

    /* Initialise the device */
    if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        /* Resuming WICED */
        result = wiced_resume_after_deep_sleep( );
        if ( result != WICED_SUCCESS )
            WPRINT_APP_INFO(( "WARMBOOT: WICED Resume failed, result = %d\n", result ));

        /* Resume network interface */
        if ( wiced_wifi_is_sta_link_up( ) )
        {
            result = wiced_network_resume_after_deep_sleep( NETWORK_INTERFACE, NETWORK_INTERFACE_ADDRESS_ASSIGNING, NETWORK_INTERFACE_ADDRESS );
            if ( result != WICED_SUCCESS )
                WPRINT_APP_INFO(( "WARMBOOT: Networking NOT inited, result = %d\n", result ));
        }
    }
    else
    {
        wiced_init( );
    }

    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_PROFILING, EVENT_DESC_FUNC_IDLE );

    wprint_permission = WICED_TRUE;

    /* Run console */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );

    /* Console is up and running in dedicated thread. Let's run now startup commands. */
    startup_commands();
}

int platform_wprint_permission( void )
{
#if POWERSAVE_STANDALONE_TEST_WAKEUP_FROM_DEEP_SLEEP_PROFILE
    UNUSED_VARIABLE( wprint_permission );
    return 0;
#else
    return ( !WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) || wprint_permission ) ? 1 : 0;
#endif
}

#if POWERSAVE_STANDALONE_TEST_WAKEUP_FROM_DEEP_SLEEP_PROFILE
void mcu_powersave_deep_sleep_event_handler( wiced_bool_t before )
{
}
#endif
