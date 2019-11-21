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
* ping deep sleep application
*
* This application snippet demonstrates how to use
* the WICED host deep sleep APIs.
*
* Features demonstrated
* -host deep sleep entrance
* -resume from deep sleep
* -wake up and ping
* -making variables persist across deep sleep cycles
*
* To demonstrate the app, work through the following steps.
*  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
*     in the include/default_wifi_config_dct.h header file to match your Wi-Fi access point
*  2. Plug the WICED eval board into your computer
*  3. Open a terminal application and connect to the WICED eval board
*  4. Build and download the application (to the WICED board)
*  5. After the application has connected to your Wi-Fi access point, you will need to unplug the board, wait for a few seconds
* and then plug the board back in.  This sequence allows the application to do warm boot cycles.
*
* After the download completes, the terminal displays WICED startup
* information and then :
*  - Detects whether warm or cold boot sequence
*  - Calls Wiced APIs to respectively resume or init based on cold/warm determination.
*  - Joins a Wi-Fi network and configures IP address(es)
*  - Every time: prints stats about deep sleep behavior
*  - Every few times, does a ping to show network is connected
*  - Re-enters deep sleep
*
* TROUBLESHOOTING
*  - Ensure AP information in the default_wifi_config_dct.h file in the include directory is accurate.  E.g. CLIENT_AP_SSID
*  needs to be defined to the APs SSID
*
* NOTES:
* Pings from this app should work at a rate well above 90% (assuming strong wireless connection with no interference).
* Pings from outside systems will work at a rate around 70% with current configuration.  The PING_DEEP_SLEEP_WAIT_TIME value will influence this behavior as follows:
* Higher values mean less time in deep sleep and higher percentage of pings succeeding.  Lower values mean the opposite is true.  Loss occurs due to
* traffic being passed up during the DHCP address process, which seems unavoidable without using static IP addresses, or implementing a traffic identifier function which
* seems too complicated and breakage prone for the benefit it would give.
* The 70% performance level applies mainly to the NetX stacks.  The LwIP stack will generally not perform as well as it does not provide DHCP save/restore functionality.
* Static IP address configuration generally avoids the issue of packets arriving before IP address is configured. (See USE_STATIC_IP_SETTINGS define below.)
*/
#include "wiced.h"
#include "wiced_low_power.h"
#include "wwd_buffer_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WIFI_SLEEP_TIME                      (10000 * MILLISECONDS)
#define POWERSAVE_RETURN_TO_SLEEP_DELAY      10
#define USE_POWERSAVE_POLL
#define NETWORK_INTERFACE                    WICED_STA_INTERFACE
#define PING_NUMBER                          3
#define PING_CONDITION                       ( ( wakeup_count % 3 ) == 0 )
#define DEADLINE_NETWORKING_TO_COMPLETE      1000
/* Amount of time to wait prior to going into deep sleep: higher values mean more time out of deep sleep */
#define PING_DEEP_SLEEP_WAIT_TIME            100

#ifdef USE_STATIC_IP_SETTINGS
static const wiced_ip_setting_t device_static_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address,   MAKE_IPV4_ADDRESS(192, 168,   1,  2) ),
    INITIALISER_IPV4_ADDRESS( .gateway,      MAKE_IPV4_ADDRESS(192, 168,   1,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,      MAKE_IPV4_ADDRESS(255, 255, 255,  0) ),
};
#define NETWORK_INTERFACE_ADDRESS_ASSIGNING  WICED_USE_STATIC_IP
#define NETWORK_INTERFACE_ADDRESS            &device_static_ip_settings
#else
#define NETWORK_INTERFACE_ADDRESS_ASSIGNING  WICED_USE_EXTERNAL_DHCP_SERVER_RESTORE
#define NETWORK_INTERFACE_ADDRESS            NULL
#endif

#ifdef USE_POWERSAVE_POLL
#define NETWORK_INTERFACE_ENABLE_POWERSAVE() wiced_wifi_enable_powersave()
#else
#define NETWORK_INTERFACE_ENABLE_POWERSAVE() wiced_wifi_enable_powersave_with_throughput( POWERSAVE_RETURN_TO_SLEEP_DELAY );
#endif

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
 *               Function Declarations
 ******************************************************/

static wiced_result_t send_ping ( void );
static void           deep_sleep( void );
static wiced_bool_t   get_ping_destination( wiced_ip_address_t* ip_address );
static void           print_wlan_log( void );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_bool_t network_suspended = WICED_FALSE;

static uint32_t     WICED_DEEP_SLEEP_SAVED_VAR( wakeup_count )        = 0;
static uint32_t     WICED_DEEP_SLEEP_SAVED_VAR( cold_boot_fast_time ) = 0;
static uint32_t     WICED_DEEP_SLEEP_SAVED_VAR( cold_boot_slow_time ) = 0;
static uint32_t     WICED_DEEP_SLEEP_SAVED_VAR( max_up_time )         = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    wiced_result_t result;
    wiced_time_t   cur_rtos_time;
    uint32_t       cur_slow_time;
    uint32_t       cur_fast_time;
    uint32_t       warm_boot_fast_time;
    uint32_t       warm_boot_slow_time;
    uint32_t       rtos_time_since_deep_sleep_enter;
    wiced_bool_t   network_inited;

    wakeup_count++;

    wiced_time_get_time( &cur_rtos_time );
    rtos_time_since_deep_sleep_enter = wiced_deep_sleep_ticks_since_enter( );
    cur_slow_time                    = platform_tick_get_time( PLATFORM_TICK_GET_SLOW_TIME_STAMP );
    cur_fast_time                    = platform_tick_get_time( PLATFORM_TICK_GET_FAST_TIME_STAMP );

    WPRINT_APP_INFO(( "Application started: cur_rtos_time=%u cur_slow_time=%lu rtos_time_since_deep_sleep_enter=%lu wakeup_count=%lu\n",
        (unsigned)cur_rtos_time, cur_slow_time, rtos_time_since_deep_sleep_enter, wakeup_count ));

    if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        WPRINT_APP_INFO(( "Warm boot.\n" ));

        warm_boot_fast_time = cur_fast_time;
        warm_boot_slow_time = cur_slow_time;

        result = wiced_resume_after_deep_sleep( );
    }
    else
    {
        WPRINT_APP_INFO(( "Cold boot.\n" ));

        warm_boot_fast_time = 0;
        warm_boot_slow_time = 0;
        cold_boot_fast_time = cur_fast_time;
        cold_boot_slow_time = cur_slow_time;

        result = wiced_init( );

        if ( result == WICED_SUCCESS )
        {
            result = NETWORK_INTERFACE_ENABLE_POWERSAVE( );
        }
    }
    if ( result != WICED_SUCCESS )
    {
        while ( 1 )
        {
            WPRINT_APP_INFO(( "*** Init failed %d, just spinning here ***\n", (int)result ));

            print_wlan_log( );

            wiced_rtos_delay_milliseconds( WIFI_SLEEP_TIME );
        }
    }

    WPRINT_APP_INFO(( "WICED inited: rtos_time_since_deep_sleep_enter=%lu\n", wiced_deep_sleep_ticks_since_enter( ) ));

    if ( !WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        /* Bring up the network interface and connect to the Wi-Fi network */
        WPRINT_APP_INFO(( "Networking is about to bring up\n" ));
        result = wiced_network_up( NETWORK_INTERFACE, NETWORK_INTERFACE_ADDRESS_ASSIGNING, NETWORK_INTERFACE_ADDRESS );
    }
    else
    {
        /* Resume network interface */
        WPRINT_APP_INFO(( "Networking is about to resume\n" ));
        result = wiced_network_resume_after_deep_sleep( NETWORK_INTERFACE, NETWORK_INTERFACE_ADDRESS_ASSIGNING, NETWORK_INTERFACE_ADDRESS );
    }
    if ( result == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Networking inited\n" ));
        network_inited = WICED_TRUE;
    }
    else
    {
        WPRINT_APP_INFO(( "Networking NOT inited: result %d\n", result ));
        network_inited = WICED_FALSE;
    }

    while (1)
    {
        /* Send an ICMP ping to the gateway */
        if ( PING_CONDITION && network_inited )
        {
            int i;

            for ( i = 0; i < PING_NUMBER; ++i )
            {
                print_wlan_log( );
                send_ping( );
            }
        }
        print_wlan_log( );

        /* Print statistic */
        WPRINT_APP_INFO(( "stats: up_num=%lu up_time=%lu max_up_time=%lu wait_up_time=%lu cpu_timer(cold=%lu warm=%lu) pmu_timer(cold=%lu warm=%lu)\n",
            PLATFORM_WLAN_POWERSAVE_GET_STATS( PLATFORM_WLAN_POWERSAVE_STATS_CALL_NUM ),
            PLATFORM_WLAN_POWERSAVE_GET_STATS( PLATFORM_WLAN_POWERSAVE_STATS_UP_TIME ),
            max_up_time,
            PLATFORM_WLAN_POWERSAVE_GET_STATS( PLATFORM_WLAN_POWERSAVE_STATS_WAIT_UP_TIME ),
            cold_boot_fast_time,
            warm_boot_fast_time,
            cold_boot_slow_time,
            warm_boot_slow_time ));

        if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
        {
            uint32_t up_time = PLATFORM_WLAN_POWERSAVE_GET_STATS( PLATFORM_WLAN_POWERSAVE_STATS_UP_TIME );
            max_up_time = MAX( max_up_time, up_time );
        }

        /* Delay to make sure UART messages are sent out */
        wiced_rtos_delay_milliseconds( PING_DEEP_SLEEP_WAIT_TIME );

        /* Enter deep-sleep */
        deep_sleep( );
    }
}

static void deep_sleep( void )
{
    int i;

    /* Wait till packets be sent */
    for ( i = 0; i < DEADLINE_NETWORKING_TO_COMPLETE; ++i )
    {
        if ( wiced_deep_sleep_is_networking_idle( NETWORK_INTERFACE ) )
        {
            break;
        }

        wiced_rtos_delay_milliseconds( 1 );
    }

    /* Suspend network timers */
    if ( !network_suspended )
    {
        if ( wiced_network_suspend( ) == WICED_SUCCESS )
        {
            network_suspended = WICED_TRUE;
        }
    }

    /*
     * Wakeup system monitor thread so it can:
     *     - kick watchdog
     *     - go to sleep for maximum period so deep-sleep be longer
     */
    wiced_wakeup_system_monitor_thread( );

    /* Enable power save */
    wiced_platform_mcu_enable_powersave( );

    /* Deep-sleep for a while */
    wiced_rtos_delay_milliseconds( WIFI_SLEEP_TIME );

    /* Disable power save */
    wiced_platform_mcu_disable_powersave( );

    /* Resume network timers */
    if ( network_suspended )
    {
        if ( wiced_network_resume( ) == WICED_SUCCESS )
        {
            network_suspended = WICED_FALSE;
        }
    }
}

static wiced_result_t send_ping( void )
{
    const uint32_t     ping_timeout = 1000;
    uint32_t           elapsed_ms;
    wiced_result_t     status;
    wiced_ip_address_t ping_target_ip;

    if ( !get_ping_destination( &ping_target_ip ) )
    {
        WPRINT_APP_INFO(("Unknown ping destination.\n"));
        status = WICED_ERROR;
    }
    else
    {
        WPRINT_APP_INFO(("Pinging %u.%u.%u.%u.\n",
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 24) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >> 16) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  8) & 0xFF),
            (unsigned int)((GET_IPV4_ADDRESS(ping_target_ip) >>  0) & 0xFF) ));

        status = wiced_ping( NETWORK_INTERFACE, &ping_target_ip, ping_timeout, &elapsed_ms );
    }

    if ( status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Ping Reply %lums since deep sleep enter %lums\n", elapsed_ms, wiced_deep_sleep_ticks_since_enter( ) ));
    }
    else if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(( "Ping timeout\n" ));
    }
    else
    {
        WPRINT_APP_INFO(( "Ping error: %d\n", (int)status ));
    }

    return WICED_SUCCESS;
}

static wiced_bool_t get_ping_destination( wiced_ip_address_t* ip_address )
{
    return ( wiced_ip_get_gateway_address( NETWORK_INTERFACE, ip_address ) == WICED_SUCCESS ) ? WICED_TRUE : WICED_FALSE;
}

static void print_wlan_log( void )
{
    static char buffer[200];
    (void)wwd_wifi_read_wlan_log( buffer, sizeof( buffer ) );
}
