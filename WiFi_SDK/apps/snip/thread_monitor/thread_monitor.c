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
 * Thread Monitor Application
 *
 * This application demonstrates how to use the WICED
 * System Monitor API to monitor the operation of the
 * application thread.
 *
 * Features demonstrated
 *  - WICED System Monitor
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   After the download completes, the app:
 *    - Registers a thread monitor with the WICED system monitor
 *    - Loops 10 times, each time notifying the system monitor that
 *      the thread is working normally
 *    - An unexpected delay is then simulated
 *    - A short time AFTER the unexpected delay occurs, the
 *      watchdog bites and the WICED eval board reboots
 *
 *  The watchdog timeout period is set in the thread_monitor.mk makefile
 *  using the global variable : APPLICATION_WATCHDOG_TIMEOUT_SECONDS
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAXIMUM_ALLOWED_INTERVAL_BETWEEN_MONITOR_UPDATES (1000*MILLISECONDS)
#define EXPECTED_WORK_TIME                               (MAXIMUM_ALLOWED_INTERVAL_BETWEEN_MONITOR_UPDATES - (100*MILLISECONDS))
#define UNEXPECTED_DELAY                                 (200*MILLISECONDS)


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

static void do_work( void );
static void update_my_thread_monitor( uint32_t count );

/******************************************************
 *               Variable Definitions
 ******************************************************/

wiced_system_monitor_t my_thread_monitor;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    uint32_t a = 0;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Register a thread monitor to keep a watchful eye on my thread processing time */
    WPRINT_APP_INFO( ( "\r\nRegistering my thread monitor\n\n" ) );
    wiced_register_system_monitor( &my_thread_monitor, MAXIMUM_ALLOWED_INTERVAL_BETWEEN_MONITOR_UPDATES );

    /* Do some work and then update my thread monitor.
     * This demonstrates normal behaviour for the thread.
     */
    while (1)
    {
        do_work();
        update_my_thread_monitor( a++ );

        if (a == 10)
        {
            /* Add an unexpected delay. This is abnormal behaviour for the thread */
            wiced_rtos_delay_milliseconds( UNEXPECTED_DELAY );
            WPRINT_APP_INFO( ( "Uh oh, I'm about to watchdog because an unexpected delay occurred!\n\n" ) );
        }
    }
}


static void do_work( void )
{
    WPRINT_APP_INFO( ( "Do some work\n" ) );
    wiced_rtos_delay_milliseconds( EXPECTED_WORK_TIME );  /* 'work' is simply a waste of time for this demo! */
}


static void update_my_thread_monitor( uint32_t count )
{
    WPRINT_APP_INFO( ( "Updating monitor: %d\n\n", (int)count ) );
    wiced_update_system_monitor( &my_thread_monitor, MAXIMUM_ALLOWED_INTERVAL_BETWEEN_MONITOR_UPDATES );
}
