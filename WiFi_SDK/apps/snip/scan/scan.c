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
 * Scan Application
 *
 * Features demonstrated
 *  - WICED scan API
 *
 * This application snippet regularly scans for nearby Wi-Fi access points
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Each time the application scans, a list of Wi-Fi access points in
 *   range is printed to the UART
 *
 */

#include <stdlib.h>
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define DELAY_BETWEEN_SCANS       ( 5000 )

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
    wiced_semaphore_t   semaphore;      /* Semaphore used for signaling scan complete */
    uint32_t            result_count;   /* Count to measure the total scan results    */
} app_scan_data_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_init( );

    while(1)
    {
        wiced_time_t    scan_start_time;
        wiced_time_t    scan_end_time;
        app_scan_data_t scan_data;

        /* Initialize the semaphore that will tell us when the scan is complete */
        wiced_rtos_init_semaphore(&scan_data.semaphore);
        scan_data.result_count = 0;
        WPRINT_APP_INFO( ( "Waiting for scan results...\n" ) );
        WPRINT_APP_INFO( ("  # Type  BSSID              RSSI Rate Chan Security               SSID                            CCode    Flag\n" ) );
        WPRINT_APP_INFO( ("------------------------------------------------------------------------------------------------------------------\n" ) );
        /* Start the scan */
        wiced_time_get_time(&scan_start_time);
        wiced_wifi_scan_networks(scan_result_handler, &scan_data );

        /* Wait until scan is complete */
        wiced_rtos_get_semaphore(&scan_data.semaphore, WICED_WAIT_FOREVER);
        wiced_time_get_time(&scan_end_time);

        WPRINT_APP_INFO( ("\nScan complete in %lu milliseconds\n", (unsigned long )(scan_end_time - scan_start_time) ) );

        /* Clean up */
        wiced_rtos_deinit_semaphore(&scan_data.semaphore);

        /* Issuing next scan after some delay (optional) */
        wiced_rtos_delay_milliseconds(DELAY_BETWEEN_SCANS);
    }
}

/*
 * Callback function to handle scan results
 */
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    /* Validate the input arguments */
    wiced_assert("Bad args", malloced_scan_result != NULL);

    if ( malloced_scan_result != NULL )
    {
        app_scan_data_t* scan_data  = (app_scan_data_t*)malloced_scan_result->user_data;

        malloc_transfer_to_curr_thread( malloced_scan_result );

        if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
        {
            wiced_scan_result_t* record = &malloced_scan_result->ap_details;

            WPRINT_APP_INFO( ( "%3ld ", scan_data->result_count ) );
            print_scan_result(record);
            scan_data->result_count++;
        }
        else
        {
            wiced_rtos_set_semaphore( &scan_data->semaphore );
        }

        free( malloced_scan_result );
    }
    return WICED_SUCCESS;
}
