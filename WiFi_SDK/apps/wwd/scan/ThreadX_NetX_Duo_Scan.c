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

/**
 * @file
 *
 * Wi-Fi Scan Application using ThreadX/NetX
 *
 * This application scans for Wi-Fi Networks in range and prints
 * scan results to a console. A new scan is initiated at 5 second
 * intervals.
 *
 * The application works with ThreadX / NetX
 *
 */

#include "tx_api.h"
#include "tx_thread.h"
#include "nx_api.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "network/wwd_network_constants.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_network.h"
#include "wwd_structures.h"
#include "wiced_utilities.h"

/******************************************************
 *                      Macros
 ******************************************************/
/** @cond */

#define NUM_BUFFERS_POOL_SIZE(x)       ((WICED_LINK_MTU_ALIGNED+sizeof(NX_PACKET)+1)*(x))
/** @endcond */

/******************************************************
 *                    Constants
 ******************************************************/

#define COUNTRY                       WICED_COUNTRY_AUSTRALIA
#define CIRCULAR_RESULT_BUFF_SIZE     (40)
#define APP_STACK_SIZE                (2048)
#define APP_TX_BUFFER_POOL_SIZE       NUM_BUFFERS_POOL_SIZE(2)
#define APP_RX_BUFFER_POOL_SIZE       NUM_BUFFERS_POOL_SIZE(2)

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
static void scan_results_handler( wiced_scan_result_t** result_ptr, void* user_data, wiced_scan_status_t status );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static TX_THREAD               app_main_thread_handle;
static char                    app_main_thread_stack [APP_STACK_SIZE];
static char                    tx_buffer_pool_memory [APP_TX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES];
static char                    rx_buffer_pool_memory [APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES];
static NX_PACKET_POOL          nx_pools[2]; /* 0=TX, 1=RX */
static wiced_mac_t             bssid_list[200]; /* List of BSSID (AP MAC addresses) that have been scanned */
static host_semaphore_type_t   num_scan_results_semaphore;
static wiced_scan_result_t     result_buff[CIRCULAR_RESULT_BUFF_SIZE];
static uint16_t                result_buff_write_pos = 0;
static uint16_t                result_buff_read_pos  = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * Main Scan app
 *
 * Initialises NetX, Wiced, then scans for wireless networks
 *
 * @param thread_input : Unused parameter - required to match thread prototype
 */
static void app_main_thread( ULONG thread_input )
{
    int                  record_number;
    wiced_scan_result_t* record;
    wiced_scan_result_t* result_ptr = (wiced_scan_result_t *) &result_buff;
    wwd_result_t         result;

    WPRINT_APP_INFO(("\nPlatform " PLATFORM " initialised\n"));
    WPRINT_APP_INFO(("Started ThreadX " ThreadX_VERSION "\n"));

    /* Compile-time checks */
    wiced_static_assert(packet_header_not_cache_aligned, sizeof(NX_PACKET) == PLATFORM_L1_CACHE_ROUND_UP(sizeof(NX_PACKET)));

    /* Initialize the NetX system.  */
    WPRINT_APP_INFO(("Initialising NetX-Duo " NetX_Duo_VERSION "\n"));
    nx_system_initialize( );

    /* Create packet pools for transmit and receive */
    WPRINT_APP_INFO(("Creating Packet pools\n"));
    if ( NX_SUCCESS != nx_packet_pool_create( &nx_pools[0], "",
        WICED_LINK_MTU_ALIGNED, PLATFORM_L1_CACHE_PTR_ROUND_UP(tx_buffer_pool_memory), APP_TX_BUFFER_POOL_SIZE ) )
    {
        WPRINT_APP_ERROR(("Couldn't create TX packet pool\n"));
        return;
    }
    if ( NX_SUCCESS != nx_packet_pool_create( &nx_pools[1], "",
        WICED_LINK_MTU_ALIGNED, PLATFORM_L1_CACHE_PTR_ROUND_UP(rx_buffer_pool_memory), APP_RX_BUFFER_POOL_SIZE ) )
    {
        WPRINT_APP_ERROR(("Couldn't create RX packet pool\n"));
        return;
    }

    /* Initialise Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( &nx_pools );
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    host_rtos_init_semaphore( &num_scan_results_semaphore );

    /* Repeatedly Scan forever */
    while ( 1 )
    {
        record_number = 1;

        /* Clear list of BSSID addresses already parsed */
        memset( &bssid_list, 0, sizeof( bssid_list ) );

        /* Start Scan */
        WPRINT_APP_INFO(("Starting Scan\n"));

        /* Examples of scan parameter values */
#if 0
        wiced_ssid_t ssid                       = { 12,  "YOUR_AP_SSID" };           /* Scan for networks named YOUR_AP_SSID only */
        wiced_mac_t  mac                        = {{0x00,0x1A,0x30,0xB9,0x4E,0x90}}; /* Scan for Access Point with the specified BSSID */
        uint16_t     chlist[]                   = { 6, 0 };                          /* Scan channel 6 only */
        wiced_scan_extended_params_t extparam = { 5, 1000, 1000, 100 };            /* Spend much longer scanning (1 second and 5 probe requests per channel) */
#endif /* if 0 */

        result_buff_read_pos = 0;
        result_buff_write_pos = 0;

        if ( WWD_SUCCESS != wwd_wifi_scan( WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_ANY, NULL, NULL, NULL, NULL, scan_results_handler, (wiced_scan_result_t **) &result_ptr, NULL, WWD_STA_INTERFACE ) )
        {
            WPRINT_APP_ERROR(("Error starting scan\n"));
            return;
        }

        WPRINT_APP_INFO(("Waiting for scan results...\n"));

        while ( host_rtos_get_semaphore( &num_scan_results_semaphore, NEVER_TIMEOUT, WICED_FALSE ) == WWD_SUCCESS )
        {
            int k;

            record = &result_buff[result_buff_read_pos];
            if ( record->channel == 0xff )
            {
                /* Scan completed */
                break;
            }

            /* Print SSID */
            WPRINT_APP_INFO(("\n#%03d SSID          : ", record_number ));
            for ( k = 0; k < record->SSID.length; k++ )
            {
                WPRINT_APP_INFO(("%c",record->SSID.value[k]));
            }
            WPRINT_APP_INFO(("\n" ));

            wiced_assert( "error", (record->bss_type==WICED_BSS_TYPE_INFRASTRUCTURE)||(record->bss_type==WICED_BSS_TYPE_ADHOC));

            /* Print other network characteristics */
            WPRINT_APP_INFO(("     BSSID         : %02X:%02X:%02X:%02X:%02X:%02X\n", record->BSSID.octet[0], record->BSSID.octet[1], record->BSSID.octet[2], record->BSSID.octet[3], record->BSSID.octet[4], record->BSSID.octet[5] ));
            WPRINT_APP_INFO(("     RSSI          : %ddBm", record->signal_strength ));
            if ( record->flags & WICED_SCAN_RESULT_FLAG_RSSI_OFF_CHANNEL )
            {
                WPRINT_APP_INFO((" (off-channel)\n"));
            }
            else
            {
                WPRINT_APP_INFO(("\n"));
            }
            WPRINT_APP_INFO(("     Max Data Rate : %.1f Mbits/s\n", (float)record->max_data_rate /1000.0));
            WPRINT_APP_INFO(("     Network Type  : %s\n", (record->bss_type==WICED_BSS_TYPE_INFRASTRUCTURE)?"Infrastructure":(record->bss_type==WICED_BSS_TYPE_ADHOC)?"Ad hoc":"Unknown" ));
            WPRINT_APP_INFO(("     Security      : %s\n", (record->security==WICED_SECURITY_OPEN)?"Open":
                                                      (record->security==WICED_SECURITY_WEP_PSK)?"WEP":
                                                      (record->security==WICED_SECURITY_WPA_TKIP_PSK)?"WPA":
                                                      (record->security==WICED_SECURITY_WPA2_AES_PSK)?"WPA2 AES":
                                                      (record->security==WICED_SECURITY_WPA2_MIXED_PSK)?"WPA2 Mixed":"Unknown" ));
            WPRINT_APP_INFO(("     Radio Band    : %s\n", (record->band==WICED_802_11_BAND_5GHZ)?"5GHz":"2.4GHz" ));
            WPRINT_APP_INFO(("     Channel       : %d\n", record->channel ));
            result_buff_read_pos++;
            if ( result_buff_read_pos >= CIRCULAR_RESULT_BUFF_SIZE )
            {
                result_buff_read_pos = 0;
            }
            record_number++;

        }

        /* Done! */
        WPRINT_APP_INFO(("\nEnd of scan results - next scan in 5 seconds\n"));

        /* Wait 5 seconds till next scan */
        host_rtos_delay_milliseconds( 5000 );
    }
}

/**
 *  Main function - starts ThreadX
 *  Called from the crt0 _start function
 *
 */
int main( )
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter( );
    return 0;
}

/**
 *  Application Define function - creates and starts the application thread
 *  Called by ThreadX whilst it is initialising
 *
 *  @param first_unused_memory: unused parameter - required to match prototype
 */
void tx_application_define( void *first_unused_memory )
{

    /* Create the application thread.  */
    tx_thread_create( &app_main_thread_handle, "app thread", app_main_thread, 0, app_main_thread_stack, APP_STACK_SIZE, 4, 4, TX_NO_TIME_SLICE, TX_AUTO_START );

}

/**
 *  Scan result callback
 *  Called whenever a scan result is available
 *
 *  @param result_ptr : pointer to pointer for location where result is stored. The inner pointer
 *                      can be updated to cause the next result to be put in a new location.
 *  @param user_data : unused
 */
static void scan_results_handler( wiced_scan_result_t** result_ptr, void* user_data, wiced_scan_status_t status )
{
    if ( result_ptr == NULL )
    {
        /* finished */
        result_buff[result_buff_write_pos].channel = 0xff;
        host_rtos_set_semaphore( &num_scan_results_semaphore, WICED_FALSE );
        return;
    }

    wiced_scan_result_t* record = ( *result_ptr );

    /* Check the list of BSSID values which have already been printed */
    wiced_mac_t * tmp_mac = bssid_list;
    while ( NULL_MAC( tmp_mac->octet ) == WICED_FALSE )
    {
        if ( CMP_MAC( tmp_mac->octet, record->BSSID.octet ) == WICED_TRUE )
        {
            /* already seen this BSSID */
            return;
        }
        tmp_mac++;
    }

    /* New BSSID - add it to the list */
    memcpy( &tmp_mac->octet, record->BSSID.octet, sizeof(wiced_mac_t) );

    /* Add the result to the list and set the pointer for the next result */
    result_buff_write_pos++;
    if ( result_buff_write_pos >= CIRCULAR_RESULT_BUFF_SIZE )
    {
        result_buff_write_pos = 0;
    }
    *result_ptr = &result_buff[result_buff_write_pos];
    host_rtos_set_semaphore( &num_scan_results_semaphore, WICED_FALSE );

    wiced_assert( "Circular result buffer overflow", result_buff_write_pos != result_buff_read_pos );
}

