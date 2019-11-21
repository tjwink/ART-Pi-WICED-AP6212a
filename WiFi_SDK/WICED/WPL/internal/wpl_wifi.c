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
 */

/**
 * Define CPL functions
 */

#include "wiced.h"
#include "wwd_wifi.h"
#include "wpl_cpl.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define AMPDU_DUMP_SIZE             200

#define AMPDU_NO_DUMP_STRING_SIZE    50

#define WPL_MAX_RATE_COUNT           (EVENT_DESC_WIFI_MAX - EVENT_DESC_WIFI_RATE0)


uint32_t wpl_wifi_parse_ampdu_dump( uint8_t *ampdu_dump_buf, uint32_t *tx_rx_rate_data )
{
    char *tx_mcs_rate_str, *rx_mcs_rate_str;
    char *tx_vhtrate_str, *rx_vhtrate_str;
    char *tx_rate_str, *rx_rate_str;
    char *tempPtr, *tempPtr1;
    char tempPtr2[10];
    uint32_t tx_rate_count = 0, rx_rate_count = 0;

    tx_mcs_rate_str = strstr( ( char * )ampdu_dump_buf, "TX MCS" );
    rx_mcs_rate_str = strstr( ( char * )ampdu_dump_buf, "RX MCS" );

    tx_vhtrate_str = strstr( ( char * )ampdu_dump_buf, "TX VHT" );
    rx_vhtrate_str = strstr( ( char * )ampdu_dump_buf, "RX VHT" );

    sscanf( tx_mcs_rate_str,"%[^\n]", tx_mcs_rate_str );
    sscanf( rx_mcs_rate_str,"%[^\n]", rx_mcs_rate_str );
    sscanf( tx_vhtrate_str,"%[^\n]", tx_vhtrate_str );
    sscanf( rx_vhtrate_str,"%[^\n]", rx_vhtrate_str );

    if ( strlen(tx_vhtrate_str)  > (strlen(tx_mcs_rate_str) ) )
    {
        tx_rate_str = tx_vhtrate_str;
        rx_rate_str = rx_vhtrate_str;
    }
    else
    {
        tx_rate_str = tx_mcs_rate_str;
        rx_rate_str = rx_mcs_rate_str;
    }

    //Parse the ampdu dump to extract the TX data
    tx_rate_str = strstr( tx_rate_str, "  ");

    tempPtr = strtok( tx_rate_str, "  " );
    while ( tempPtr != NULL ) {
        tempPtr = strtok( NULL, "  " );

        if ( tempPtr != NULL )
        {
            int idx = 0;
            tempPtr1 = tempPtr;
            while ( strncmp( tempPtr1, "(", 1 ) ) {
                tempPtr2[idx] = *tempPtr1;
                idx++;
                tempPtr1++;
            }
            tempPtr2[idx] = 0;

            tx_rx_rate_data[tx_rate_count] = ( uint32_t )atoi( tempPtr2 ) << 16;
            tx_rate_count++;
        }
    }

    //Parse the ampdu dump to extract the RX data
    rx_rate_str = strstr( rx_rate_str,"  " );

    tempPtr = strtok( rx_rate_str, "  " );
    while ( tempPtr != NULL ) {
        tempPtr = strtok( NULL, "  " );

        if ( tempPtr != NULL )
        {
            int idx = 0;
            tempPtr1 = tempPtr;
            while(strncmp( tempPtr1, "(", 1 ) ) {
                tempPtr2[idx] = *tempPtr1;
                idx++;
                tempPtr1++;
            }
            tempPtr2[idx] = 0;

            tx_rx_rate_data[rx_rate_count] |= ( uint32_t )atoi( tempPtr2 );
            rx_rate_count++;
        }
    }

    return tx_rate_count;
}

uint32_t wpl_wifi_get_data_from_ampdu_dump( wwd_interface_t interface, uint32_t *tx_rx_rate_data )
{
    uint8_t *ampdu_dump_buf;
    char *ampdu_cmd = "ampdu";
    uint32_t ret, rate_count;


    rate_count = 0;

    //Get number of packets per each data rate
     ampdu_dump_buf = calloc( AMPDU_DUMP_SIZE, sizeof(char) );
     if ( ampdu_dump_buf == NULL ) {
         WPRINT_APP_INFO( ( "Failed to allocate dump buffer of %d bytes\n", AMPDU_DUMP_SIZE ) );
         goto exit;
     }

    ret = wwd_get_dump( interface, ampdu_dump_buf, AMPDU_DUMP_SIZE, ampdu_cmd, (uint16_t)strlen(ampdu_cmd) );

    if ( ret ) {
        WPRINT_APP_INFO( ( "dump failed with error : %d, make sure that FW supports this particular dump feature\n", ( int16_t )ret ) );
        goto exit;
    }

    WPRINT_APP_DEBUG( ( "%s\n", ampdu_dump_buf ) );

    if ( strlen( (char *)ampdu_dump_buf ) < AMPDU_NO_DUMP_STRING_SIZE)
        goto exit;

    //Dump is collected, now clear it
   if ( wwd_ampdu_clear_dump( interface ) )
   {
       WPRINT_APP_INFO( ( "WPL: Failed to clear ampdu dump\n" ) );
       goto exit;
   }

    rate_count = wpl_wifi_parse_ampdu_dump( ampdu_dump_buf, tx_rx_rate_data );

exit:
    free( ampdu_dump_buf );
    return rate_count;
}

void wpl_wifi_get_power_data(void)
{
    uint32_t band = 0xFFFFFFFF;
    uint32_t bandwidth = 0xFFFFFFFF;
    uint32_t cnt, pm_mode, rate_count;
    wiced_interface_t interface;
    uint32_t tx_rx_rate_data[WPL_MAX_RATE_COUNT] = { 0 };
    uint8_t rate_index = EVENT_DESC_WIFI_RATE0;

    wiced_get_default_ready_interface( &interface );

    if (!wiced_network_is_ip_up( interface ) )
    {
        WPRINT_APP_DEBUG( ( "Interface : %d not up\n", interface ) );
        return;
    }

    //Get Wi-Fi PM mode

    if ( wwd_wifi_get_pm_mode( interface, &pm_mode ) )
    {
       WPRINT_APP_INFO( ( "WPL: Couldn't get PM mode \n" ) );
       return;
    }
    cpl_log_update( EVENT_PROC_ID_WIFI, EVENT_ID_WIFI_DATA, EVENT_DESC_WIFI_PMMODE, pm_mode );

    //Get band
    if ( wwd_wifi_get_current_band( interface, &band ) )
    {
        WPRINT_APP_INFO( ( "WPL: Couldn't get band \n" ) );
        return;
    }
    cpl_log_update( EVENT_PROC_ID_WIFI, EVENT_ID_WIFI_DATA, EVENT_DESC_WIFI_BAND, band );

    //Get bandwidth
    if ( wwd_wifi_get_bw( &bandwidth ) )
    {
        WPRINT_APP_INFO( ( "WPL: Couldn't get Bandwidth \n" ) );
        return;
    }

    cpl_log_update( EVENT_PROC_ID_WIFI, EVENT_ID_WIFI_DATA, EVENT_DESC_WIFI_BW, bandwidth );

    rate_count = wpl_wifi_get_data_from_ampdu_dump ( interface, tx_rx_rate_data );

    cpl_log_update( EVENT_PROC_ID_WIFI, EVENT_ID_WIFI_DATA, EVENT_DESC_WIFI_RATE_TYPE, rate_count );

    for ( cnt = 0 ;  cnt < rate_count ; cnt++) {
            WPRINT_APP_DEBUG( ( "tx_rx_rate_data[%u]:0x%08X\n", (unsigned int)cnt,(unsigned int)tx_rx_rate_data[cnt] ) );
            cpl_log_update( EVENT_PROC_ID_WIFI, EVENT_ID_WIFI_DATA, rate_index++, tx_rx_rate_data[cnt] );
       }
}
