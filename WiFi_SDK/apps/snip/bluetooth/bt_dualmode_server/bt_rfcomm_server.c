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
 * RFCOMM Server Sample Application
 *
 */

#include "wiced_bt_cfg.h"
#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_cfg.h"
#include "bt_rfcomm_server.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* Mask of RFCOMM events handled by app callback */
#define BT_RFCOMM_SERVER_EVT_MASK   ( ( wiced_bt_rfcomm_port_event_t)(PORT_EV_FC | PORT_EV_FCS | PORT_EV_RXCHAR | \
                                            PORT_EV_TXEMPTY | PORT_EV_CTS | PORT_EV_DSR | \
                                            PORT_EV_RING | PORT_EV_CTSS | PORT_EV_DSRS ) )

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
void bt_rfcomm_server_init( void );
static void bt_rfcomm_server_log_data( uint8_t *p_data, uint16_t len );

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint16_t bt_rfcomm_server_handle;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* RFCOMM connection management callback callback */
void bt_rfcomm_server_conn_cback( wiced_bt_rfcomm_result_t code, uint16_t port_handle )
{
    if ( code == WICED_BT_RFCOMM_SUCCESS )
    {
        /* RFCOMM connection established. Send test string to remote device */
        WPRINT_BT_APP_INFO( ( "RFCOMM connection established.\n" ) );
    }
    else if ( code == WICED_BT_RFCOMM_CLOSED )
    {
        WPRINT_BT_APP_INFO( ( "RFCOMM connection closed.\n" ) );
    }
    else
    {
        WPRINT_BT_APP_INFO( ( "%s unhandled code=0x%x\n", __FUNCTION__, code ) );
    }
}

/* RFCOMM port event callback */
void bt_rfcomm_server_evt_cback( wiced_bt_rfcomm_port_event_t event, uint16_t port_handle )
{
    WPRINT_BT_APP_INFO( ( "bt_rfcomm_server_evt_cback event mask=0x%04X\n", (int)event ) );
}

/* RFCOMM Data RX callback */
int bt_rfcomm_server_data_cback( uint16_t port_handle, void *p_data, uint16_t len )
{
    uint16_t len_sent;
    char echo[]  = " echo from server: ";
    int i;

    WPRINT_BT_APP_INFO( ( "RFCOMM RX (len=%i)\n", len ) );
    bt_rfcomm_server_log_data ( p_data, len );

    /* Echo back to client */
    WPRINT_BT_APP_INFO( ( "RFCOMM TX (len=%i)\n", len ) );
    bt_rfcomm_server_log_data ( p_data, len );

    /* wiced_bt_rfcomm_write_data(port_handle, (char *)p_data, len, &len_sent); */

    wiced_bt_rfcomm_write_data( port_handle, (char *)echo, sizeof(echo), &len_sent );
    for(i = 0; i < len; i++){
        ( (char *)p_data )[i]++;
    }
    wiced_bt_rfcomm_write_data( port_handle, (char *)p_data, len, &len_sent );
    bt_rfcomm_server_log_data ( p_data, len );

    return 0;
}

/* Log data */
static void bt_rfcomm_server_log_data( uint8_t *p_data, uint16_t len )
{
    uint16_t i, j, offset = 0;

    /* Display incoming data */
    while ( len > 0 )
    {
        WPRINT_BT_APP_INFO( ( "   %04X: ", offset ) );

        j = 16;
        if ( len < 16 )
            j = len;

        for ( i=0; i<j; i++ )
        {
            WPRINT_BT_APP_INFO( ( "%02X ", ((uint8_t *)p_data)[offset+i] ) );
        }

        WPRINT_BT_APP_INFO( ( "\n" ) );
        offset += j;
        len -= j;
    }
}

/* Initialize the RFCOMM server */
void bt_rfcomm_server_init( void )
{
    int result = 0;
    wiced_bt_device_address_t bd_addr = { 0 };
    /* Initialize SDP server database for rfcble_app */
    wiced_bt_sdp_db_init( (UINT8 *)wiced_bt_sdp_db, wiced_bt_sdp_db_size );

    /* Create RFCOMM server connection */
    result = wiced_bt_rfcomm_create_connection( UUID_SERVCLASS_SERIAL_PORT,
                BT_RFCOMM_SERVER_APP_SCN,
                TRUE,
                BT_RFCOMM_SERVER_APP_MTU,
                bd_addr,
                &bt_rfcomm_server_handle,
                bt_rfcomm_server_conn_cback );

    WPRINT_BT_APP_INFO(("rfcomm create_connection result:%d\n", result ));

    /* Set data callback RFCOMM */
    wiced_bt_rfcomm_set_data_callback( bt_rfcomm_server_handle, bt_rfcomm_server_data_cback );

    /* Set event callback RFCOMM, and events to be notified of */
    wiced_bt_rfcomm_set_event_mask( bt_rfcomm_server_handle, BT_RFCOMM_SERVER_EVT_MASK );
    wiced_bt_rfcomm_set_event_callback( bt_rfcomm_server_handle, bt_rfcomm_server_evt_cback );

    /* Enable connectablity (use default connectablity window/interval) */
    wiced_bt_dev_set_connectability ( BTM_CONNECTABLE, 0, 0 );

    /* Enable discoverablity (use default discoverablity window/interval) */
    wiced_bt_dev_set_discoverability ( BTM_GENERAL_DISCOVERABLE, 0, 0 );

    WPRINT_BT_APP_INFO( ( "Waiting for RFCOMM connection (scn=%i)...\n", BT_RFCOMM_SERVER_APP_SCN ) );

}
