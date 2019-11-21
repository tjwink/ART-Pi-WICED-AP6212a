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
 * BLE Proximity Reporter Sample Application
 *
 * Features demonstrated
 *  -Proximity Reporter implementation for details refer to BT SIG Proximity Profile 1.0 spec
 *
 * On startup this demo:
 *  - Initializes the Proximity reporter GATT database
 *  - Begins advertising
 *  - Waits for GATT clients to connect
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer
 * 2. Build and download the application (to the WICED board)
 * 3. On application start the device acts as a GATT server and advertises itself as Proximity Reporter
 * 4. Connect to Proximity reporter using one of the LE clients (LEExplorer(android)) or (BLE Utility(Apple Store))
 * 5. Once connected the client can read/write Link Level and Alert levels
 *
 */


#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_gatt_db.h"
#include "ble_proximity_reporter.h"

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
 *               Function Declarations
 ******************************************************/

static void                   ble_proximity_reporter_init      ( void );
static wiced_bt_dev_status_t  ble_proximity_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   ble_proximity_tx_power_callback  ( wiced_bt_tx_power_result_t *p_tx_power );
static wiced_bt_gatt_status_t ble_proximity_gatt_write_request ( wiced_bt_gatt_write_t *p_write_request );
static wiced_bt_gatt_status_t ble_proximity_gatt_read_request  ( wiced_bt_gatt_read_t *p_read_request );
static wiced_bt_gatt_status_t ble_proximity_gatt_cback         ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* Proximity reporter attribute values */
static uint8_t  proximity_immediate_alert_level;
static uint8_t  proximity_link_loss_alert_level;
static int8_t   proximity_tx_power_level;

/* GATT attrIbute values */
static uint32_t proximity_gatt_attribute_service_changed = 0;
static uint16_t proximity_gatt_generic_access_appearance = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    /* Initialize WICED platform */
    wiced_init( );

    /* Initialize Bluetooth controller and host stack */
    wiced_bt_stack_init( ble_proximity_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

/* TX Power report handler */
static void ble_proximity_tx_power_callback( wiced_bt_tx_power_result_t *p_tx_power )
{
    if ( ( p_tx_power->status == WICED_BT_SUCCESS ) && ( p_tx_power->hci_status == HCI_SUCCESS ) )
    {
        WPRINT_BT_APP_INFO( ("Local TX power: %i\n", p_tx_power->tx_power) );
        proximity_tx_power_level = p_tx_power->tx_power;
    }
    else
    {
        WPRINT_BT_APP_INFO( ("Unable to read Local TX power. (btm_status=0x%x, hci_status=0x%x)\n", p_tx_power->status, p_tx_power->hci_status) );
        proximity_tx_power_level = 0;
    }
}

/* Bluetooth management event handler */
static wiced_bt_dev_status_t ble_proximity_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda;

    switch ( event )
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth controller and host stack enabled */
            WPRINT_BT_APP_INFO( ("Bluetooth enabled (%s)\n", ((p_event_data->enabled.status == WICED_BT_SUCCESS) ? "success":"failure")) );

            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
                wiced_bt_dev_read_local_addr( bda );
                WPRINT_BT_APP_INFO( ("Local Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]) );

                /* Enable proximity reporter */
                ble_proximity_reporter_init( );
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WPRINT_BT_APP_INFO( ("Security reqeust\n") );
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WPRINT_BT_APP_INFO( ("Pairing IO Capabilities reqeust\n") );
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE; /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND; /* Bonding required */
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WPRINT_BT_APP_INFO( ("Pairing complete %i.\n", p_event_data->pairing_complete.pairing_complete_info.ble.status) );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* adv state change callback */
            WPRINT_BT_APP_INFO( ("---->>> New ADV state: %d\n", p_event_data->ble_advert_state_changed) );
            break;

        default:
            WPRINT_BT_APP_INFO( ("Unhandled Bluetooth Management Event: 0x%x\n", event) );
            break;
    }

    return ( status );
}

/* Handler for attrubute write requests */
static wiced_bt_gatt_status_t ble_proximity_gatt_write_request( wiced_bt_gatt_write_t *p_write_request )
{
    uint8_t attribute_value = *(uint8_t *) p_write_request->p_val;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch ( p_write_request->handle )
    {
        case HDLC_LINK_LOSS_ALERT_LEVEL_VALUE:
            proximity_link_loss_alert_level = attribute_value;
            WPRINT_BT_APP_INFO( ("Link loss alert level changed to: %i\n", attribute_value) );
            break;

        case HDLC_IMMEDIATE_ALERT_LEVEL_VALUE:
            proximity_immediate_alert_level = attribute_value;
            WPRINT_BT_APP_INFO( ("Proximity alert (level: %i)\n", attribute_value) );
            break;

        default:
            WPRINT_BT_APP_INFO( ("Write request to invalid handle: 0x%x\n", p_write_request->handle) );
            status = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
    }

    return ( status );
}

/* Handler for attrubute read requests */
static wiced_bt_gatt_status_t ble_proximity_gatt_read_request( wiced_bt_gatt_read_t *p_read_request )
{
    wiced_bt_gatt_status_t status   = WICED_BT_GATT_SUCCESS;
    uint16_t attribute_value_length = 0;
    void*    p_attribute_value_source;

    switch ( p_read_request->handle )
    {
        case HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE:
            p_attribute_value_source = &proximity_gatt_attribute_service_changed;
            attribute_value_length   = sizeof( proximity_gatt_attribute_service_changed );
            break;

        case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
            p_attribute_value_source = (void *) wiced_bt_cfg_settings.device_name;
            attribute_value_length   = strlen( (char *) wiced_bt_cfg_settings.device_name );
            break;

        case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
            p_attribute_value_source = &proximity_gatt_generic_access_appearance;
            attribute_value_length   = sizeof( proximity_gatt_generic_access_appearance );
            break;

        case HDLC_TX_POWER_LEVEL_VALUE:
            p_attribute_value_source = &proximity_tx_power_level;
            attribute_value_length   = sizeof( proximity_tx_power_level );
            break;

        case HDLC_LINK_LOSS_ALERT_LEVEL_VALUE:
            p_attribute_value_source = &proximity_link_loss_alert_level;
            attribute_value_length   = sizeof( proximity_link_loss_alert_level );
            break;

        case HDLC_IMMEDIATE_ALERT_LEVEL_VALUE:
            p_attribute_value_source = &proximity_immediate_alert_level;
            attribute_value_length   = sizeof( proximity_immediate_alert_level );
            break;

        default:
            status = WICED_BT_GATT_READ_NOT_PERMIT;
            WPRINT_BT_APP_INFO( ("Read request to invalid handle: 0x%x\n", p_read_request->handle) );
            break;
    }

    /* Validate destination buffer size */
    if ( attribute_value_length > *p_read_request->p_val_len )
    {
        *p_read_request->p_val_len = attribute_value_length;
    }

    /* Copy the attribute value */
    if ( attribute_value_length )
    {
        memcpy( p_read_request->p_val, p_attribute_value_source, attribute_value_length );
    }

    /* Indicate number of bytes copied */
    *p_read_request->p_val_len = attribute_value_length;

    return ( status );
}

/* GATT event handler */
static wiced_bt_gatt_status_t ble_proximity_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t *bda;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            /* GATT connection status change */
            bda = p_event_data->connection_status.bd_addr;
            WPRINT_BT_APP_INFO( ("GATT connection to [%02X:%02X:%02X:%02X:%02X:%02X] %s.\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], (p_event_data->connection_status.connected ? "established" : "released")) );

            if ( p_event_data->connection_status.connected )
            {
                /* Connection established. Get current TX power  (required for setting TX power attribute in GATT database) */
                wiced_bt_dev_read_tx_power( p_event_data->connection_status.bd_addr, p_event_data->connection_status.transport, (wiced_bt_dev_cmpl_cback_t *) ble_proximity_tx_power_callback );

                /* Disable connectability. */
                wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
            }
            else
            {
                /* Connection released. Re-enable BLE connectability. */
                wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
                /* test code for directed adv */
                /* wiced_bt_start_advertisements (BTM_BLE_ADVERT_DIRECTED_HIGH, 0, p_event_data->connection_status.bd_addr); */
                WPRINT_BT_APP_INFO( ("Waiting for proximity monitor to connect...\n") );
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            /* GATT attribute read/write request */
            if ( p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE )
            {
                status = ble_proximity_gatt_write_request( &p_event_data->attribute_request.data.write_req );
            }
            else if ( p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_READ )
            {
                status = ble_proximity_gatt_read_request( &p_event_data->attribute_request.data.read_req );
            }
            break;

        default:
            break;
    }

    return ( status );
}

/* Initialize Proximity Reporter */
static void ble_proximity_reporter_init( void )
{
    UINT8    raw_bytes[31], *p = raw_bytes, num_elem = 0;
    wiced_bt_ble_advert_elem_t adv_elem[2];

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    raw_bytes[num_elem]             = 0x02;
    adv_elem[num_elem].p_data       = p;

    p++;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = wiced_bt_cfg_settings.device_name;
    num_elem ++;

    /* Set advertising data: device name and discoverable flag */
    wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    /* Register for gatt event notifications */
    wiced_bt_gatt_register( &ble_proximity_gatt_cback );

    /* Initialize GATT database */
    wiced_bt_gatt_db_init( (uint8_t *) gatt_db, gatt_db_size );

    /* Enable Bluetooth LE advertising and connectability */

    /* start LE advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WPRINT_BT_APP_INFO( ("Waiting for proximity monitor to connect...\n") );
}
