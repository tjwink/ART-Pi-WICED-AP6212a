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
* Apple iBeacon sample
*
* During initialization the app configures stack to send advertisement packets.
* Non-connectable undirected advertisements.
*
* Features demonstrated
*  - configuring iBeacon advertisements
*
* To demonstrate the app, work through the following steps.
*
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Monitor advertisement packets on over the air sniffer
*
*/
#include <string.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"

#include "ibeacon.h"

/******************************************************************************
 *                               Constants
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                              Variables Definitions
 ******************************************************************************/
/* Variable to hold the iBeacon uuid */
const uint8_t ibeacon_uuid[ LEN_UUID_128 ] = { UUID_IBEACON };

/* Variable to hold the iBeacon proximity type */
const uint8_t ibeacon_type[ LEN_UUID_16 ] = { IBEACON_PROXIMITY };
const uint8_t ibeacon_company_id[ LEN_UUID_16 ] = { IBEACON_COMPANY_ID_APPLE };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

/* Variable to hold the iBeacon proximity type */
const uint8_t ibeacon_major_number[2] = { IBEACON_MAJOR_NUMBER };
const uint8_t ibeacon_minor_number[2] = { IBEACON_MINOR_NUMBER };

/******************************************************************************
 *                             Function Definitions
 ******************************************************************************/
static void ibeacon_start_advertisement( void );
static wiced_result_t ibeacon_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void ibeacon_set_advertisement_data( void );

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{

    wiced_core_init();
    WPRINT_BT_APP_INFO(( "ibeacon_application_start\n" ));

    /* Register the dynamic configurations and */
    /* initialize the stack */
    wiced_bt_stack_init( ibeacon_management_cback,
                        &wiced_bt_cfg_settings,
                        wiced_bt_cfg_buf_pools);
}

/*
 * This function sets the Apple iBeacon advertisement data.
 *  makes the device discoverable and starts the advertisements.
 */
void ibeacon_start_advertisement( void )
{
    wiced_result_t result = 0;

    /* Set advertisement data */
    ibeacon_set_advertisement_data();

    /* Make the device discoverable.
     * The device starts sending non-connectable low duty cycle advertisements
     * for an infinite duration. */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH,
                                             BLE_ADDR_RANDOM, NULL );
    WPRINT_BT_APP_INFO(( "ibeacon_start_advertisement %d\n", result ));
}


/*
 * ibeacon  bt/ble link management callbacks
 */
wiced_result_t ibeacon_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_ble_advert_mode_t          *p_mode;

    WPRINT_BT_APP_INFO(( "ibeacon_management_cback evt: %d\n", event ));

    switch( event )
    {
        /* Bluetooth  stack enabled . Start ibeacon application*/
        case BTM_ENABLED_EVT:
            /* Fill the adv data and start advertisements */
            ibeacon_start_advertisement();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WPRINT_BT_APP_INFO(("Advertisement State Change: %d\n", *p_mode));
            break;

        default:
            break;
    }
    return result;
}

/*
 * This function creates Apple iBeacon advertising data format.
 */
static void ibeacon_set_advertisement_data( void )
{
    /* TX power for advertisement packets , 10dBm */
    uint8_t tx_power_lcl = 10;

    uint8_t ibeacon_data[IBEACON_DATA_LENGTH];
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG|BTM_BLE_BREDR_NOT_SUPPORTED;

    /* first adv element */
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    /* Second adv element */
    adv_elem[num_elem].len          = IBEACON_DATA_LENGTH;
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_MANUFACTURER;
    adv_elem[num_elem].p_data       = ibeacon_data;

    /* Setting Company Identifier */
    ibeacon_data[0] = ibeacon_company_id[0];
    ibeacon_data[1] = ibeacon_company_id[1];

    /* Setting beacon type */
    ibeacon_data[2] = ibeacon_type[0];
    ibeacon_data[3] = ibeacon_type[1];

    /* Setting the ibeacon UUID in the manufacturer data */
    memcpy( &ibeacon_data[4], ibeacon_uuid, LEN_UUID_128 );

    /* Setting the Major field */
    ibeacon_data[20] = ibeacon_major_number[0];
    ibeacon_data[21] = ibeacon_major_number[1];

    /* Setting the Manor field */
    ibeacon_data[22] = ibeacon_minor_number[0];
    ibeacon_data[23] = ibeacon_minor_number[1];

    /* Measured power */
    ibeacon_data[24] = tx_power_lcl;

    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}
