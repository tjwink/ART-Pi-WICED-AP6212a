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
 * Proximity Reporter Sample Application (GATT Server database)
 *
 */
#include "wiced_bt_gatt.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "wiced_bt_cfg.h"
#include "ble_proximity_reporter.h"
#include "wiced_bt_gatt_db.h"

/* GATT database */
const uint8_t gatt_db[] =
{
    // Generic Attribute service
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ATTRIBUTE, UUID_SERVCLASS_GATT_SERVER),

    CHARACTERISTIC_UUID16 (HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED,
                           HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE,
                           GATT_UUID_GATT_SRV_CHGD,
                           LEGATTDB_CHAR_PROP_NOTIFY,
                           LEGATTDB_PERM_NONE),

    // Generic Access service
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ACCESS, UUID_SERVCLASS_GAP_SERVER),

    CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_DEVICE_NAME,
                               HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
                               GATT_UUID_GAP_DEVICE_NAME,
                               LEGATTDB_CHAR_PROP_READ,
                               LEGATTDB_PERM_READABLE),

    CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_APPEARANCE,
                           HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,
                           GATT_UUID_GAP_ICON,
                           LEGATTDB_CHAR_PROP_READ,
                           LEGATTDB_PERM_READABLE),

    // Link Loss service
    PRIMARY_SERVICE_UUID16 (HDLS_LINK_LOSS, UUID_SERVCLASS_LINKLOSS),

    CHARACTERISTIC_UUID16_WRITABLE (HDLC_LINK_LOSS_ALERT_LEVEL,
                                    HDLC_LINK_LOSS_ALERT_LEVEL_VALUE,
                                    GATT_UUID_ALERT_LEVEL,
                                    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    // Immediate alert service
    PRIMARY_SERVICE_UUID16 (HDLS_IMMEDIATE_ALERT, UUID_SERVCLASS_IMMEDIATE_ALERT),

    CHARACTERISTIC_UUID16_WRITABLE (HDLC_IMMEDIATE_ALERT_LEVEL,
                                    HDLC_IMMEDIATE_ALERT_LEVEL_VALUE,
                                    GATT_UUID_ALERT_LEVEL,
                                    LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                    LEGATTDB_PERM_WRITE_CMD),

    // TX Power service
    PRIMARY_SERVICE_UUID16 (HDLS_TX_POWER, UUID_SERVCLASS_TX_POWER),

    CHARACTERISTIC_UUID16 (HDLC_TX_POWER_LEVEL,
                           HDLC_TX_POWER_LEVEL_VALUE,
                           GATT_UUID_TX_POWER_LEVEL,
                           LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
};

const uint16_t gatt_db_size = sizeof(gatt_db);
