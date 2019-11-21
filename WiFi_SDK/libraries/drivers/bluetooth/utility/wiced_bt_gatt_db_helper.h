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

/* @file
 * This file contains definitions for WICED API functions for dynamic GATT database creation
 */

// wiced_bt_gatt_db.h

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __GATT_DATABASE_H__
#define __GATT_DATABASE_H__

#include <stdint.h>
#include <string.h>
#include "wiced_bt_dev.h"
#include "bt_target.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced.h"
#include "wiced_bt_stack.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"

// External definitions
extern uint8_t  *gatt_server_db;
extern uint16_t gatt_server_db_len;

/**
 * Function     wiced_bt_gatt_db_primary_service_add
 *
 * Add a GATT primary service to the GATT database
 *
 * @param[in]   handle : the handle for the GATT primary service
 * @param[in]   uuid   : the GATT service UUID
 * @return      Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_primary_service_add(uint16_t handle, wiced_bt_uuid_t* uuid);


/**
 * Function     wiced_bt_gatt_db_secondary_service_add
 *
 * Add a GATT secondary service to the GATT database
 *
 * @param[in]   handle : the handle for the GATT secondary service
 * @param[in]   uuid : the included GATT service UUID
 * @return      Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_secondary_service_add(uint16_t handle, wiced_bt_uuid_t* uuid);


/**
 * Function     wiced_bt_gatt_db_included_service_add
 *
 * Add a GATT included service to the GATT database
 *
 * @param[in]   handle           : the handle for the GATT included service
 * @param[in]   service_handle   : the service handle for the GATT included service
 * @param[in]   end_group_handle : the end group handle for the GATT included service
 * @param[in]   uuid             : the included GATT service UUID
 * @return      Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_included_service_add(uint16_t handle, uint16_t service_handle, uint16_t end_group_handle, wiced_bt_uuid_t* uuid);


/**
 * Function     wiced_bt_gatt_db_characteristic_add
 *
 * Add a GATT characteristic to the GATT database
 *
 * @param[in]   handle       : the handle for the GATT characteristic
 * @param[in]   handle_value : the handle for the GATT characteristic
 * @param[in]   uuid         : the GATT characteristic UUID
 * @param[in]   property     : the GATT characteristic property
 * @param[in]   permission   : the GATT characteristic permission
 * @return      Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_characteristic_add(uint16_t handle, uint16_t handle_value, wiced_bt_uuid_t* uuid, uint8_t property, uint8_t permission);


/**
 * Function     wiced_bt_gatt_db_descriptor_add
 *
 * Add a GATT descriptor to the GATT database
 *
 * @param[in]   handle     : the handle for the GATT descriptor
 * @param[in]   uuid       : the GATT characteristic UUID
 * @param[in]   permission : the GATT characteristic permission
 * @return      Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_descriptor_add(uint16_t handle, wiced_bt_uuid_t* uuid, uint8_t permission);

#endif /* __GATT_DATABASE_H__ */

#ifdef __cplusplus
} /*extern "C" */
#endif
