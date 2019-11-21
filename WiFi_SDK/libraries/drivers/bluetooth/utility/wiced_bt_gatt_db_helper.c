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

#include "wiced.h"
#include "wiced_bt_gatt.h"
#include "gki.h"
#include "wiced_bt_gatt_db_helper.h"


/******************************************************************************
 * Constants
 ******************************************************************************/
#define MAX_GATT_DB_PKT_LEN  64


/*************************************************************************************
 * GATT server definitions
 *************************************************************************************/

// GATT database pointer and length
uint8_t* gatt_server_db         = NULL;
uint16_t gatt_server_db_len     = 0;
uint16_t gatt_server_db_buf_len = 0;


/*************************************************************************************
** GATT function implementation
*************************************************************************************/

/* Local function: is the characteristic or descriptor permission writable? */
static wiced_bool_t is_permission_writable(uint8_t permission)
{
    if ((permission & (LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ |
                       LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_AUTH_WRITABLE)) == 0)
    {
        return WICED_FALSE;
    }
    return WICED_TRUE;
}


/* Local static function: Add a GATT service/characteristic/descriptor database packet into
 * the device GATT database.
 * When the application starts, 1 KB buffer is preallocated for the database. If this buffer
 * is not enough, the next size buffer will be used to replace the current buffer. If no
 * larger buffer, return WICED_FALSE.
 */
static wiced_bool_t gatt_db_add_buf(uint8_t* pkt, uint16_t pkt_len)
{
    wiced_bt_gatt_status_t gatt_status;
    uint8_t*  buf;
    uint16_t  db_len = gatt_server_db_len + pkt_len;

    if (gatt_server_db == NULL)
    {
        /* GATT database buffer not allocated yet, allocate 1 KB buffer */
        gatt_server_db = (uint8_t*)GKI_getbuf(1000);
        if (gatt_server_db == NULL)
        {
            return WICED_FALSE;
        }
        gatt_server_db_buf_len = GKI_get_buf_size(gatt_server_db);
    }

    if (gatt_server_db_buf_len < db_len)
    {
        /* Allocate a buffer large enough for the current database */
        buf = (uint8_t*)GKI_getbuf(db_len);
        if (buf == NULL) /* No buffer large enough */
        {
            return WICED_FALSE;
        }
        memcpy(buf, gatt_server_db, gatt_server_db_len);
        GKI_freebuf(gatt_server_db);
        gatt_server_db = buf;
        gatt_server_db_buf_len = GKI_get_buf_size(gatt_server_db);
    }

    /* Add the new GATT service/characteristic/descriptor database packet into
     * the device GATT database
     */
    memcpy(&gatt_server_db[gatt_server_db_len], pkt, pkt_len);
    gatt_server_db_len = db_len;
    WPRINT_APP_INFO(( "%s: db_len = %d\n", __func__, gatt_server_db_len ));

    /* Initialize the GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, gatt_server_db_len);

    WPRINT_APP_INFO(("%s: status = %d, db_len = %d\n", __func__, gatt_status, gatt_server_db_len));
    return WICED_TRUE;
}


/*
 * Function     wiced_bt_gatt_db_primary_service_add
 *
 * Add a GATT primary service to the GATT database
 *
 * param:   handle : the handle for the GATT primary service
 * param:   uuid   : the GATT service UUID
 * return:  Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_primary_service_add(uint16_t handle, wiced_bt_uuid_t* uuid)
{
    uint16_t pkt_len;
    uint8_t  pkt[MAX_GATT_DB_PKT_LEN];
    uint8_t* p = pkt;

    /* Build the service database packet */
    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, LEGATTDB_PERM_READABLE);
    if (uuid->len == LEN_UUID_16)
    {
        UINT8_TO_STREAM(p, 4);
        UINT16_TO_STREAM(p, GATT_UUID_PRI_SERVICE);
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        UINT8_TO_STREAM(p, 18);
        UINT16_TO_STREAM(p, GATT_UUID_PRI_SERVICE);
        ARRAY_TO_STREAM(p, uuid->uu.uuid128, LEN_UUID_128);
    }
    else
    {
        return WICED_FALSE;
    }

    /* Calculate the service database packet length */
    pkt_len = (uint32_t)p - (uint32_t)pkt;

    /* Add the service database packet length */
    return gatt_db_add_buf(pkt, pkt_len);
}


/*
 * Function     wiced_bt_gatt_db_secondary_service_add
 *
 * Add a GATT secondary service to the GATT database
 *
 * param:   handle : the handle for the GATT secondary service
 * param:   uuid : the included GATT service UUID
 * return:  Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_secondary_service_add(uint16_t handle, wiced_bt_uuid_t* uuid)
{
    uint16_t pkt_len;
    uint8_t  pkt[MAX_GATT_DB_PKT_LEN];
    uint8_t* p = pkt;

    /* Build the service database packet */
    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, LEGATTDB_PERM_READABLE);
    if (uuid->len == LEN_UUID_16)
    {
        UINT8_TO_STREAM(p, 4);
        UINT16_TO_STREAM(p, GATT_UUID_SEC_SERVICE);
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        UINT8_TO_STREAM(p, 18);
        UINT16_TO_STREAM(p, GATT_UUID_SEC_SERVICE);
        ARRAY_TO_STREAM(p, uuid->uu.uuid128, LEN_UUID_128);
    }
    else
    {
        return WICED_FALSE;
    }

    /* Calculate the service database packet length */
    pkt_len = (uint32_t)p - (uint32_t)pkt;

    /* Add the service database packet length */
    return gatt_db_add_buf(pkt, pkt_len);
}


/*
 * Function     wiced_bt_gatt_db_included_service_add
 *
 * Add a GATT included service to the GATT database
 *
 * param:   handle           : the handle for the GATT included service
 * param:   service_handle   : the service handle for the GATT included service
 * param:   end_group_handle : the end group handle for the GATT included service
 * param:   uuid             : the included GATT service UUID
 * return:  Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_included_service_add(uint16_t handle, uint16_t service_handle, uint16_t end_group_handle, wiced_bt_uuid_t* uuid)
{
    uint16_t pkt_len;
    uint8_t  pkt[MAX_GATT_DB_PKT_LEN];
    uint8_t* p = pkt;

    /* Build the service database packet */
    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, LEGATTDB_PERM_READABLE);

    if (uuid->len == LEN_UUID_16)
    {
        UINT8_TO_STREAM(p, 8);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        UINT8_TO_STREAM(p, 6);
    }
    else
    {
        return WICED_FALSE;
    }

    UINT16_TO_STREAM(p, GATT_UUID_INCLUDE_SERVICE);
    UINT16_TO_STREAM(p, service_handle);
    UINT16_TO_STREAM(p, end_group_handle);

    if (uuid->len == LEN_UUID_16)
    {
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
    }

    /* Calculate the service database packet length */
    pkt_len = (uint32_t)p - (uint32_t)pkt;

    /* Add the service database packet length */
    return gatt_db_add_buf(pkt, pkt_len);
}


/*
 * Function     wiced_bt_gatt_db_characteristic_add
 *
 * Add a GATT characteristic to the GATT database
 *
 * param:   handle       : the handle for the GATT characteristic
 * param:   handle_value : the handle for the GATT characteristic
 * param:   uuid         : the GATT characteristic UUID
 * param:   property     : the GATT characteristic property
 * param:   permission   : the GATT characteristic permission
 * return:  Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_characteristic_add(uint16_t handle, uint16_t handle_value, wiced_bt_uuid_t* uuid, uint8_t property, uint8_t permission)
{
    uint16_t     pkt_len;
    uint8_t      pkt[MAX_GATT_DB_PKT_LEN];
    uint8_t*     p = pkt;
    wiced_bool_t permission_writable = is_permission_writable(permission);

    /* Build the characteristic database packet */
    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, LEGATTDB_PERM_READABLE);

    if (uuid->len == LEN_UUID_16)
    {
        UINT8_TO_STREAM(p, 7);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        UINT8_TO_STREAM(p, 21);
    }
    else
    {
        return WICED_FALSE;
    }

    UINT16_TO_STREAM(p, GATT_UUID_CHAR_DECLARE);
    UINT8_TO_STREAM(p, property);
    UINT16_TO_STREAM(p, handle_value);

    if (uuid->len == LEN_UUID_16)
    {
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
        UINT16_TO_STREAM(p, handle_value);
        UINT8_TO_STREAM(p, permission);
        UINT8_TO_STREAM(p, LEGATTDB_UUID16_SIZE);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        ARRAY_TO_STREAM(p, uuid->uu.uuid128, LEN_UUID_128);
        UINT16_TO_STREAM(p, handle_value);
        UINT8_TO_STREAM(p, permission | LEGATTDB_PERM_SERVICE_UUID_128);
        UINT8_TO_STREAM(p, LEGATTDB_UUID128_SIZE);
    }

    if (permission_writable)
    {
        UINT8_TO_STREAM(p, 0);
    }

    if (uuid->len == LEN_UUID_16)
    {
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        ARRAY_TO_STREAM(p, uuid->uu.uuid128, LEN_UUID_128);
    }

    /* Calculate the characteristic database packet length */
    pkt_len = (uint32_t)p - (uint32_t)pkt;

    /* Add the characteristic database packet length */
    return gatt_db_add_buf(pkt, pkt_len);
}


/*
 * Function     wiced_bt_gatt_db_descriptor_add
 *
 * Add a GATT descriptor to the GATT database
 *
 * param:   handle     : the handle for the GATT descriptor
 * param:   uuid       : the GATT characteristic UUID
 * param:   permission : the GATT characteristic permission
 * return:  Return WICED_TRUE if service is added, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_gatt_db_descriptor_add(uint16_t handle, wiced_bt_uuid_t* uuid, uint8_t permission)
{
    uint16_t     pkt_len;
    uint8_t      pkt[MAX_GATT_DB_PKT_LEN];
    uint8_t*     p = pkt;
    wiced_bool_t permission_writable = is_permission_writable(permission);

    /* Build the descriptor database packet */
    UINT16_TO_STREAM(p, handle);

    if (uuid->len == LEN_UUID_16)
    {
        UINT8_TO_STREAM(p, permission);
        UINT8_TO_STREAM(p, LEGATTDB_UUID16_SIZE);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        UINT8_TO_STREAM(p, permission | LEGATTDB_PERM_SERVICE_UUID_128);
        UINT8_TO_STREAM(p, LEGATTDB_UUID128_SIZE);
    }
    else
    {
        return WICED_FALSE;
    }

    if (permission_writable)
    {
        UINT8_TO_STREAM(p, 0);
    }

    if (uuid->len == LEN_UUID_16)
    {
        UINT16_TO_STREAM(p, uuid->uu.uuid16);
    }
    else if (uuid->len == LEN_UUID_128)
    {
        ARRAY_TO_STREAM(p, uuid->uu.uuid128, LEN_UUID_128);
    }

    /* Calculate the descriptor database packet length */
    pkt_len = (uint32_t)p - (uint32_t)pkt;

    /* Add the descriptor database packet length */
    return gatt_db_add_buf(pkt, pkt_len);
}
