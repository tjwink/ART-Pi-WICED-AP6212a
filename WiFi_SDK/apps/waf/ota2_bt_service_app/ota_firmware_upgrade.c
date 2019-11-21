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
 * Bluetooth Over the Air (OTA) Firmware Upgrade Application
 *
 * This is the Factory Reset (FR) application for the Iwa platform.
 * This application is loaded by the bootloader when recovery button
 * is pressed for more than 10 seconds.
 *
 * The application responsibility is to publish OTA FW upgrade service
 * in the GATT database and to pass stack callbacks to the library.
 * See libraries/ota2_bt_service/ota_fw_upgrade.c file for the
 * description of the OTA protocol.
 * This version of the OTA firmware upgrade relies on the Bluetooth
 * standard security.  It is expected that full implementation
 * will use application level verification that downloaded firmware is
 * for this Product ID and that the image has not been tempered with.
 *
 * Features demonstrated
 *  - BT Firmware, Filesystem and App Upgrade
 *
 * To demonstrate the app, work through the following steps.
 * 1. Press and hold SW11 button and reboot the Iwa WICED board.
 * 2. Keep holding the SW11 button for approximately 10~11 sec.
 * 3. Search the BT device name "BTOTA" from client and Pair from client.
 * 4. Use WsOtaUpgrade application to try over the air upgrade.
 */

#include <string.h>
#include "wiced.h"
#include "wiced_bt_stack.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_bt_firmware_upgrade.h"
#include "wiced_bt_fw_upgrade.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HANDLE_OTA_FW_UPGRADE_GATT_SERVICE              1
#define HANDLE_OTA_FW_UPGRADE_GAP_SERVICE               2
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME             3
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL         4
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE       5
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL   6

#define OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID                 WICED_NVRAM_VSID_START
#define OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID           (WICED_NVRAM_VSID_START + 1)

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/*
 * This is the GATT database for the OTA upgrade application.  The database
 * defines mandatory GATT and GAP service and OTA FW Upgrade service itself.
 * To merge OTA functionality into a different application, copy service
 * UUID_OTA_FW_UPGRADE_SERVICE and relative characteristic to the database of
 * the target application.
 * Note that the handles do not need to be sequential, but need to be sorted in
 * ascending order.  The first handle of the service is
 * HANDLE_OTA_FW_UPGRADE_SERVICE = 0xff00. The service needs to be placed at the
 * end of the device's GATT database.
 */
const uint8_t ota_fw_upgrade_gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16(HANDLE_OTA_FW_UPGRADE_GATT_SERVICE, UUID_SERVICE_GATT),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16(HANDLE_OTA_FW_UPGRADE_GAP_SERVICE, UUID_SERVICE_GAP),

        // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16(HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME, HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE /*| LEGATTDB_PERM_AUTH_READABLE*/),

        // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16(HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE, HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE /*| LEGATTDB_PERM_AUTH_READABLE*/),

    // Handle 0xff00: Broadcom vendor specific WICED Upgrade Service.
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),

        // Handles 0xff03: characteristic WS Control Point, handle 0xff04 characteristic value.
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ /*| LEGATTDB_PERM_AUTH_WRITABLE*/),

            // Declare client characteristic configuration descriptor
            // Value of the descriptor can be modified by the client
            // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ /*| LEGATTDB_PERM_AUTH_WRITABLE */),

        // Handle 0xff07: characteristic WS Data, handle 0xff08 characteristic value. This
        // characteristic is used to send next portion of the FW Similar to the control point
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ /*| LEGATTDB_PERM_AUTH_WRITABLE */),
};

static void                     app_init(void);
static wiced_result_t           app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                     app_advertisement_stopped(void);
static wiced_bt_gatt_status_t   app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
//static void                     app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
static void                     app_set_advertisement_data(void);
static wiced_bool_t             app_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             app_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
//static void                     app_timeout(uint32_t count);
static wiced_bt_gatt_status_t   app_connection_status_event(wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t   app_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t   app_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t   app_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);
static wiced_bt_gatt_status_t   app_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

wiced_timer_t app_timer;
uint16_t      app_conn_id = 0;

extern const wiced_bt_cfg_settings_t app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t app_buf_pools[];

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start ()
{
    WICED_BT_TRACE("BT OTA Start\n");

    // Register call back and configuration with stack
    wiced_bt_stack_init(app_management_callback, &app_cfg_settings, app_buf_pools);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void app_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    /* Initialize wiced app */
//    wiced_bt_app_init();

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init(ota_fw_upgrade_gatt_database, sizeof(ota_fw_upgrade_gatt_database));

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef WICED_BT_TRACE_ENABLE
    /* Register callback for receiving hci traces */
    //wiced_bt_dev_register_hci_trace(app_hci_trace_callback);

    /* Starting the app timers , seconds timer and the ms timer  */
//    wiced_bt_app_start_timer(1, 0, app_timeout, NULL);
#endif

    /* OTA Firmware upgrade Initialization */
    if (wiced_ota_fw_upgrade_init(NULL) == WICED_FALSE)
    {
        WICED_BT_TRACE("OTA upgrade Init failure !!! \n");
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    app_set_advertisement_data();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}

#if 0
/*
 *  Pass protocol traces up through the UART
 */
void app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data );
}
#endif

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void app_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = (uint16_t)strlen((const char *)app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * This function is invoked when advertisements stop.  Continue advertising if there
 * are no active connections
 */
void app_advertisement_stopped(void)
{
    wiced_result_t result;

    // while we are not connected
    if (app_conn_id == 0)
    {
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
        WICED_BT_TRACE("ADV stop\n");
    }
}

#if 0
/*
 * The function invoked on timeout of app seconds timer.
 */
void app_timeout(uint32_t count)
{
    static uint32_t timer_count = 0;
    WICED_BT_TRACE("app_timeout:%d\n", timer_count++);
}
#endif

/*
 * Application management callback.  Stack passes various events to the function that may
 * be of interest to the application.
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    uint8_t                          *p_keys;
    wiced_bt_ble_advert_mode_t       *p_mode;

    WICED_BT_TRACE("app_management_callback: %x\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        app_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        app_save_link_keys(&p_event_data->paired_device_link_keys_update);
        wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (app_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

     case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
         /* save keys to NVRAM */
         p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
         wiced_hal_write_nvram ( OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
         WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
         break;


     case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
         /* read keys from NVRAM */
         p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
         wiced_hal_read_nvram( OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
         WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
         break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if (*p_mode == BTM_BLE_ADVERT_OFF)
        {
            app_advertisement_stopped();
        }
        break;

    default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    int to_copy;

    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
    switch(p_read_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL:
        if (p_read_data->offset >= strlen((const char *)app_cfg_settings.device_name))
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = strlen((const char *)app_cfg_settings.device_name) - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, app_cfg_settings.device_name + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    case HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = 2 - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, &app_cfg_settings.gatt_cfg.appearance + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (p_write_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_write_data);
    }

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process indication_confirm from peer device
 */
wiced_bt_gatt_status_t app_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t app_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        app_conn_id = p_status->conn_id;
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
    else
    {
        app_conn_id = 0;
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    }
    // Pass connection up/down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t app_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = app_gatts_req_read_handler(p_data->conn_id, &p_data->data.read_req);
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = app_gatts_req_write_handler(p_data->conn_id, &p_data->data.write_req);
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_MTU:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_CONF:
        result = app_indication_cfm_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are ommitted.
 */
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = app_connection_status_event(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = app_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }
    return result;
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t app_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d\n", bytes_written, OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t app_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d\n", bytes_read, OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

