/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file wiced_hci_bt_gatt.c
 *
 * This handles the BLE api functions when USE_WICED_HCI is defined.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_gatt.h"
#include "wiced_hci_bt_internal_common.h"
#include "wwd_debug.h"
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_gatt_cb(uint16_t command, uint8_t* payload, uint32_t len);

/******************************************************
 *               External Function Declarations
 ******************************************************/
/******************************************************
 *               Variable Definitions
 ******************************************************/
wiced_hci_bt_gatt_context_t     wh_bt_gatt_context;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_bt_gatt_status_t wiced_bt_gatt_send_notification (uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t* data         = NULL;
    uint8_t  length       = 0;

    /* sizes of the conn_id, attr_handle, val_len */
    length += sizeof(uint16_t)*2 + (val_len*sizeof(uint8_t));

    data = (uint8_t*)calloc(length, sizeof(uint8_t));

    memcpy(&data[0], &conn_id, sizeof(uint16_t));
    memcpy(&data[2], &attr_handle, sizeof(uint16_t));
    memcpy(&data[4], p_val, val_len*sizeof(uint8_t));

    wiced_hci_send(HCI_CONTROL_GATT_COMMAND_NOTIFY,data, length);
    free(data);

    return result;
}

wiced_bt_gatt_status_t wiced_bt_gatt_send_indication (uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val )
{
    wiced_result_t result  = WICED_SUCCESS;

    uint8_t* data         = NULL;
    uint8_t  length       = 0;

    /* sizes of the conn_id, attr_handle, val_len */
    length += sizeof(uint16_t)*2 + (val_len*sizeof(uint8_t));

    data = (uint8_t*)calloc(length, sizeof(uint8_t));

    memcpy(&data[0], &conn_id, sizeof(uint16_t));
    memcpy(&data[2], &attr_handle, sizeof(uint16_t));
    memcpy(&data[4], p_val, val_len*sizeof(uint8_t));

    wiced_hci_send(HCI_CONTROL_GATT_COMMAND_INDICATE,data, length);
    free(data);

    return result;
}

wiced_bt_gatt_status_t wiced_bt_gatt_db_init (const uint8_t *p_gatt_db, uint16_t size)
{
    //wiced_hci_send(HCI_CONTROL_GATT_COMMAND_DB_INIT,(uint8_t*)p_gatt_db, size);
    return WICED_SUCCESS;
}

wiced_bt_gatt_status_t wiced_bt_gatt_register (wiced_bt_gatt_cback_t *p_gatt_cback)
{
    wh_bt_gatt_context.gatt_mgmt_cb = p_gatt_cback;
    wh_bt_gatt_context.gatt_context_cb = wiced_hci_gatt_cb;
    wiced_hci_set_event_callback(GATT, wh_bt_gatt_context.gatt_context_cb);
    //wiced_hci_send(HCI_CONTROL_GATT_COMMAND_REGISTER, NULL, 0 );
    return WICED_SUCCESS;
}


static void wiced_hci_gatt_cb(uint16_t command, uint8_t* payload, uint32_t len)
{
    wiced_bt_gatt_event_data_t event_data;

    WICED_HCI_DEBUG_LOG(("[%s] \n",__func__));

    switch(command)
    {
        case HCI_CONTROL_GATT_EVENT_COMMAND_STATUS:
#ifdef ENABLE_BT_PROTOCOL_TRACES
            uint8_t gatt_command_status = 0;
            STREAM_TO_UINT8(gatt_command_status, payload);
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_LE_EVENT_COMMAND_STATUS: %d\n", gatt_command_status));
#endif
            break;

        case HCI_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE:
            break;

        case HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED\n"));
            break;

        case HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED:
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED\n",__func__));
            break;

        case HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED:
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED\n",__func__));
            break;

        case HCI_CONTROL_GATT_EVENT_READ_REQUEST:
        {
            uint8_t       data[4] = {0}; /* data provided in this event is only 4 bytes */
            uint8_t*      read_response_data = NULL;
            uint8_t*      ptr = NULL;
            uint8_t       val[50] = {'\0'};
            uint16_t length = 0;
            uint16_t val_length =23;

            STREAM_TO_ARRAY(data, payload, len);

            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_READ_REQUEST\n",__func__));

            event_data.attribute_request.conn_id = (uint16_t)((data[0])|data[1] <<8);
            event_data.attribute_request.request_type = GATTS_REQ_TYPE_READ;
            event_data.attribute_request.data.read_req.p_val = val;
            event_data.attribute_request.data.read_req.offset = 0;
            event_data.attribute_request.data.read_req.p_val_len = &val_length;
            event_data.attribute_request.data.read_req.handle = (uint16_t)((data[2])|data[3]<<8);
            WICED_HCI_DEBUG_LOG(("value of read_handle: %x\n",event_data.attribute_request.data.read_req.handle));
            wh_bt_gatt_context.gatt_mgmt_cb(GATT_ATTRIBUTE_REQUEST_EVT,&event_data);

            WICED_HCI_DEBUG_LOG(("[%s] conn_id = %x  handle = %x\n",__func__,
                    event_data.attribute_request.conn_id,
                    event_data.attribute_request.data.handle));

            length += ( sizeof(event_data.attribute_request.conn_id) + sizeof(event_data.attribute_request.data.handle) + *(event_data.attribute_request.data.read_req.p_val_len));

            read_response_data = (uint8_t*)calloc(length, sizeof(uint8_t));
            ptr = read_response_data;
            UINT16_TO_STREAM(ptr,event_data.attribute_request.conn_id);
            UINT16_TO_STREAM(ptr,event_data.attribute_request.data.handle);
            ARRAY_TO_STREAM(ptr, event_data.attribute_request.data.read_req.p_val,*(event_data.attribute_request.data.read_req.p_val_len));
            wiced_hci_send(HCI_CONTROL_GATT_COMMAND_READ_RESPONSE,read_response_data,length);

            free(read_response_data);

            break;
        }

        case HCI_CONTROL_GATT_EVENT_READ_RESPONSE:
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_READ_RESPONSE\n",__func__));
            break;

        case HCI_CONTROL_GATT_EVENT_WRITE_REQUEST:
        {
            uint8_t data[4] = {0}; /* data provided in this event is only 4 bytes */
            uint8_t*      write_response_data = NULL;
            uint8_t*      ptr = NULL;
            uint8_t       status_code = 0;
            uint16_t length = 0;

            STREAM_TO_ARRAY(data, payload, len);

            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_WRITE_REQUEST\n",__func__));

            event_data.attribute_request.conn_id = (uint16_t)((data[0])|data[1] <<8);
            event_data.attribute_request.request_type = GATTS_REQ_TYPE_WRITE;
            event_data.attribute_request.data.write_req.handle = (uint16_t)((data[2])|data[3]<<8);
            event_data.attribute_request.data.write_req.p_val = &data[4];
            event_data.attribute_request.data.write_req.val_len = len-4;
            //          TODO: Need to figure out offset usage
            event_data.attribute_request.data.write_req.offset = 0;
            event_data.attribute_request.data.write_req.is_prep = WICED_TRUE;

            WICED_HCI_DEBUG_LOG(("[%s] conn_id = %x  handle = %x\n",__func__,
                    event_data.attribute_request.conn_id,
                    event_data.attribute_request.data.handle));

            wh_bt_gatt_context.gatt_mgmt_cb(GATT_ATTRIBUTE_REQUEST_EVT,&event_data);

            /* Populate data and send it to the 20706 side. */

            length += ( sizeof(event_data.attribute_request.conn_id)+sizeof(event_data.attribute_request.data.write_req.handle) + 1);

            write_response_data = (uint8_t*)calloc(length, sizeof(uint8_t));
            ptr = write_response_data;

            UINT16_TO_STREAM(ptr,event_data.attribute_request.conn_id);
            UINT16_TO_STREAM(ptr,event_data.attribute_request.data.write_req.handle);
            UINT8_TO_STREAM(ptr,status_code);

            wiced_hci_send(HCI_CONTROL_GATT_COMMAND_WRITE_RESPONSE, write_response_data, length);
            free(write_response_data);

            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_COMMAND_WRITE_RESPONSE\n",__func__));
            break;
        }

        case HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE:
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE\n",__func__));
            break;

        case HCI_CONTROL_GATT_EVENT_INDICATION:
        {
            uint8_t data[4] = {0}; /* data provided in this event is only 4 bytes */
            uint8_t*      indicate_confirm_data = NULL;
            uint8_t*      ptr = NULL;
            uint16_t length = 0;

            STREAM_TO_ARRAY(data, payload, len);
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_INDICATION\n",__func__));
            event_data.attribute_request.conn_id = (uint16_t)((data[0])|data[1] <<8);
            event_data.attribute_request.data.handle = (uint16_t)((data[2])|data[3]<<8);

            event_data.attribute_request.request_type = GATTS_REQ_TYPE_CONF;
            wh_bt_gatt_context.gatt_mgmt_cb(GATT_ATTRIBUTE_REQUEST_EVT,&event_data);

            /* Populate data to send data for indication confirm */

            length += ( sizeof(event_data.attribute_request.conn_id)+sizeof(event_data.attribute_request.data.write_req.handle));

            indicate_confirm_data = (uint8_t*)calloc(length, sizeof(uint8_t));
            ptr = indicate_confirm_data;

            UINT16_TO_STREAM(ptr,event_data.attribute_request.conn_id);
            UINT16_TO_STREAM(ptr,event_data.attribute_request.data.write_req.handle);
            wiced_hci_send(HCI_CONTROL_GATT_COMMAND_INDICATE_CONFIRM,indicate_confirm_data,length);
            free(indicate_confirm_data);
        }
        break;

        case HCI_CONTROL_GATT_EVENT_NOTIFICATION:
        {
            uint8_t data[4] = {0}; /* data provided in this event is only 4 bytes */
            STREAM_TO_ARRAY(data, payload, len);
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_GATT_EVENT_NOTIFICATION\n",__func__));
            event_data.attribute_request.conn_id = (uint16_t)((data[0])|data[1] <<8);
            event_data.attribute_request.data.handle = (uint16_t)((data[2])|data[3]<<8);
        }
        break;

        default:
            WICED_HCI_DEBUG_LOG(("[%s] Command Not Handled: %x\n",__func__,command));
            break;
    }
}
