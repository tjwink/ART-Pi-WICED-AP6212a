/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file wiced_hci_bt_dm.c
 *
 * This handles the Device management functions when USE_WICED_HCI is defined.
 *
 */
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
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
static void wiced_hci_dm_cb(uint16_t command, uint8_t* payload, uint32_t len);

/******************************************************
 *               External Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
wiced_hci_bt_dm_context_t     wh_bt_dm_context;
wiced_mutex_t                 global_trace_mutex; // TODO: is this avoidable

/******************************************************
 *               External Variable Declaration
 ******************************************************/
extern wiced_hci_bt_gatt_context_t     wh_bt_gatt_context;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_bt_stack_init( wiced_bt_management_cback_t *p_bt_management_cback,
        const wiced_bt_cfg_settings_t     *p_bt_cfg_settings,
        const wiced_bt_cfg_buf_pool_t     wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] )
{
    wiced_result_t result = WICED_SUCCESS;

    wh_bt_dm_context.dm_mgmt_cb = p_bt_management_cback;

    /* Setting the event callbacks as the first step to ensure that any data from the BT chip is routed to the right callback */
    wiced_hci_set_event_callback(DEVICE, wiced_hci_dm_cb);
    wiced_hci_set_event_callback(LE, wiced_hci_dm_cb);

    WICED_HCI_DEBUG_LOG(("[%s] init bus\n",__FUNCTION__));
    /* start wiced_hci only here. Once started, it stays up. */
    if ( (result = wiced_hci_up() ) != WICED_SUCCESS )
    {
        WICED_HCI_DEBUG_LOG( ( "[%s] Failed to initialize wiced hci\n",__FUNCTION__) );
        return result;
    }
    WICED_HCI_DEBUG_LOG(("[%s] wiced_hci up\n",__FUNCTION__));

    return result;
}

wiced_result_t wiced_bt_stack_deinit(void)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));

    /* Call the wiced_hci_down */
    if ( (result = wiced_hci_down() ) != WICED_SUCCESS )
    {
        WICED_HCI_DEBUG_LOG( ( "[%s] Failed to initialize wiced hci\n",__FUNCTION__) );
        return result;
    }

    /* Send BTM_DISABLED_EVENT */
    wh_bt_dm_context.dm_mgmt_cb( BTM_DISABLED_EVT, NULL );

    memset(&wh_bt_dm_context, 0 ,sizeof(wh_bt_dm_context));
    memset(&wh_bt_gatt_context, 0 ,sizeof(wh_bt_gatt_context));

    return result;
}

static void wiced_hci_dm_cb(uint16_t command, uint8_t* payload, uint32_t len)
{
    wiced_bt_management_evt_t evt = 0;
    wiced_bt_management_evt_data_t data;
    uint8_t* p = payload;
    WICED_HCI_DEBUG_LOG((" [%s] [%x] [%d]\n\n", __func__, command, (int)len));

    switch(command)
    {
        case HCI_CONTROL_EVENT_USER_CONFIRMATION:
            STREAM_TO_BDADDR(data.user_confirmation_request.bd_addr, payload);
            STREAM_TO_UINT32(data.user_confirmation_request.numeric_value, payload);
            WICED_HCI_DEBUG_LOG(("bd_addr received from 20706: %x %x %x %x %x %x\n",data.user_confirmation_request.bd_addr[0], data.user_confirmation_request.bd_addr[1],
                    data.user_confirmation_request.bd_addr[2],data.user_confirmation_request.bd_addr[3],
                    data.user_confirmation_request.bd_addr[4], data.user_confirmation_request.bd_addr[5] ));
            evt = BTM_USER_CONFIRMATION_REQUEST_EVT;
            break;

        case HCI_CONTROL_EVENT_READ_LOCAL_BDA:
            STREAM_TO_BDADDR( wh_bt_dm_context.bd_addr, payload );
            evt = 0xff;
            break;

        case HCI_CONTROL_EVENT_NVRAM_DATA:
            {
                WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_EVENT_NVRAM_DATA\n",__func__));

                STREAM_TO_UINT16(wh_bt_dm_context.nvram_id, p);
                WICED_HCI_DEBUG_LOG(("NVRAM write:id:%x\n", wh_bt_dm_context.nvram_id));

                memcpy((void*)&data.paired_device_link_keys_update.bd_addr, p, sizeof(wiced_bt_device_address_t));
                p += sizeof(wiced_bt_device_address_t);
                WICED_HCI_DEBUG_LOG(("Bluetooth address of the paired device: %x %x %x %x %x %x\n",data.paired_device_link_keys_update.bd_addr[0], data.paired_device_link_keys_update.bd_addr[1],
                        data.paired_device_link_keys_update.bd_addr[2],data.paired_device_link_keys_update.bd_addr[3],
                        data.paired_device_link_keys_update.bd_addr[4], data.paired_device_link_keys_update.bd_addr[5] ));

                memcpy((void*)&data.paired_device_link_keys_update.key_data.br_edr_key_type, p, sizeof(uint8_t));
                p += sizeof(uint8_t);

                memcpy((void*)&data.paired_device_link_keys_update.key_data.br_edr_key, p, LINK_KEY_LEN);
                p += LINK_KEY_LEN;

                memcpy((void*)&data.paired_device_link_keys_update.key_data.le_keys_available_mask, p, sizeof(wiced_bt_dev_le_key_type_t));
                p += sizeof(wiced_bt_dev_le_key_type_t);

                memcpy((void*)&data.paired_device_link_keys_update.key_data.ble_addr_type,p,  sizeof(wiced_bt_ble_address_type_t));
                p += sizeof(wiced_bt_ble_address_type_t);

                memcpy((void*)&data.paired_device_link_keys_update.key_data.static_addr_type,p,sizeof(wiced_bt_ble_address_type_t));
                p += sizeof(wiced_bt_ble_address_type_t);

                memcpy(data.paired_device_link_keys_update.key_data.static_addr,p,  sizeof(wiced_bt_device_address_t));
                p += sizeof(wiced_bt_device_address_t);

                memcpy((void*)&data.paired_device_link_keys_update.key_data.le_keys,p, sizeof(wiced_bt_ble_keys_t));
                p += sizeof(wiced_bt_ble_keys_t);

                evt = BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT;
            }
            break;

        case HCI_CONTROL_EVENT_UPDATE_LINK_KEY:
            memcpy((void*)&data.paired_device_link_keys_update, payload, sizeof(data.paired_device_link_keys_update));
            evt = BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT;
            break;

        case HCI_CONTROL_EVENT_REQUEST_ID_KEYS:
            evt = BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT;
            break;

        case HCI_CONTROL_EVENT_READ_RSSI:
            break;

        case HCI_CONTROL_EVENT_DEVICE_STARTED:
            WICED_HCI_DEBUG_LOG(("[%s] HCI_CONTROL_EVENT_DEVICE_STARTED\n",__func__));
            if ( payload )
            {
                STREAM_TO_UINT8(data.enabled.status, payload);
                evt = BTM_ENABLED_EVT;
            }
            break;

        case HCI_CONTROL_EVENT_SECURITY_REQ:
            evt = BTM_SECURITY_REQUEST_EVT;
            break;

        case HCI_CONTROL_EVENT_SECURITY_FAILED:
            evt = BTM_SECURITY_FAILED_EVT;
            break;

        case HCI_CONTROL_EVENT_IO_CAPABILITIES_BR_EDR_REQUEST:
            evt = BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT;
            break;

// TODO: to add all events corresponding to this.

        case HCI_CONTROL_LE_EVENT_COMMAND_STATUS:
        {
            evt = 0xff;
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_LE_EVENT_COMMAND_STATUS\n"));
            break;
        }
        case HCI_CONTROL_LE_EVENT_SCAN_STATUS:
        {
            evt = 0xff;
            STREAM_TO_UINT8( data.ble_scan_state_changed, payload );
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_LE_EVENT_SCAN_STATUS: %d\n", data.ble_scan_state_changed));
            break;
        }

        case HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT:
        {
            evt = 0xff;
            WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
            break;
        }

        case HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE:
        {
            evt = BTM_BLE_ADVERT_STATE_CHANGED_EVT;
            STREAM_TO_UINT8( data.ble_advert_state_changed, payload);
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE[%s]\n",__func__));
            break;
        }

        case HCI_CONTROL_LE_EVENT_CONNECTED:
        {
            wiced_bt_gatt_event_data_t    event_data;
            wiced_bt_device_address_t  temp_bdadr;
            WICED_HCI_DEBUG_LOG(("[%d] LE Device Connected event\n",(int)len));

            if(payload)
            {
                STREAM_TO_UINT8( event_data.connection_status.addr_type , payload );
                STREAM_TO_BDADDR(temp_bdadr, payload);
                event_data.connection_status.bd_addr = temp_bdadr;
                STREAM_TO_UINT16( event_data.connection_status.conn_id, payload );
                STREAM_TO_UINT8( event_data.connection_status.link_role, payload );
                event_data.connection_status.transport = BT_TRANSPORT_LE;
                event_data.connection_status.connected = WICED_TRUE;

                wh_bt_gatt_context.gatt_mgmt_cb( GATT_CONNECTION_STATUS_EVT, &event_data );

                evt = 0xff;
            }
            break;
        }

        case HCI_CONTROL_LE_EVENT_DISCONNECTED:
        {
            wiced_bt_gatt_event_data_t    event_data;
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_LE_EVENT_DISCONNECTED:[%s]\n",__func__));

            evt = 0xff;
            memset(&event_data, 0, sizeof(wiced_bt_gatt_event_data_t));
            if(payload)
            {
                STREAM_TO_UINT16( event_data.connection_status.conn_id, payload );
                STREAM_TO_UINT8( event_data.connection_status.reason, payload );
                event_data.connection_status.connected = WICED_FALSE;

                wh_bt_gatt_context.gatt_mgmt_cb( GATT_CONNECTION_STATUS_EVT, &event_data );
            }
        }
        break;

        case HCI_CONTROL_LE_EVENT_IDENTITY_ADDRESS:
        {
            evt = 0xff;
            WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
            break;
        }
        case HCI_CONTROL_LE_EVENT_PEER_MTU:
        {
            evt = 0xff;
            WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
            break;
        }
        case HCI_CONTROL_EVENT_COMMAND_STATUS:
        {
#ifdef ENABLE_BT_PROTOCOL_TRACES
            uint8_t control_event_status;
            STREAM_TO_UINT8(control_event_status, payload);
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_EVENT_COMMAND_STATUS: %d\n", control_event_status));
#endif
            evt = 0xff;
            break;
        }
        case HCI_CONTROL_EVENT_WICED_TRACE:
        {
#ifdef ENABLE_BT_PROTOCOL_TRACES
            char str[200];
            memcpy(str,payload,len);
            str[len]='\0';
            WICED_HCI_DEBUG_LOG(("\n20706 Trace message:\n------------------------\n%s------------------------\n",str));
#endif
            evt = 0xff;
            break;
        }
        case HCI_CONTROL_EVENT_HCI_TRACE:
        {
            //WICED_HCI_DEBUG_LOG(("this is the HCI_CONTROL_EVENT_HCI_TRACE\n"));
            evt = 0xff;
            break;
        }
        case HCI_CONTROL_EVENT_PAIRING_COMPLETE:
        {
            WICED_HCI_DEBUG_LOG(("This is the HCI_CONTROL_EVENT_PAIRING_COMPLETE\n"));
            evt = BTM_PAIRING_COMPLETE_EVT;
        }
        break;
        case HCI_CONTROL_EVENT_CONNECTION_STATUS:
        {
            wiced_bool_t is_connected;
            uint8_t reason;

            STREAM_TO_UINT8( is_connected, payload );
            STREAM_TO_UINT8( reason, payload );

            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_EVENT_CONNECTION_STATUS is_connected:%d, reason:%d\n",is_connected, reason));
            if(wh_bt_dm_context.dm_callstatus_cb)
                wh_bt_dm_context.dm_callstatus_cb(NULL, NULL, is_connected, 0, 0, reason);

            evt = 0xff;
        }
        break;
        default:
            WICED_HCI_DEBUG_LOG((" [%s] this is the default case\n\n", __func__));
            evt = 0xff;
            break;

    }
    if(evt != 0xff)
        wh_bt_dm_context.dm_mgmt_cb( evt, &data );
}

void wiced_bt_set_local_bdaddr( wiced_bt_device_address_t  bda )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;

    length += ( sizeof(wiced_bt_device_address_t));
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    BDADDR_TO_STREAM(p, bda);
    wiced_hci_send(HCI_CONTROL_COMMAND_SET_LOCAL_BDA, data, length);
    free(data);
}

//TODO: this needs to be implemented on 20706

wiced_result_t wiced_bt_dev_write_eir ( uint8_t *p_buff, uint16_t len )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t data[BD_NAME_LEN+1];

    memcpy( &data[ 1 ], &p_buff[ 2 ], p_buff[ 0 ] );

    /* length of name is in the first byte of p_buff */
    data[ 0 ] = p_buff[ 0 ];

    /*Comenting below code since It is decided that 2070X Embedded application itself handles this functionality*/
    wiced_hci_send( HCI_CONTROL_COMMAND_SET_LOCAL_NAME, data, p_buff[ 0 ] + 1 );

    return result;
}

void wiced_bt_dev_read_local_addr (wiced_bt_device_address_t bd_addr)
{
    wiced_bt_device_address_t bda = {0,0,0,0,0,0};
    //TODO: need to check how to read the bd_addr.
    wiced_hci_send( HCI_CONTROL_COMMAND_READ_LOCAL_BDA, bda, sizeof(bda) );
}

wiced_result_t wiced_bt_dev_set_discoverability (uint8_t inq_mode, uint16_t window, uint16_t interval)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*        data  = NULL;
    uint16_t      length  = 2;
    uint8_t*      p = NULL;
    uint16_t discoveravle_connectable = 0x0101;
    uint16_t not_discoverable_connectable = 0x0000;

    /* 20706 uses one single API to do both discoverablity and connectablity(set_visiblity)
        * The first byte is to set Discoverablity and 2nd is to set Connectablity. When both are made 0 it means it's neither discoverable nor Connectable */
    WICED_HCI_DEBUG_LOG((" [%s] [inq_mode: %d]\n", __func__, inq_mode));

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    if( inq_mode != BTM_NON_DISCOVERABLE )
    {
        UINT16_TO_STREAM( p, discoveravle_connectable );
        wiced_hci_send( HCI_CONTROL_COMMAND_SET_VISIBILITY, data, length );
    }
    else
    {
        UINT16_TO_STREAM( p, not_discoverable_connectable );
        wiced_hci_send( HCI_CONTROL_COMMAND_SET_VISIBILITY, data, length );
    }
    free( data );
    return (result);
}

wiced_result_t wiced_bt_dev_set_connectability (uint8_t page_mode, uint16_t window, uint16_t interval)
{
    wiced_result_t result = WICED_SUCCESS;
    // TODO: To set the value as done in discoverablity and send the data.
    return (result);
}

void wiced_bt_dev_confirm_req_reply (wiced_result_t res, wiced_bt_device_address_t bd_addr)
{
    uint8_t*       data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      p = NULL;
    uint8_t       accept_pairing = WICED_TRUE;

    length = ((sizeof(wiced_bt_device_address_t)) + (sizeof(uint8_t)));
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    if( res == WICED_SUCCESS)
    {
        BDADDR_TO_STREAM(p, bd_addr);
        UINT8_TO_STREAM(p, accept_pairing);

        wiced_hci_send( HCI_CONTROL_COMMAND_USER_CONFIRMATION, data, length);
    }
    free(data);
}

wiced_result_t wiced_bt_dev_set_sniff_mode (wiced_bt_device_address_t remote_bda, uint16_t min_period,
        uint16_t max_period, uint16_t attempt,
        uint16_t timeout)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      p = NULL;

    length = ( sizeof(wiced_bt_device_address_t) + sizeof(min_period) + sizeof(max_period) + sizeof(attempt) + sizeof(timeout) );

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    BDADDR_TO_STREAM(p, remote_bda);
    UINT16_TO_STREAM(p, min_period);
    UINT16_TO_STREAM(p, max_period);
    UINT16_TO_STREAM(p, attempt);
    UINT16_TO_STREAM(p, timeout);

//    wiced_hci_send( HCI_CONTROL_COMMAND_SET_SNIFF_MODE, data, length );

    free(data);

    return (result);
}

wiced_result_t wiced_bt_dev_cancel_sniff_mode (wiced_bt_device_address_t remote_bda)
{
    wiced_result_t result = WICED_SUCCESS;

//    wiced_hci_send( HCI_CONTROL_COMMAND_CANCEL_SNIFF_MODE, remote_bda, sizeof(remote_bda) );

    return (result);
}

wiced_result_t wiced_bt_dev_set_sniff_subrating (wiced_bt_device_address_t remote_bda, uint16_t max_latency,
        uint16_t min_remote_timeout, uint16_t min_local_timeout)
{
    wiced_result_t retval = WICED_SUCCESS;
    uint8_t*       data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      p = NULL;

    length = ( sizeof(wiced_bt_device_address_t) + sizeof(max_latency) + sizeof(min_remote_timeout) + sizeof(min_local_timeout) );

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    BDADDR_TO_STREAM(p, remote_bda);
    UINT16_TO_STREAM(p, max_latency);
    UINT16_TO_STREAM(p, min_remote_timeout);
    UINT16_TO_STREAM(p, min_local_timeout);

//    wiced_hci_send( HCI_CONTROL_COMMAND_SET_SNIFF_SUBRATING, data, length );

    free(data);

    return (retval);
}

wiced_result_t wiced_bt_dev_read_rssi (wiced_bt_device_address_t remote_bda, wiced_bt_transport_t transport,
        wiced_bt_dev_cmpl_cback_t *p_cb)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      p = NULL;

    wh_bt_dm_context.dm_cmpl_cb = p_cb;

    length = ( sizeof(wiced_bt_device_address_t) + sizeof(transport) );

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    BDADDR_TO_STREAM(p, remote_bda );
    UINT8_TO_STREAM(p, transport );
//    wiced_hci_send( HCI_CONTROL_COMMAND_READ_RSSI, data, length );

    free(data);
    return (result);
}

wiced_result_t wiced_bt_set_pairing_mode(uint8_t pairing_mode)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      p = NULL;

    length = sizeof(pairing_mode);

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    UINT8_TO_STREAM( p, pairing_mode );

    wiced_hci_send( HCI_CONTROL_COMMAND_SET_PAIRING_MODE, data, length );

    free(data);

    return (result);
}

wiced_result_t wiced_bt_dev_read_tx_power (wiced_bt_device_address_t remote_bda, wiced_bt_transport_t transport,
                                            wiced_bt_dev_cmpl_cback_t *p_cback)
{
/*  TODO: Need to Implement this function.(Used by ble_proximity_reporter) */
/* not supported on the Embedded side yet */

    return WICED_SUCCESS;
}

wiced_bt_dev_status_t wiced_bt_dev_register_connection_status_change(wiced_bt_connection_status_change_cback_t *p_wiced_bt_connection_status_change_cback)
{
    wiced_bt_dev_status_t retval = WICED_SUCCESS;

    wh_bt_dm_context.dm_callstatus_cb = p_wiced_bt_connection_status_change_cback;

    return (retval);
}

/**
 * Function         wiced_bt_dev_push_nvram_data
 * .....MCU host push all the saved NVRAM informatoin mainly paired device Info
 *
 * @param[in]       paired_device_info : Remote device address, Link key
 *
 * @return          WICED_BT_SUCCESS if successful
 *
 */
wiced_result_t wiced_bt_dev_push_nvram_data(wiced_bt_device_link_keys_t *paired_device_info)
{
    uint8_t*                data  = NULL;
    uint8_t*                p     = NULL;
    //TODO: Currently using nvram_id 0x10, that commonly received from 2070x but need to enhance the code to use the actual value saved in NVRAM
    uint16_t                nvram_id = 0x10;
    uint16_t                length  = 0;

    length += ( sizeof(uint16_t) + sizeof(wiced_bt_device_link_keys_t));
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    UINT16_TO_STREAM(p, nvram_id);

    memcpy(p, paired_device_info->bd_addr, sizeof(wiced_bt_device_address_t));
    p += sizeof(wiced_bt_device_address_t);

    memcpy(p, &paired_device_info->key_data.br_edr_key_type, sizeof(uint8_t));
    p += sizeof(uint8_t);

    memcpy(p, paired_device_info->key_data.br_edr_key, LINK_KEY_LEN);
    p += LINK_KEY_LEN;

    memcpy(p, &paired_device_info->key_data.le_keys_available_mask, sizeof(wiced_bt_dev_le_key_type_t));
    p += sizeof(wiced_bt_dev_le_key_type_t);

    memcpy(p, &paired_device_info->key_data.ble_addr_type, sizeof(wiced_bt_ble_address_type_t));
    p += sizeof(wiced_bt_ble_address_type_t);

    memcpy(p, &paired_device_info->key_data.static_addr_type, sizeof(wiced_bt_ble_address_type_t));
    p += sizeof(wiced_bt_ble_address_type_t);

    memcpy(p, paired_device_info->key_data.static_addr, sizeof(wiced_bt_device_address_t));
    p += sizeof(wiced_bt_device_address_t);

    memcpy(p, &paired_device_info->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
    p += sizeof(wiced_bt_ble_keys_t);

    wiced_hci_send(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_bool_t wiced_bt_dev_allow_host_sleep( void )
{
    return WICED_TRUE;
}

