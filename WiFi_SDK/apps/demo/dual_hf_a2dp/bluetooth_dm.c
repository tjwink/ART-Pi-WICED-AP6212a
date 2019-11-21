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

/*
 *@bluetooth_dm.c : Bluetooth Device Manager/ service discover/ Device discovery
 *
 */

#include "wiced.h"
#include "bluetooth_dm.h"
#include "app_dct.h"
#include "rtos.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc.h"
#include "bluetooth_nv.h"
#include "dual_a2dp_hfp_audio.h"
#include "wiced_bt_sdp.h"
#include "hashtable.h"
#include "bluetooth_hfp.h"

/******************************************************
 *                      Macros
 ******************************************************/
    // SDP Record for Device ID
#define HDLR_DEVICE_ID                        0x10000
    // SDP Record for A2DP Sink
#define HDLR_A2DP_SINK                        0x10001
    // SDP Record for AVRC Controller
#define HDLR_AVRC_CONTROLLER                  0x10002
    // SDP Record for AVRC Target
#define HDLR_AVRC_TARGET                      0x10003
    // SDP Record for Hands-Free Unit
#define HDLR_HANDS_FREE_UNIT                  0x10004

#define BLUETOOTH_DEVICE_NAME           "WICED AUDIO"

#define EIR_DATA_LENGTH                 240
#define EIR_COMPLETE_LOCAL_NAME_TYPE    0x09
#define EIR_COMPLETE_16BITS_UUID_TYPE   0x03
#define BLUETOOTH_MAX_CONNECTIONS              2

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

static void bt_audio_reconnect_on_link_loss(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint8_t bluetooth_device_name[249] = BLUETOOTH_DEVICE_NAME;

uint8_t uuid_list[] =
{
    0x08, 0x11, /* Headset */
    0x1E, 0x11, /* Handsfree */
    0x0E, 0x11, /* AV Remote Control */
    0x0B, 0x11, /* Audio Sink */
};


const uint8_t sdp_database[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_2(391),

    // SDP Record for Device ID
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HDLR_DEVICE_ID),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),
        SDP_ATTR_PROTOCOL_DESC_LIST(1),
        SDP_ATTR_BROWSE_LIST,
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x000F),
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0000),
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(76),
        SDP_ATTR_RECORD_HANDLE(HDLR_A2DP_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(0x0102),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                SDP_ATTR_VALUE_UINT2(0x0102),
        SDP_ATTR_SERVICE_NAME(15),
            'W', 'I', 'C', 'E', 'D', ' ', 'A', '2', 'D', 'P', ' ', 'S', 'I', 'N', 'K',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(78),
        SDP_ATTR_RECORD_HANDLE(HDLR_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
            SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
            SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(0x0103),
        SDP_ATTR_SERVICE_NAME(14),
            'W', 'I', 'C', 'E', 'D', ' ', 'A', 'V', 'R', 'C', 'P', ' ', 'C', 'T',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0001),

    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(75),
        SDP_ATTR_RECORD_HANDLE(HDLR_AVRC_TARGET),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x0104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(0x105),
        SDP_ATTR_SERVICE_NAME(14),
            'W', 'I', 'C', 'E', 'D', ' ', 'A', 'V', 'R', 'C', 'P', ' ', 'T', 'G',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0002),

    // SDP Record for Hands-Free Unit
    SDP_ATTR_SEQUENCE_1(75),
        SDP_ATTR_RECORD_HANDLE(HDLR_HANDS_FREE_UNIT),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
            SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
            SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(HANDS_FREE_SCN),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_VALUE_UINT2(0x0106),
        SDP_ATTR_SERVICE_NAME(15),
            'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0016),
};

/* Name, address and class of device shall be over-written using DCT entries if present.
* Please use the DCT entries to configure these parameters */
wiced_bt_cfg_settings_t wiced_bt_audio_cfg_settings =
{
    .device_name               = (uint8_t*)BLUETOOTH_DEVICE_NAME,
    .device_class              = {0x24, 0x04, 0x18},
    .security_requirement_mask = BTM_SEC_NONE,
    .max_simultaneous_links    = 3,/*TBD*/

    /*BR/EDR scan settings */
    .br_edr_scan_cfg =
    {
        .inquiry_scan_type     = BTM_SCAN_TYPE_INTERLACED,
        .inquiry_scan_interval = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,
        .inquiry_scan_window   = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,

        .page_scan_type        = BTM_SCAN_TYPE_INTERLACED,
        .page_scan_interval    = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,
        .page_scan_window      = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW,
    },

    /*BLE scan settings */
    .ble_scan_cfg =
        {
            .scan_mode = BTM_BLE_SCAN_MODE_NONE,
        },

    .rfcomm_cfg =
        {
            .max_links = 4,
            .max_ports = 20,
        },

    .l2cap_application =
     {
         .max_links       = 6,
         .max_psm         = 5, /*TBD*/
         .max_channels    = 12,
     },

     .avdt_cfg =
     {
         .max_links = 4,
     },

     /* Audio/Video Remote Control configuration */
     .avrc_cfg =
     {
         .roles     = AVRC_CONN_ACCEPTOR, /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
         .max_links = 4  /**< Maximum simultaneous remote control links */
     },

     .max_number_of_buffer_pools = WICED_BT_CFG_NUM_BUF_POOLS,

    .addr_resolution_db_size = 3,
};

const wiced_bt_cfg_buf_pool_t wiced_bt_audio_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 268,       60   },      /* Small Buffer Pool */
    { 360,      20   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 660,      40  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

static bluetooth_dm_context_t dm_ctxt_data =
{
    .connect_state  = WICED_FALSE,
    .num_retries    = 0,
};

wiced_bt_device_address_t local_address = {0x81,0x34,0x09,0xAD,0xCD,0xAC};
static uint8_t connected_devices = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_audio_write_eir( uint8_t *device_name )
{
    uint8_t eir_cfg[EIR_DATA_LENGTH] = {0};
    uint8_t* p = eir_cfg;
    uint8_t name_len = strlen((char*)device_name);

    *p++ = (uint8_t)(name_len+1);                 /* Length */
    *p++ = (uint8_t)EIR_COMPLETE_LOCAL_NAME_TYPE; /* EIR Data Type */
    memcpy(p, device_name, name_len);   /* Name string */
    p += name_len;

    *p++ = sizeof(uuid_list)+1;
    *p++ = (uint8_t) EIR_COMPLETE_16BITS_UUID_TYPE;
    memcpy(p, uuid_list, sizeof(uuid_list));

    return wiced_bt_dev_write_eir(eir_cfg, EIR_DATA_LENGTH);
}

void bt_audio_init_services( void )
{
    wiced_app_service_t *service = NULL;

    service = wiced_get_entry(SERVICE_BT_HFP);
    if(service != NULL)
    {
        service->init_service();
    }

    service = wiced_get_entry(SERVICE_BT_A2DP);
    if(service != NULL)
    {
        service->init_service();
    }
}


bluetooth_dm_context_t* wiced_bt_dm_get_context( void )
{
   return &dm_ctxt_data;
}

extern void bt_audio_avrcp_disconnect_service(void);
extern void bt_audio_a2dp_disconnect_service(void);
void bt_audio_disconnect_services(void)
{
    app_queue_element_t message;

    wiced_rtos_lock_mutex( &app_data.lock );

    /* Order for disconnecting AVRCP, A2DP and then HFP */

    message.event = WICED_BT_CONNECT_AVRCP;
    message.function = bt_audio_avrcp_disconnect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    message.event = WICED_BT_DISCONNECT_A2DP;
    message.function = bt_audio_a2dp_disconnect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    message.event = WICED_BT_DISCONNECT_HFP;
    message.function = wiced_get_entry(SERVICE_BT_HFP)->disconnect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    wiced_rtos_unlock_mutex( &app_data.lock );

    return;
}

extern void bt_audio_avrcp_connect_service(void);
extern void bt_audio_a2dp_connect_service(void);

void bt_audio_connect_services(void)
{
    app_queue_element_t message;

    wiced_rtos_lock_mutex( &app_data.lock );

    /* Order for connecting HFP and then A2DP, AVRCP */

    message.event = WICED_BT_CONNECT_HFP;
    message.function = wiced_get_entry(SERVICE_BT_HFP)->connect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    message.event = WICED_BT_CONNECT_A2DP;
    message.function = bt_audio_a2dp_connect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    message.event = WICED_BT_CONNECT_AVRCP;
    message.function = bt_audio_avrcp_connect_service;
    message.wait_for_event_complete = WICED_TRUE;
    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    wiced_rtos_unlock_mutex( &app_data.lock );

    return;
}

static void bt_audio_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    dm_ctxt_data.connect_state = is_connected;

    WPRINT_APP_INFO( ("[BT-DM] Connection status callback is_connected:%d reason:%x\n", is_connected, reason ) );

    if( is_connected )
    {
        if( connected_devices >= (BLUETOOTH_MAX_CONNECTIONS-1) )
        {
            wiced_bt_dev_set_discoverability( BTM_NON_DISCOVERABLE, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval );
        }
        connected_devices++;
    }
    else
    {
        if( reason == HCI_ERR_CONNECTION_TOUT )
        {
            WPRINT_APP_INFO( ("[BT-DM] Device disconnected(link-loss). Starting Reconnection-Timer...\n") );
            wiced_rtos_start_timer( &dm_ctxt_data.timer );
        }
        connected_devices--;
        wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval );
    }
}


static void bt_audio_dm_linkloss_timer_handler( void )
{
    if( dm_ctxt_data.num_retries >= DM_LINK_LOSS_NUM_RETRIES || dm_ctxt_data.connect_state == WICED_TRUE )
    {
        WPRINT_APP_INFO( ("[BT-DM] Reconnection timer - num of connection-retry:%d is_connected:%d\n", dm_ctxt_data.num_retries, dm_ctxt_data.connect_state ) );
        dm_ctxt_data.num_retries = 0;
        wiced_rtos_stop_timer(&dm_ctxt_data.timer);
        return;
    }
    else
    {
        bt_audio_reconnect_on_link_loss();
        dm_ctxt_data.num_retries++;
    }
}


static wiced_bt_dev_status_t bt_audio_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda;
    wiced_result_t result = WICED_ERROR;

    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:
            bt_audio_hfp_sco_connection_evt_handler(&p_event_data->sco_connected);
        break;

        case BTM_SCO_DISCONNECTED_EVT:
            bt_audio_hfp_sco_disconnection_evt_handler(&p_event_data->sco_disconnected);
        break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:
            bt_audio_hfp_sco_connection_request_evt_handler(&p_event_data->sco_connection_request);
        break;

        case BTM_ENABLED_EVT:
        {
            /* Bluetooth controller and host stack enabled */
            WPRINT_APP_INFO( ("bt_audio_management_callback: Bluetooth enabled (%s)\n", ((p_event_data->enabled.status == WICED_BT_SUCCESS) ? "success":"failure")) );

            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
                /* Configure the Device Name and Class of Device from the DCT */
                platform_dct_bt_config_t* dct_bt_config;
                 // Read config
                wiced_dct_read_lock( (void**) &dct_bt_config, WICED_TRUE, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t) );
                WPRINT_APP_INFO( ("WICED DCT BT ADDR 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x \r\n",
                                  dct_bt_config->bluetooth_device_address[0], dct_bt_config->bluetooth_device_address[1],
                                  dct_bt_config->bluetooth_device_address[2], dct_bt_config->bluetooth_device_address[3],
                                  dct_bt_config->bluetooth_device_address[4], dct_bt_config->bluetooth_device_address[5]) );
                wiced_bt_set_local_bdaddr ( dct_bt_config->bluetooth_device_address );
                WPRINT_APP_INFO( ("bt_audio_management_callback:wiced_bt_dev_write_local_addr result = 0x%x\n", (unsigned int)result) );
                result = bt_audio_write_eir(dct_bt_config->bluetooth_device_name);
                wiced_dct_read_unlock( (void*) dct_bt_config, WICED_TRUE );
#else
                result = bt_audio_write_eir(BLUETOOTH_DEVICE_NAME);
#endif
                WPRINT_APP_INFO( ("bt_audio_management_callback: wiced_bt_dev_write_eir result = 0x%x\n", (unsigned int)result) );

                wiced_bt_dev_read_local_addr( bda );
                WPRINT_APP_INFO( ("bt_audio_management_callback:Local Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]) );

                /**
                 * TODO Remove this, and make changes in the embedded app to solicit this information.
                 */
#ifdef USE_WICED_HCI
                {
                    uint8_t count = 0;
                    bt_audio_paired_device_info_t out_device;
                    wiced_bt_device_link_keys_t link_keys;

                    for ( count = 0; count < BT_AUDIO_NV_MAX_LAST_PAIRED_DEVICES; count++)
                    {
                        if( bt_audio_nv_get_device_info_by_index(count, &out_device) == WICED_BADARG )
                        {
                            WPRINT_APP_INFO( ("%s no more entries in NVRAM\n", __func__) );
                            break;
                        }
                        else
                        {
                            WPRINT_APP_INFO(("%s Bluetooth address of the paired device extracted \n", __func__));
                            WPRINT_APP_INFO(("from NVRAM: %x %x %x %x %x %x\n",out_device.device_link.bd_addr[0], out_device.device_link.bd_addr[1],
                                    out_device.device_link.bd_addr[2],out_device.device_link.bd_addr[3], out_device.device_link.bd_addr[4], out_device.device_link.bd_addr[5]));
                            memcpy(&link_keys,&out_device.device_link,sizeof(out_device.device_link));
                            wiced_bt_dev_push_nvram_data(&link_keys);
                        }
                    }
                }
#endif
                result = wiced_bt_sdp_db_init( ( uint8_t * )sdp_database, sizeof( sdp_database ) );
                WPRINT_APP_INFO( ("bt_audio_management_callback: wiced_bt_sdp_db_init result (bool) = 0x%x\n", (unsigned int)result) );

                result = wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval);
                WPRINT_APP_INFO( ("bt_audio_management_callback: discoverability result = 0x%x\n", (unsigned int)result) );

                result = wiced_bt_dev_set_connectability (BTM_CONNECTABLE, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.page_scan_window, wiced_bt_audio_cfg_settings.br_edr_scan_cfg.page_scan_interval);
                WPRINT_APP_INFO( ("bt_audio_management_callback:connectability result = 0x%x\n", (unsigned int)result) );

                bt_audio_init_services();

                result = wiced_bt_dev_register_connection_status_change( bt_audio_connection_status_callback );
                WPRINT_APP_INFO( ("bt_audio_management_callback registering acl_change callback result: 0x%x, cb = 0x%x\n", result, (unsigned int) bt_audio_connection_status_callback) );
            }
        }
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: IO capabilities BR/EDR request \n") );
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: User confirmation request \n") );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: Link key update evt\n") );
            bt_audio_nv_update_device_link_key( &p_event_data->paired_device_link_keys_update );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            {
                bt_audio_paired_device_info_t out_device;

                WPRINT_APP_INFO( ("bt_audio_management_callback: Link key request evt\n") );
                if(WICED_SUCCESS == bt_audio_nv_get_device_info_by_addr( &p_event_data->paired_device_link_keys_request.bd_addr, &out_device ) )
                {
                    memcpy(&p_event_data->paired_device_link_keys_request.key_data, &out_device.device_link.key_data, sizeof( out_device.device_link.key_data ) );
                }
                else
                {
                    status = WICED_BT_ERROR;
                }
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: Local ID key update evt\n") );
            bt_audio_nv_update_local_id_keys( &p_event_data->local_identity_keys_update );
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
#ifndef USE_WICED_HCI
            if(WICED_SUCCESS != bt_audio_nv_get_local_id_keys( &p_event_data->local_identity_keys_request ))
                status = WICED_BT_ERROR;
#endif
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:
            bt_audio_hfp_sco_connection_change_evt_handler(&p_event_data->sco_connection_change);
        break;

        case BTM_SECURITY_REQUEST_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: Security reqeust\n") );
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WPRINT_APP_INFO( ("bt_audio_management_callback: Pairing complete %i.\n", p_event_data->pairing_complete.pairing_complete_info.ble.status) );
            break;

        default:
            WPRINT_APP_INFO( ("bt_audio_management_callback: Unhandled Bluetooth Management Event: 0x%x\n", event) );
            break;
    }

    return ( status );
}


void bt_audio_enable_bluetooth( void )
{
    wiced_result_t result;
    bt_audio_nv_init();

    result = wiced_bt_stack_init( bt_audio_management_callback, &wiced_bt_audio_cfg_settings, wiced_bt_audio_cfg_buf_pools );
    WPRINT_APP_INFO( ("bt_audio_enable_bluetooth: result: 0x%x, cb = 0x%x\n", result, (unsigned int) bt_audio_management_callback) );

    wiced_rtos_init_timer( &dm_ctxt_data.timer, DM_LINK_LOSS_RETRY_INTERVAL, (timer_handler_t)bt_audio_dm_linkloss_timer_handler, NULL );
}

static void bt_audio_reconnect_on_link_loss(void)
{
    app_queue_element_t message;

    wiced_rtos_lock_mutex( &app_data.lock );

    message.event                   = WICED_BT_RECONNECT_ON_LINK_LOSS;
    message.function                = bt_audio_connect_services;
    message.wait_for_event_complete = WICED_FALSE;

    wiced_rtos_push_to_queue( &app_data.queue, &message, WICED_NO_WAIT );

    wiced_rtos_unlock_mutex(&app_data.lock);
}
