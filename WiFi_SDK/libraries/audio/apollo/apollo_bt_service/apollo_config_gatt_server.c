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
 * BLE Vendor Specific Device
 *
 */

#include <string.h>
#include "wiced_log.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "apollo_streamer.h"
#include "apollo_config_gatt_server_private.h"
#include "apollo_config_gatt_server.h"
#include "apollo_bt_main_service_private.h"
#include "apollocore.h"

/******************************************************************************
 *                                Constants
******************************************************************************/

// Two bytes for Broadcom company ID in adv packet
#define ADV_MANU_ID_BCM_BYTE0               0x0F
#define ADV_MANU_ID_BCM_BYTE1               0x00

// Two bytes for Broadcom Apollo config adv packet
#define ADV_TYPE_APOLLO_CONFIG_BYTE0        0xF1
#define ADV_TYPE_APOLLO_CONFIG_BYTE1        0xF2

#define ADV_DATA_APOLLO_CONFIG_DEVICE       0x00
#define ADV_DATA_APOLLO_CONFIG_SOURCE       0X01  // device was configured as source
#define ADV_DATA_APOLLO_CONFIG_SINK         0x02  // device was configured as sink
#define ADV_DATA_APOLLO_CONFIG_SOURCE_A2DP  0x03  // device was configured as a2dp sink

#define GATT_ATTRIBUTE_SIZE                 APOLLO_CONFIG_GATT_ATTRIBUTE_SIZE

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

static void                     apollo_config_application_init                  ( void );
static wiced_bt_gatt_status_t   apollo_config_gatt_server_connection_handler    ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   apollo_config_gatt_server_connection_up         ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   apollo_config_gatt_server_connection_down       ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   apollo_config_gatt_server_callback              ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t   read_request_handler                            ( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t   write_request_handler                           ( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                     apollo_config_set_advertisement_data            ( uint8_t type );
static wiced_bt_gatt_status_t   write_and_execute_request_handler               ( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag );
static void                     apollo_config_read_dct                          ( apollo_config_gatt_server_dct_t *dct );
static void                     apollo_config_write_dct                         ( apollo_config_gatt_server_dct_t *dct );
static wiced_result_t           apollo_config_advertisement_start               ( void );

/******************************************************************************
 *                                Structures
 ******************************************************************************/

typedef struct
{
    BD_ADDR         remote_addr;                            /* remote peer device address */
    uint32_t        timer_count;                            /* timer count */
    uint32_t        fine_timer_count;                       /* fine timer count */
    uint16_t        conn_id;                                /* connection ID referenced by the stack */
    uint16_t        peer_mtu;                               /* peer MTU */
    uint8_t         flag_indication_sent;                   /* indicates waiting for confirmation */
    uint8_t         flag_stay_connected;                    /* stay connected or disconnect after all messages are sent */
    uint8_t         battery_level;                          /* dummy battery level */
    wiced_bool_t    app_dct_dirty;                          /* dirty flag for determining what to save */
} apollo_config_state_t;

typedef PACKED struct
{
    BD_ADDR         bdaddr;                                 /* BD address of the bonded host */
    uint16_t        characteristic_client_configuration;    /* Current value of the client configuration descriptor */
    uint8_t         number_of_blinks;                       /* Sensor config, number of times to blink the LEd when button is pushed. */
} host_info_t;

typedef struct
{
    uint16_t        handle;
    uint16_t        attr_len;
    void*           p_attr;
} attribute_t;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

static uint8_t apollo_config_device_name[ ]                                 = "Apollo";
static uint8_t apollo_config_appearance_name[2]                             = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };

static uint8_t apollo_config_char_nw_mode_value                             = APOLLO_ROLE_SINK;
static uint8_t apollo_config_char_nw_security_value                         = WICED_SECURITY_OPEN;
//AP name
static uint8_t apollo_config_char_nw_ssid_value[GATT_ATTRIBUTE_SIZE]        = "wiced_audio_2g";
static uint8_t apollo_config_char_nw_passphrase_value[GATT_ATTRIBUTE_SIZE]  = "wiced-audio";
static uint8_t apollo_config_char_nw_channel_value                          = 36;
static uint8_t apollo_config_char_nw_band_value                             = 5;//"5GHZ";

static uint8_t apollo_config_char_spk_name_value[GATT_ATTRIBUTE_SIZE]       = "Bedroom spk";
static uint8_t apollo_config_char_spk_channel_value                         = CHANNEL_MAP_FL;
static uint8_t apollo_config_char_spk_output_port_value[ ]                  = "wm8553";
static uint8_t apollo_config_char_spk_output_vol_value                      = 0XDD;

static uint8_t apollo_config_char_src_input_value                           = APOLLO_AUDIO_SOURCE_BT;
static uint8_t apollo_config_char_src_loopback_value[ ]                     = "False";
static uint8_t apollo_config_char_src_input_port_value[ ]                   = "ak4964";
static uint8_t apollo_config_char_src_input_vol_value                       = 0xAA;

static char    apollo_config_char_mfr_name_value[ ]                         = { 'B', 'r', 'o', 'a', 'd', 'c', 'o', 'm', 0, };
static char    apollo_config_char_model_num_value[ ]                        = { '1', '2', '3', '4',   0,   0,   0,   0 };
static uint8_t apollo_config_char_system_id_value[ ]                        = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

static apollo_config_state_t                apollo_config_state;
static host_info_t                          apollo_config_hostinfo;
static wiced_bool_t                         is_connected = FALSE;

static apollo_config_gatt_server_dct_t      apollo_gatt_dct;

static apollo_config_gatt_server_params_t   apollo_gatt_params;

static attribute_t gatt_user_attributes[ ] =
{
    { HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME_VAL,        sizeof( apollo_config_device_name )                , apollo_config_device_name },
    { HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,  sizeof( apollo_config_appearance_name )            , apollo_config_appearance_name },

    { HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL,             APOLLO_ROLE_SOURCE                                 , &apollo_config_char_nw_mode_value },
    { HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL,         WICED_SECURITY_OPEN                                , &apollo_config_char_nw_security_value},
    { HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL,             sizeof( apollo_config_char_nw_ssid_value )         , apollo_config_char_nw_ssid_value },
    { HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL,       sizeof( apollo_config_char_nw_passphrase_value)    , apollo_config_char_nw_passphrase_value },
    { HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL_VAL,       36                                                 , &apollo_config_char_nw_channel_value },
    { HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND_VAL,             5                                                  , &apollo_config_char_nw_band_value },

    { HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL,        sizeof( apollo_config_char_spk_name_value)         , apollo_config_char_spk_name_value },
    { HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL_VAL,     CHANNEL_MAP_FL                                     , &apollo_config_char_spk_channel_value },
    { HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT_VAL, sizeof( apollo_config_char_spk_output_port_value)  , apollo_config_char_spk_output_port_value },
    { HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL,   1                                                 , &apollo_config_char_spk_output_vol_value },

    { HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL,        APOLLO_AUDIO_SOURCE_BT                             , &apollo_config_char_src_input_value },
    { HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK_VAL,     sizeof( apollo_config_char_src_loopback_value)     , apollo_config_char_src_loopback_value },
    { HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT_VAL,   sizeof( apollo_config_char_src_input_port_value)   , apollo_config_char_src_input_port_value },
    { HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL,    1                                                  , &apollo_config_char_src_input_vol_value },


    { HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,   sizeof(apollo_config_char_mfr_name_value)          , apollo_config_char_mfr_name_value },
    { HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,  sizeof(apollo_config_char_model_num_value)         , apollo_config_char_model_num_value },
    { HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,  sizeof(apollo_config_char_system_id_value)         , apollo_config_char_system_id_value },
    { HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL_VAL,       1                                                  , &apollo_config_state.battery_level },
};

extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Apollo configuration application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
static const uint8_t apollo_config_gatt_server_database[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_APOLLO_CONFIG_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
       characteristics of GAP service */
    PRIMARY_SERVICE_UUID16( HANDLE_APOLLO_CONFIG_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare proprietary Apollo Config Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_APOLLO_CONFIG_NW_SERVICE, UUID_APOLLO_CONFIG_SERVICE ),

        /* Declare characteristic for  mode  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE, HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_MODE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Security  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY, HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_SECURITY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for  SSID  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID, HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_SSID, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Passphrase  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE, HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_PASSPHRASE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

            /* Declare characteristic for Channel  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL, HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_CHANNEL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

            /* Declare characteristic for Band  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND, HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_BAND, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),


        /* Declare characteristic for Speaker name  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME, HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_NAME, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Speaker channel  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL, HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_CHANNEL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for output port  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT, HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_OUTPUT_PORT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for output volume  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL, HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_OUTPUT_VOL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Source name  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT, HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Source Loopback */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK, HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK_VAL,
            UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_LOOPBACK, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Source input port */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT, HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT_VAL,
                UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT_PORT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        /* Declare characteristic for Source input volume */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL, HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL,
                UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT_VOL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
             LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

    /* Declare Device Info service */
    PRIMARY_SERVICE_UUID16( HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_APOLLO_CONFIG_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

static inline uint8_t advertisement_type_translator(void)
{
    uint8_t advert_type;

    if ( apollo_config_char_nw_mode_value == APOLLO_ROLE_SINK )
    {
        advert_type = ADV_DATA_APOLLO_CONFIG_SINK;
    }
    else if ( apollo_config_char_nw_mode_value == APOLLO_ROLE_SOURCE )
    {
        advert_type = ADV_DATA_APOLLO_CONFIG_SOURCE;
    }
    else
    {
        advert_type = ADV_DATA_APOLLO_CONFIG_DEVICE;
    }

    return advert_type;
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void apollo_config_application_init( void )
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config_application_init\n");

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( apollo_config_gatt_server_callback );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_gatt_register: %d\n", gatt_status);

    gatt_status =  wiced_bt_gatt_db_init( apollo_config_gatt_server_database, sizeof( apollo_config_gatt_server_database ) );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_gatt_db_init %d\n", gatt_status);

    /* Set the advertising parameters and make the device discoverable */
    apollo_config_set_advertisement_data( advertisement_type_translator( ) );

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_start_advertisements %d\n", result);

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    apollo_config_state.flag_stay_connected = 1;
    apollo_config_state.app_dct_dirty = WICED_FALSE;
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void apollo_config_set_advertisement_data(uint8_t type)
{
    wiced_result_t                              result;
    wiced_bt_ble_advert_elem_t                  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;
    uint8_t apollo_service_uuid[LEN_UUID_128]   = { UUID_APOLLO_CONFIG_SERVICE };

    adv_elem[num_elem].advert_type              = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len                      = 1;
    adv_elem[num_elem].p_data                   = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type              = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len                      = strlen((const char *)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data                   = (uint8_t *)wiced_bt_cfg_settings.device_name;
    num_elem++;

    adv_elem[num_elem].advert_type              = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len                      = LEN_UUID_128;
    adv_elem[num_elem].p_data                   = apollo_service_uuid;
    num_elem ++;

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_ble_set_advertisement_data %d\n", result);
}
/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
wiced_result_t apollo_config_advertisement_stopped(void)
{
    wiced_bt_dev_status_t result = WICED_ERROR;
    if ( apollo_config_state.flag_stay_connected && !apollo_config_state.conn_id )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "ADV stopped\n");
    }
    return result;
}

static wiced_result_t apollo_config_advertisement_start(void)
{
    wiced_result_t result;

    result = apollo_gatt_params.gatt_event_cbf( APOLLO_CONFIG_GATT_EVENT_DCT_READ, &apollo_gatt_dct, apollo_gatt_params.user_context );

    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_gatt_server_event_callback failed !\n");
        goto _exit;
    }
    apollo_config_read_dct( &apollo_gatt_dct );
    apollo_config_application_init( );

 _exit:
    return result;
}

static void apollo_config_read_dct( apollo_config_gatt_server_dct_t *dct )
{
    if(dct->is_configured == 1)
    {
        apollo_config_char_nw_mode_value = dct->mode;
        memcpy(apollo_config_char_nw_ssid_value, dct->nw_ssid_name, sizeof(apollo_config_char_nw_ssid_value));
        memcpy(apollo_config_char_nw_passphrase_value, dct->nw_pass_phrase, sizeof(apollo_config_char_nw_passphrase_value));
        if (dct->mode == APOLLO_ROLE_SOURCE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo device was configured as a SOURCE!\n");
            apollo_config_char_src_input_value = dct->src_type;
            apollo_config_char_src_input_vol_value = dct->src_vol;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "ssid = %s\n", dct->nw_ssid_name);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "passphrase = %s\n", dct->nw_pass_phrase);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "source input = %d\n", dct->src_type);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "source volume = %d\n", dct->src_vol);
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo device was configured as a SINK!\n");
            apollo_config_char_spk_channel_value = dct->spk_channel_map;
            apollo_config_char_spk_output_vol_value = dct->spk_vol;
            memcpy(apollo_config_char_spk_name_value, dct->spk_name, sizeof(apollo_config_char_spk_name_value));
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "ssid = %s\n", dct->nw_ssid_name);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "passphrase = %s\n", dct->nw_pass_phrase);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "speaker name = %s\n", dct->spk_name);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "speaker channel = %d\n", dct->spk_channel_map);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "speaker volume = %d\n", dct->spk_vol);
        }
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config : Apollo Audio device was not configured previously\n");
    }
}

static void apollo_config_write_dct( apollo_config_gatt_server_dct_t *dct )
{

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config : apollo_config_write_dct\n");
    apollo_gatt_dct.is_configured = 1;
    apollo_gatt_params.gatt_event_cbf( APOLLO_CONFIG_GATT_EVENT_DCT_WRITE, &apollo_gatt_dct, apollo_gatt_params.user_context );
}

/*
 * Find attribute description by handle
 */
static attribute_t * apollo_config_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gatt_user_attributes ) / sizeof( gatt_user_attributes[0] ); i++ )
    {
        if ( gatt_user_attributes[i].handle == handle )
        {
            return ( &gatt_user_attributes[i] );
        }
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "attribute not found:%x\n", handle );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t read_request_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = apollo_config_get_attribute(p_read_data->handle) ) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch(p_read_data->handle){

        case HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME_VAL:
            puAttribute->p_attr = (void *) wiced_bt_cfg_settings.device_name;
            puAttribute->attr_len  = strlen((char*)wiced_bt_cfg_settings.device_name);
            break ;

        case HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL:
            puAttribute->p_attr = apollo_config_appearance_name;
            puAttribute->attr_len    = strlen((char*)apollo_config_appearance_name);
            break;

        case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL:
            puAttribute->p_attr = &apollo_config_char_nw_mode_value;
            puAttribute->attr_len    = sizeof(apollo_config_char_nw_mode_value);
            break;

        case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL :
            puAttribute->p_attr = &apollo_config_char_nw_security_value;
            puAttribute->attr_len = sizeof(apollo_config_char_nw_security_value);
            break;

        case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL:
            puAttribute->p_attr = apollo_config_char_nw_ssid_value;
            puAttribute->attr_len    = strlen((char*)apollo_config_char_nw_ssid_value);
            break;

        case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL :
            puAttribute->p_attr = apollo_config_char_nw_passphrase_value;
            puAttribute->attr_len = strlen((char*) apollo_config_char_nw_passphrase_value);
            break;

         case HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL_VAL :
            puAttribute->p_attr   = &apollo_config_char_nw_channel_value;
            puAttribute->attr_len = sizeof(apollo_config_char_nw_channel_value);
            break;

         case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND_VAL :
            puAttribute->p_attr   = &apollo_config_char_nw_band_value;
            puAttribute->attr_len = sizeof(apollo_config_char_nw_band_value);
            break;

         case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL :
             puAttribute->p_attr = apollo_config_char_spk_name_value;
             puAttribute->attr_len   = strlen((char*) apollo_config_char_spk_name_value);
             break;

         case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL_VAL :
             puAttribute->p_attr = &apollo_config_char_spk_channel_value;
             puAttribute->attr_len = sizeof(apollo_config_char_spk_channel_value);
             break;

         case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT_VAL :
             puAttribute->p_attr = apollo_config_char_spk_output_port_value;
             puAttribute->attr_len = strlen((char*) apollo_config_char_spk_output_port_value);
             break;

         case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL :
             puAttribute->p_attr = &apollo_config_char_spk_output_vol_value;
             puAttribute->attr_len = sizeof(apollo_config_char_spk_output_vol_value);
             break;

         case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL :
             puAttribute->p_attr = &apollo_config_char_src_input_value;
             puAttribute->attr_len = sizeof(apollo_config_char_src_input_value);
            break;

         case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK_VAL :
             puAttribute->p_attr = apollo_config_char_src_loopback_value;
             puAttribute->attr_len = strlen((char *) apollo_config_char_src_loopback_value);
            break ;

         case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT_VAL :
              puAttribute->p_attr   = apollo_config_char_src_input_port_value;
              puAttribute->attr_len = strlen((char *)apollo_config_char_src_input_port_value);
             break;

          case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL :
              puAttribute->p_attr = &apollo_config_char_src_input_vol_value;
              puAttribute->attr_len = sizeof(apollo_config_char_src_input_vol_value);
             break ;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( apollo_config_state.battery_level++ > 99)
        {
            apollo_config_state.battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n",
                  conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy);

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t write_request_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    attribute_t *puAttribute;
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n",
                        conn_id, p_data->handle,
                        p_data->is_prep,
                        p_data->offset,
                        p_data->val_len );

    puAttribute = (attribute_t *)apollo_config_get_attribute(p_data->handle);


    if(puAttribute)
    {
        if(p_data->offset > puAttribute->attr_len)
            return WICED_BT_GATT_INVALID_OFFSET;

        if( (p_data->val_len + p_data->offset) > puAttribute->attr_len )
            return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch ( p_data->handle )
    {

    case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL\n");
           apollo_config_char_nw_mode_value = *p_attr;
           apollo_gatt_dct.mode= apollo_config_char_nw_mode_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.mode = %d\n ", apollo_gatt_dct.mode);
       }
       break;


    case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL\n");
           apollo_config_char_nw_security_value = *p_attr;
           apollo_gatt_dct.security = apollo_config_char_nw_security_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO,"apollo_gatt_dct.security = %d\n", apollo_gatt_dct.security );
       }
       break;

    case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL\n");
           memset( apollo_gatt_dct.nw_ssid_name, 0, strlen( ( char* ) apollo_config_char_nw_ssid_value ) );
           memcpy( apollo_gatt_dct.nw_ssid_name, p_data->p_val, p_data->val_len );
           memcpy( apollo_config_char_nw_ssid_value, p_data->p_val, sizeof( apollo_config_char_nw_ssid_value ) );
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.nw_ssid_name = %s\n", apollo_gatt_dct.nw_ssid_name);
       }
       break;

    case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL\n");
           memset( apollo_gatt_dct.nw_pass_phrase, 0, strlen( ( char* ) apollo_config_char_nw_passphrase_value ) );
           memcpy( apollo_gatt_dct.nw_pass_phrase, p_data->p_val, p_data->val_len );
           memcpy( apollo_config_char_nw_passphrase_value ,p_data->p_val, sizeof( apollo_config_char_nw_passphrase_value ) );
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.nw_pass_phrase = %s\n", apollo_gatt_dct.nw_pass_phrase);
       }
       break;

    case HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL_VAL:
       {
           apollo_config_char_nw_channel_value = *p_attr;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
       }
       break;

    case HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND_VAL:
       {
           apollo_config_char_nw_band_value = *p_attr;
       }
       break;

    case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL\n");
           memset( apollo_gatt_dct.spk_name, 0, strlen((char*)apollo_config_char_spk_name_value ) );
           memcpy( apollo_gatt_dct.spk_name, p_data->p_val, p_data->val_len );
           memcpy( apollo_config_char_spk_name_value, p_data->p_val, sizeof(apollo_config_char_spk_name_value) );
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.spk_name = %s\n", apollo_gatt_dct.spk_name);
       }
       break;

    case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL\n");
           apollo_config_char_spk_channel_value = *p_attr;
           apollo_gatt_dct.spk_channel_map = apollo_config_char_spk_channel_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.spk_channel_map = %d\n" , apollo_gatt_dct.spk_channel_map);
       }
       break;

    case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT_VAL:
       {
            uint8_t *p_dest;
            p_dest = ( ( uint8_t * ) puAttribute->p_attr ) + p_data->offset;
            memset( p_dest , 0, strlen( ( char* ) apollo_config_char_spk_output_port_value ) );
            memcpy( p_dest, p_data->p_val, p_data->val_len );
       }
       break;

    case HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL\n");
           apollo_config_char_spk_output_vol_value = *p_attr;
           apollo_gatt_dct.spk_vol = apollo_config_char_spk_output_vol_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.spk_vol = %d\n" , apollo_gatt_dct.spk_vol);
       }
       break;

    case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL\n");
           apollo_config_char_src_input_value = *p_attr;
           apollo_gatt_dct.src_type = apollo_config_char_src_input_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.src_type = %d\n", apollo_gatt_dct.src_type);
       }
       break;

    case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK_VAL:
       {
           uint8_t *p_dest;
           p_dest = ((uint8_t *)puAttribute->p_attr) + p_data->offset;
           memset(p_dest ,0 ,strlen((char*)apollo_config_char_src_loopback_value));
           memcpy( p_dest, p_data->p_val, p_data->val_len );
       }
       break;

    case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT_VAL:
       {
            uint8_t *p_dest;
            p_dest = ((uint8_t *)puAttribute->p_attr) + p_data->offset;
            memset( p_dest , 0, strlen( ( char* ) apollo_config_char_src_input_port_value ) );
            memcpy( p_dest, p_data->p_val, p_data->val_len );
       }
       break;

    case HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL:
       {
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "writing HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL\n");
           apollo_config_char_src_input_vol_value = *p_attr;
           apollo_gatt_dct.src_vol = apollo_config_char_src_input_vol_value;
           apollo_config_state.app_dct_dirty = WICED_TRUE;
           wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_gatt_dct.src_vol = %d\n", apollo_gatt_dct.src_vol);
       }
       break;

    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
    return result;
}

/*
 * Write Execute Procedure
 */
static wiced_bt_gatt_status_t write_and_execute_request_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag )
{
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "write exec: flag:%d\n", exec_flag);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
static wiced_bt_gatt_status_t mtu_request_handler( uint16_t conn_id, uint16_t mtu)
{
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}


/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t apollo_config_gatt_server_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config_conn_up  id:%d\n", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    apollo_config_state.conn_id = p_status->conn_id;
    memcpy( apollo_config_state.remote_addr, p_status->bd_addr, sizeof( BD_ADDR ) );

    /* Stop advertising */
    result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Stopping Advertisements %d\n", result);

    memcpy( apollo_config_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    apollo_config_hostinfo.characteristic_client_configuration = 0;
    apollo_config_hostinfo.number_of_blinks                    = 0;

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t apollo_config_gatt_server_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "connection_down  conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason);

    /* Resetting the device info */
    memset( apollo_config_state.remote_addr, 0, 6 );
    apollo_config_state.conn_id = 0;

    if ( apollo_config_state.app_dct_dirty != WICED_FALSE )
    {
        apollo_config_write_dct( &apollo_gatt_dct );
    }

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if ( apollo_config_state.flag_stay_connected )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "connection down , start advs\n");
        apollo_config_set_advertisement_data( advertisement_type_translator( ) );
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "wiced_bt_start_advertisements %d\n", result);
    }
    if ( apollo_config_state.app_dct_dirty != WICED_FALSE )
    {
        apollo_gatt_params.gatt_event_cbf( APOLLO_CONFIG_GATT_EVENT_DCT_WRITE_COMPLETED, &apollo_gatt_dct, apollo_gatt_params.user_context );
    }

    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static wiced_bt_gatt_status_t apollo_config_gatt_server_connection_handler( wiced_bt_gatt_connection_status_t *p_status )
{
    is_connected = p_status->connected;
    if ( p_status->connected )
    {
        return apollo_config_gatt_server_connection_up( p_status );
    }

    return apollo_config_gatt_server_connection_down( p_status );
}

/*
 * Process GATT request from the peer
 */
static wiced_bt_gatt_status_t apollo_config_gatt_server_request_handler( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config_gatt_server_request_handler. conn %d, type %d\n", p_data->conn_id, p_data->request_type);

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = read_request_handler( p_data->conn_id, &(p_data->data.read_req) );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = write_request_handler( p_data->conn_id, &(p_data->data.write_req) );
            break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = write_and_execute_request_handler( p_data->conn_id, p_data->data.exec_write );
            break;

        case GATTS_REQ_TYPE_MTU:
            result = mtu_request_handler( p_data->conn_id, p_data->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Received GATTS_REQ_TYPE_CONF\n");
            result = WICED_BT_GATT_SUCCESS;
            break;

       default:
            break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are omitted.
 */
static wiced_bt_gatt_status_t apollo_config_gatt_server_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_config_gatts_callback ,event = %d\n" , event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = apollo_config_gatt_server_connection_handler( &p_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = apollo_config_gatt_server_request_handler( &p_data->attribute_request );
            break;

        default:
            break;
    }
    return result;
}

wiced_result_t apollo_config_gatt_server_start( apollo_config_gatt_server_params_t *params )
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_management_evt_data_t *bt_evt_data = NULL;

    wiced_action_jump_when_not_true( (params != NULL) && (params->gatt_event_cbf != NULL), _exit, result = WICED_BADARG );

    bt_evt_data = apollo_bt_service_get_management_evt_data();
    wiced_action_jump_when_not_true( bt_evt_data != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "BT GATT server: BT management data pointer is NULL !\n") );
    wiced_action_jump_when_not_true( bt_evt_data->enabled.status == WICED_BT_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "BT GATT server: BT stack is not up !\n") );

    memcpy( &apollo_gatt_params, params, sizeof(apollo_gatt_params) );

    result = apollo_config_advertisement_start();

 _exit:
    return result;
}

wiced_result_t apollo_config_gatt_server_stop( void )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    return result;
}
