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
 * Features demonstrated
 *  - GATT database and Device configuration initialization
 *  - Registration with LE stack for various events
 *  - Sending data to the client
 *  - Processing read/write requests from the client
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer
 * 2. Build and download the application (to the WICED board)
 * 3. On application start the device acts as a GATT server and advertises itself as "WiFiInt"
 * 4. On the iOS device, install the WicedWifiIntroducer App present in "/peerapps/iOS"
 * 5. On the iOS device, connect to the WiFi AP that you want the WICED device to connect to.
 * 6. Open the WiFi Introducer application and you should be greeted with the splash screen.
 * 7. The iOS app will scan until a nearby WICED device is detected.
 *    Bring the WICED device close to the iOS device, enter the WiFi AP passphrase and press connect.
 *    You will be prompted to pair the device. Select "Pair" and a secure LE connection will be made
 *    between the iOS device and the WICED device using the ECDH algorithm to generate and exchange the secure keys.
 * 8. The WiFi network credentials will be sent to the WICED device.
 *    The connection result will be notified to the iOS app.
 * 9. Upon successful connection, the network credentials will be saved to the DCT.
 * 10.For more information on how to use iOS app, refer "<WICED_SDK>/doc/WICED-BLE-WiFi-Introducer.pdf"
 */

#include <string.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "ble_wifi_introducer.h"
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "command_console.h"
#include "button_manager.h"
#include "wiced_utilities.h"
#include "wifi/command_console_wifi.h"

#ifdef WIFI_CONFIG_APPLICATION_DEFINED
#include "wifi_config_dct.h"
#else/* #ifdef WIFI_CONFIG_APPLICATION_DEFINED */
#include "default_wifi_config_dct.h"
#endif /* #ifdef WIFI_CONFIG_APPLICATION_DEFINED */

/******************************************************************************
 *                                Constants
******************************************************************************/

#define WIFI_INTRODUCER_CONSOLE_COMMAND_HISTORY_LENGTH  (10)
#define MAX_WIFI_INTRODUCER_COMMAND_LENGTH              (85)

#define GATT_ATTRIBUTE_SIZE                 WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE

#define BUTTON_WORKER_STACK_SIZE              ( 4096 )
#define BUTTON_WORKER_QUEUE_SIZE              ( 4 )

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    /* Launch BTLE GATT configuration process */
    ACTION_CONFIG_GATT_LAUNCH
} app_action_t;

typedef enum
{
    CONFIG_GATT_BUTTON
} application_button_t;

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

static void                     wifi_introducer_gatt_server_init                                ( void );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_connection_status_handler           ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_connection_up                       ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_connection_down                     ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t           wifi_introducer_bt_management_callback                          ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_callback                            ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_read_request_handler                ( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_write_request_handler               ( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                     wifi_introducer_set_advertisement_data                          ( void );
static void                     wifi_introducer_send_message                                    ( void );
static wiced_bt_gatt_status_t   wifi_introducer_gatt_server_write_and_execute_request_handler   ( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag );
static void                     app_button_event_handler                                        ( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state );

wiced_result_t                  wifi_introducer_load_key_to_addr_resolution_db                  ( void );
static void                     wifi_join_thread(wiced_thread_arg_t arg);
static void                     create_wifi_join_thread(void);
void                            wifi_introducer_save_link_key                                   ( wiced_bt_device_link_keys_t  paired_device_keys );
void                            wifi_introducer_read_link_key                                   ( wiced_bt_device_link_keys_t* paired_device_keys );

#ifdef WICED_SET_BT_PAIRABLE_MODE
void wiced_bt_set_pairable_mode(uint8_t allow_pairing, uint8_t connect_only_paired);
#endif
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
} wifi_introducer_state_t;

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

static uint8_t wifi_introducer_device_name[ ]                                 = "WiFiInt";
static uint8_t wifi_introducer_appearance_name[2]                             = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
static uint8_t wifi_introducer_char_nw_security_value                         = WICED_SECURITY_OPEN;
static uint8_t wifi_introducer_char_nw_ssid_value[GATT_ATTRIBUTE_SIZE]        = CLIENT_AP_SSID;
static uint8_t wifi_introducer_char_nw_passphrase_value[GATT_ATTRIBUTE_SIZE]  = CLIENT_AP_PASSPHRASE;
static uint8_t wifi_introducer_char_notify_value                              = 0; // 0 means not configured , 1 means configured


static wifi_introducer_state_t  wifi_introducer_state;
static host_info_t              wifi_introducer_hostinfo;
static wiced_bool_t             is_connected = FALSE;
static                           wiced_thread_t join_handler;

attribute_t gatt_user_attributes[] =
{
        { HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,        sizeof( wifi_introducer_device_name )                , wifi_introducer_device_name },
        { HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,  sizeof( wifi_introducer_appearance_name )            , wifi_introducer_appearance_name },
        { HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY_VAL,         WICED_SECURITY_OPEN                                  , &wifi_introducer_char_nw_security_value},
        { HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY_VAL,           sizeof( wifi_introducer_char_notify_value )          , &wifi_introducer_char_notify_value },
        { HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_CFG_DESC,             2                                                    , (void*)&wifi_introducer_hostinfo.characteristic_client_configuration },
        { HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID_VAL,             WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE                  , wifi_introducer_char_nw_ssid_value },
        { HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE_VAL,       WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE                  , wifi_introducer_char_nw_passphrase_value },

};
static wiced_semaphore_t                join_semaphore;

static char                             wifi_introducer_command_buffer[MAX_WIFI_INTRODUCER_COMMAND_LENGTH];
static char                             wifi_introducer_command_history_buffer[MAX_WIFI_INTRODUCER_COMMAND_LENGTH * WIFI_INTRODUCER_CONSOLE_COMMAND_HISTORY_LENGTH];

extern const wiced_bt_cfg_settings_t    wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t    wiced_bt_cfg_buf_pools[];

button_manager_t                        button_manager;
wiced_worker_thread_t                   button_worker_thread;
wiced_bool_t                            button_gatt_launch_was_pressed;

static const wiced_button_manager_configuration_t button_manager_configuration =
{
    .short_hold_duration     = 500  * MILLISECONDS,
    .debounce_duration       = 150  * MILLISECONDS,

    .event_handler           = app_button_event_handler,
};

/* Static button configuration */
static const wiced_button_configuration_t button_configurations[] =
{
#if (WICED_PLATFORM_BUTTON_COUNT > 0)
    [ CONFIG_GATT_BUTTON ] = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT , ACTION_CONFIG_GATT_LAUNCH },
#endif
};

/* Button objects for the button manager */
static button_manager_button_t buttons[] =
{
#if (WICED_PLATFORM_BUTTON_COUNT > 0)
    [ CONFIG_GATT_BUTTON ] = { &button_configurations[ CONFIG_GATT_BUTTON ] },
#endif
};

//static wiced_semaphore_t    scan_complete_semaphore;
//static wiced_security_t     auth_type;
static wiced_bool_t         wifi_introducer_ssid_name       = FALSE;
static wiced_bool_t         wifi_introducer_ssid_password   = FALSE;

wiced_bt_device_link_keys_t             wifi_introducer_device_link_keys;
/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the WiFi Introducer Sensor application.  It defines
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
static const uint8_t wifi_introducer_gatt_server_database[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_WIFI_INTRO_SENS_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
       characteristics of GAP service */
    PRIMARY_SERVICE_UUID16( HANDLE_WIFI_INTRO_SENS_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare proprietary WiFi Introducer Config Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_WIFI_INTRO_SENS_NW_SERVICE, UUID_WIFI_INTRODUCER_SERVICE ),

        /* Declare characteristic used to notify/indicate change */
        CHARACTERISTIC_UUID128( HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY, HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY_VAL,
         UUID_WIFI_INTRODUCER_CHARACTERISTIC_NOTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE ),

         /* Declare client characteristic configuration descriptor
          * Value of the descriptor can be modified by the client
          * Value modified shall be retained during connection and across connection
          * for bonded devices.  Setting value to 1 tells this application to send notification
          * when value of the characteristic changes.  Value 2 is to allow indications. */
         CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_CFG_DESC, GATT_UUID_CHAR_CLIENT_CONFIG,
         LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* Declare characteristic for Security  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY, HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY_VAL,
            UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_SECURITY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
            | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE ),
            //| LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE ),

        /* Declare characteristic for  SSID  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID, HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID_VAL,
            UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_SSID, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
            | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE ),

        /* Declare characteristic for Passphrase  */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE, HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE_VAL,
            UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_PASSPHRASE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
            | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE ),


    /* Declare Device Info service */
    PRIMARY_SERVICE_UUID16( HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state )
{
    wiced_result_t result = WICED_SUCCESS;
    platform_dct_wifi_config_t* dct_tables = NULL;

    if ( ( button_gatt_launch_was_pressed != WICED_TRUE ) &&
         ( button->configuration->application_event == ACTION_CONFIG_GATT_LAUNCH ) &&
         ( event == BUTTON_CLICK_EVENT ) )
    {
        button_gatt_launch_was_pressed = WICED_TRUE;

        WPRINT_APP_INFO(("Back button is pressed!\r\n"));

        /* get the wi-fi config section for modifying, any memory allocation required would be done inside wiced_dct_read_lock() */
        wiced_dct_read_lock( (void**) &dct_tables, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_tables ) );

        if (dct_tables != NULL)
        {
            memset((char *)dct_tables->stored_ap_list[0].details.SSID.value, 0,
                    sizeof(dct_tables->stored_ap_list[0].details.SSID.value));
            dct_tables->stored_ap_list[0].details.SSID.length = strlen( CLIENT_AP_SSID );
            strlcpy((char *)dct_tables->stored_ap_list[0].details.SSID.value,
                    CLIENT_AP_SSID,
                    sizeof(dct_tables->stored_ap_list[0].details.SSID.value));

            memset((char *)dct_tables->stored_ap_list[0].security_key, 0,
                    sizeof(dct_tables->stored_ap_list[0].security_key));
            dct_tables->stored_ap_list[0].security_key_length = strlen( CLIENT_AP_PASSPHRASE );
            strlcpy((char *)dct_tables->stored_ap_list[0].security_key,
                    CLIENT_AP_PASSPHRASE,
                    sizeof(dct_tables->stored_ap_list[0].security_key));

            dct_tables->stored_ap_list[0].details.security = CLIENT_AP_SECURITY;

            result |= wiced_dct_write( (const void*) dct_tables, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

            result = wiced_dct_read_unlock(dct_tables, WICED_TRUE);

            if (result == WICED_SUCCESS)
            {
                dct_tables = NULL;
                wiced_deinit();
                wiced_rtos_delay_milliseconds(500);
                wiced_framework_reboot();
            }
            else
            {
                WPRINT_APP_INFO(("Can't free/release WiFi configuration !\r\n"));
            }
        }
    }

    return;
}

static wiced_result_t wifi_introducer_button_handler_init( )
{
    wiced_result_t result;

    result = wiced_rtos_create_worker_thread( &button_worker_thread, WICED_DEFAULT_WORKER_PRIORITY, BUTTON_WORKER_STACK_SIZE, BUTTON_WORKER_QUEUE_SIZE );
    result = button_manager_init( &button_manager, &button_manager_configuration, &button_worker_thread, buttons, ARRAY_SIZE( buttons ) );

    return result;
}

/*
static void wifi_introducer_button_handler_deinit()
{
    button_manager_deinit( &button_manager );
    wiced_rtos_delete_worker_thread( &button_worker_thread );
}
*/

void application_start( void )
{
    wiced_init();
    wiced_rtos_init_semaphore(&join_semaphore);

    wifi_introducer_button_handler_init();

    WPRINT_APP_INFO( ( "WiFi Introducer Sensor Start\n" ) );

    /* Join with new credentials */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    if ( wiced_network_is_up( WICED_STA_INTERFACE ) == WICED_FALSE )
    {
        /* Register call back and configuration with stack */
        wiced_bt_stack_init( wifi_introducer_bt_management_callback ,
                            &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );

        create_wifi_join_thread();
    }
}


/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void wifi_introducer_gatt_server_init( void )
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    WPRINT_APP_INFO( ( "wifi_introducer_gatt_server_init\n" ) );

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( wifi_introducer_gatt_server_callback );

    WPRINT_APP_INFO(( "\n wiced_bt_gatt_register: %d\n", gatt_status ));

    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init( wifi_introducer_gatt_server_database, sizeof( wifi_introducer_gatt_server_database ) );

    WPRINT_APP_INFO( ( "wiced_bt_gatt_db_init %d\n", gatt_status ) );
#ifdef WICED_SET_BT_PAIRABLE_MODE
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
#endif
    /* Set the advertising parameters and make the device discoverable */
    wifi_introducer_set_advertisement_data();

#ifndef USE_WICED_HCI
    /* Load the address resolution DB with the keys stored in the DCT */
    result = wifi_introducer_load_key_to_addr_resolution_db();
    WPRINT_APP_INFO(("\n wiced_bt_dev_add_device_to_address_resolution_db . result = %d \n" , result));
#endif
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    WPRINT_APP_INFO( ( "wiced_bt_start_advertisements %d\n", result ) );

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    wifi_introducer_state.flag_stay_connected = 1;
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void wifi_introducer_set_advertisement_data(void)
{
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;
    uint8_t wifi_introducer_service_uuid[LEN_UUID_128]    = { UUID_WIFI_INTRODUCER_SERVICE };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t *)wiced_bt_cfg_settings.device_name;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = wifi_introducer_service_uuid;

    num_elem ++;

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    WPRINT_APP_INFO( ( "wiced_bt_ble_set_advertisement_data %d\n", result ) );
}

/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
void wifi_introducer_advertisement_stopped( void )
{
    wiced_result_t result;

    if ( wifi_introducer_state.flag_stay_connected && !wifi_introducer_state.conn_id )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WPRINT_APP_INFO( ( "wiced_bt_start_advertisements: %d\n", result ) );
    }
    else
    {
        WPRINT_APP_INFO( ( "ADV stop\n") );
    }
}

void wifi_introducer_save_link_key( wiced_bt_device_link_keys_t paired_device_keys )
{
    WPRINT_BT_APP_INFO( ("[WiFi Introducer] wifi_introducer_save_link_key\n"));
    memcpy(&wifi_introducer_device_link_keys, &paired_device_keys, sizeof(wiced_bt_device_link_keys_t));
    wiced_dct_write( &wifi_introducer_device_link_keys, DCT_APP_SECTION, 0, sizeof( wifi_introducer_device_link_keys ) );
}

void wifi_introducer_read_link_key( wiced_bt_device_link_keys_t* paired_device_keys )
{
    WPRINT_APP_INFO( ("[WiFi Introducer] wifi_introducer_read_link_keys\n"));
    memset( &paired_device_keys->key_data, 0x00, sizeof( paired_device_keys->key_data) );
    memcpy(&paired_device_keys->key_data, &wifi_introducer_device_link_keys.key_data, sizeof( paired_device_keys->key_data ));
}

/* NOTE: This is handled in the 43012 Embedded App for now.
 * TODO: Need to get the link keys stored in the DCT for Embedded Mode.
 */
#ifndef USE_WICED_HCI
wiced_result_t wifi_introducer_load_key_to_addr_resolution_db( void )
{
    WPRINT_APP_INFO( ("[WiFi Introducer] wifi_introducer_load_key_to_addr_resolution_db\n"));
    //read DCT
    WPRINT_APP_INFO(("\n [WiFi Introducer] reading DCT \n"));
    wiced_bt_device_link_keys_t *dct;
    memset(&wifi_introducer_device_link_keys, 0x00 , sizeof(wiced_bt_device_link_keys_t));

    wiced_dct_read_lock( (void**) &dct, TRUE, DCT_APP_SECTION, 0, sizeof( *dct ) );
    memcpy(&wifi_introducer_device_link_keys, dct, sizeof(wiced_bt_device_link_keys_t));
    wiced_dct_read_unlock( dct, WICED_FALSE );

    return wiced_bt_dev_add_device_to_address_resolution_db(&wifi_introducer_device_link_keys);
}
#endif

static void wifi_join_thread(wiced_thread_arg_t arg)
{
    wiced_scan_result_t     ap_info;

    do
    {
        /* Wait for SSID & pass pharse to be configured */
        wiced_rtos_get_semaphore( &join_semaphore, NEVER_TIMEOUT);

        if ( ( wwd_wifi_is_ready_to_transceive( 0 ) == WWD_SUCCESS ) )
        {
           wiced_network_down( WICED_STA_INTERFACE );
        }
        if ( wiced_wifi_find_ap( (char*)wifi_introducer_char_nw_ssid_value, &ap_info, NULL ) == WICED_SUCCESS )
        {
           print_scan_result( &ap_info );
        }

        int result_wifi_join = wifi_join( (char*)wifi_introducer_char_nw_ssid_value , strlen( (char*)wifi_introducer_char_nw_ssid_value), ap_info.security,
                                               wifi_introducer_char_nw_passphrase_value, strlen( (char*)wifi_introducer_char_nw_passphrase_value ), NULL, NULL, NULL );
        if (0 != result_wifi_join)
        {
             WPRINT_APP_INFO(("Join Failed!!!\n"));
             //writing BLE notification
             wifi_introducer_char_notify_value = 0;
             wifi_introducer_ssid_name = FALSE;
             wifi_introducer_ssid_password = FALSE;
        }
        else
        {
            WPRINT_APP_INFO(("Join Succeeded!!!\n"));
            //writing BLE notification
            wifi_introducer_char_notify_value = 1;
        }
        wifi_introducer_send_message();
        wifi_introducer_ssid_name = FALSE;
        wifi_introducer_ssid_password = FALSE;

    } while (1);
}

static void create_wifi_join_thread(void)
{
    wiced_result_t         result;

    result = wiced_rtos_create_thread(&join_handler, WICED_DEFAULT_WORKER_PRIORITY, "WWD Join Thrd", wifi_join_thread, WICED_DEFAULT_APPLICATION_STACK_SIZE, NULL);
    if (result)
    {
        WPRINT_APP_INFO(("%s: thread creation failed !!!!!!!!!\n",__func__ ));
        return;
    }

 }
/*
 * wifi_introducer bt/ble link management callback
 */
static wiced_result_t wifi_introducer_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;

    WPRINT_APP_INFO(("\n wifi_introducer_bt_management_callback: %x\n", event ));

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:

        //result = wiced_bt_dev_write_local_addr ( local_address );
        //WPRINT_APP_INFO( ("wiced_bt_dev_write_local_addr result = 0x%x\n", (unsigned int)result) );

        result = command_console_init(STDIO_UART, sizeof(wifi_introducer_command_buffer), wifi_introducer_command_buffer,
                                  WIFI_INTRODUCER_CONSOLE_COMMAND_HISTORY_LENGTH, wifi_introducer_command_history_buffer, " ");

        if (result != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(("Error starting the command console\r\n"));
        }

        wifi_introducer_gatt_server_init();

        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;         /**< No Input, No Output */
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;                     /**< No OOB data */
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;     /**< LE Secure Connection, MITM, Bonding */
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID; /**< encryption information of peer device */ /**< identity key of the peer device */
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        WPRINT_APP_INFO(( "[WiFi Introducer] Pairing IO capabitlies request event\n"));
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WPRINT_APP_INFO(( "[WiFi Introducer] Pairing Complete: %d\n",p_info->reason));
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
    {
        WPRINT_APP_INFO(("[WiFi Introducer] Paired Device Link Keys Update Event\n"));
        WPRINT_APP_INFO( ("GATT connection to [%02X:%02X:%02X:%02X:%02X:%02X] \n",
                              p_event_data->paired_device_link_keys_request.bd_addr[0],
                              p_event_data->paired_device_link_keys_request.bd_addr[1],
                              p_event_data->paired_device_link_keys_request.bd_addr[2],
                              p_event_data->paired_device_link_keys_request.bd_addr[3],
                              p_event_data->paired_device_link_keys_request.bd_addr[4],
                              p_event_data->paired_device_link_keys_request.bd_addr[5]) );

        wifi_introducer_save_link_key( p_event_data->paired_device_link_keys_request );
        break;
    }

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
    {
        WPRINT_APP_INFO( ("[WiFi Introducer] Paired Device Link Keys Request Event\n") );
        wifi_introducer_read_link_key(&p_event_data->paired_device_link_keys_request);
        WPRINT_APP_INFO(( "Key retrieval success\n" ));

        break;
    }

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WPRINT_APP_INFO(( "Advertisement State Change: %d\n", *p_mode));
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
            wifi_introducer_advertisement_stopped();
        }
        break;

    case BTM_PASSKEY_REQUEST_EVT:
    case BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT:
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
    case BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT:
    case BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT:
    default:
        WPRINT_LIB_INFO(("[WiFi Introducer] Unhandled Bluetooth Stack Callback event :%d\n", event));
        break;
    }

    return result;
}

/*
 * Check if client has registered for notification/indication
 * and send message if appropriate
 */

static void wifi_introducer_send_message( void )
{
    WPRINT_APP_INFO( ( "\n %s: Client's Characteristic configuration:%d\n", __func__, wifi_introducer_hostinfo.characteristic_client_configuration ) );
    /* If client has not registered for indication or notification, no action */
    if ( wifi_introducer_hostinfo.characteristic_client_configuration == 0 )
    {
        return;
    }
    else if ( wifi_introducer_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        uint8_t *p_attr = &wifi_introducer_char_notify_value;
        WPRINT_APP_INFO(("wifi_introducer_char_notify_value = %d \n", wifi_introducer_char_notify_value));
        wiced_bt_gatt_send_notification( wifi_introducer_state.conn_id, HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY_VAL, sizeof(wifi_introducer_char_notify_value), p_attr );
    }
}

/*
 * Find attribute description by handle
 */
static attribute_t * wifi_introducer_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gatt_user_attributes ) / sizeof( gatt_user_attributes[0] ); i++ )
    {
        if ( gatt_user_attributes[i].handle == handle )
        {
            return ( &gatt_user_attributes[i] );
        }
    }
    WPRINT_APP_INFO(( "attribute not found:%x\n", handle ));
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_read_request_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = wifi_introducer_get_attribute(p_read_data->handle) ) == NULL)
    {
        WPRINT_APP_INFO(("read_hndlr attr not found hdl:%x\n", p_read_data->handle ));
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch(p_read_data->handle){

        case HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME_VAL:
            puAttribute->p_attr = (void *) wiced_bt_cfg_settings.device_name;
            puAttribute->attr_len  = strlen((char*)wiced_bt_cfg_settings.device_name);
            break ;

        case HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL:
            puAttribute->p_attr = wifi_introducer_appearance_name;
            puAttribute->attr_len    = strlen((char*)wifi_introducer_appearance_name);
            break;

        case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY_VAL :
            puAttribute->p_attr = &wifi_introducer_char_nw_security_value;
            puAttribute->attr_len = sizeof(wifi_introducer_char_nw_security_value);
            break;

        case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID_VAL:
            puAttribute->p_attr = wifi_introducer_char_nw_ssid_value;
            break;

        case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE_VAL :
            puAttribute->p_attr = wifi_introducer_char_nw_passphrase_value;
            break;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( wifi_introducer_state.battery_level++ > 99)
        {
            wifi_introducer_state.battery_level = 0;
        }
    }

    if( p_read_data->handle == HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        //puAttribute->p_attr = &wifi_introducer_write;
        //puAttribute->attr_len = sizeof(wifi_introducer_write);
    }
    attr_len_to_copy = puAttribute->attr_len;

    WPRINT_APP_INFO( ( "read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy ) );

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

/*
 * Process write request or write command from peer device
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_write_request_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    attribute_t             *puAttribute;
    wiced_bt_gatt_status_t  result    = WICED_BT_GATT_SUCCESS;
    uint8_t                 *p_attr   = p_data->p_val;

    WPRINT_APP_INFO( ( "write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len ) );

    puAttribute = (attribute_t *)wifi_introducer_get_attribute( p_data->handle );

    if( puAttribute )
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
    case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY_VAL:

        WPRINT_APP_INFO(( "wifi_introducer_gatt_server_write_request_handler:security value\n" ));

        break;
    case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID_VAL:

        wifi_introducer_ssid_name = TRUE;

        memset(wifi_introducer_char_nw_ssid_value, 0, sizeof(wifi_introducer_char_nw_ssid_value));
        memcpy( wifi_introducer_char_nw_ssid_value, p_data->p_val, p_data->val_len );

        WPRINT_APP_INFO(( "wifi_introducer_gatt_server_write_request_handler:ssid value: %s\n", wifi_introducer_char_nw_ssid_value ));
        break;

    case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE_VAL:

        wifi_introducer_ssid_password = TRUE;

        memset(wifi_introducer_char_nw_passphrase_value, 0, sizeof(wifi_introducer_char_nw_passphrase_value));
        memcpy( wifi_introducer_char_nw_passphrase_value ,p_data->p_val, p_data->val_len );

        WPRINT_APP_INFO(( "wifi_introducer_gatt_server_write_request_handler:wifi_introducer_char_nw_passphrase_value value: %s\n", wifi_introducer_char_nw_passphrase_value ));
        break;

    case HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_CFG_DESC:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        wifi_introducer_hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
        WPRINT_APP_INFO(("\n wifi_introducer_hostinfo.characteristic_client_configuration = %d \n" , wifi_introducer_hostinfo.characteristic_client_configuration));
        break;

    default:
        WPRINT_APP_INFO(( "wifi_introducer_gatt_server_write_request_handler:default value\n" ));
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    if ( ( wifi_introducer_ssid_name == TRUE ) && ( wifi_introducer_ssid_password == TRUE ) )
    {
        WPRINT_APP_INFO(("SSID configured \n"));
        wiced_rtos_set_semaphore( &join_semaphore );
    }
    return result;
}

/*
 * Write Execute Procedure
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_write_and_execute_request_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag )
{
    WPRINT_APP_INFO(("write exec: flag:%d\n", exec_flag));
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_mtu_request_handler( uint16_t conn_id, uint16_t mtu)
{
    WPRINT_APP_INFO(("req_mtu: %d\n", mtu));
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t wifi_introducer_gatt_server_confirmation_handler( uint16_t conn_id, uint16_t handle )
{
    WPRINT_APP_INFO(( "wifi_introducer_indication_confirmation, conn %d hdl %d\n", conn_id, handle ));

    if ( !wifi_introducer_state.flag_indication_sent )
    {
        WPRINT_APP_INFO(("WiFi Introducer: Wrong Confirmation!"));
        return WICED_BT_GATT_SUCCESS;
    }
    wifi_introducer_state.flag_indication_sent = 0;
    wifi_introducer_send_message();

    return WICED_BT_GATT_SUCCESS;
}

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WPRINT_APP_INFO( ( "wifi_introducer_conn_up  id:%d\n", p_status->conn_id) );

    WPRINT_APP_INFO( ("wifi_introducer_conn_up connected to [%02X:%02X:%02X:%02X:%02X:%02X] \n",
                          p_status->bd_addr[0],
                          p_status->bd_addr[1],
                          p_status->bd_addr[2],
                          p_status->bd_addr[3],
                          p_status->bd_addr[4],
                          p_status->bd_addr[5]) );


    /* Update the connection handler.  Save address of the connected device. */
    wifi_introducer_state.conn_id = p_status->conn_id;
    memcpy(wifi_introducer_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    WPRINT_APP_INFO( ( "Stopping Advertisements%d\n", result ) );

#if 0

    // Initiating the security. If call to the Bond returns success, device is bonded, and we just need to setup encryption
    if( ( result = wiced_bt_dev_sec_bond( p_status->bd_addr, p_status->addr_type, BT_TRANSPORT_LE, 0, NULL ) ) == WICED_BT_SUCCESS )
    {
        WPRINT_APP_INFO(( "wifi_introducer starting encryption\n" ));
        wiced_bt_dev_set_encryption( p_status->bd_addr, BT_TRANSPORT_LE, NULL );

    }
    else
    {
        //WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", result );
        /* Updating the bd address in the  host info in NVRAM  */
        memcpy( wifi_introducer_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    }

#endif

    memcpy( wifi_introducer_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    wifi_introducer_hostinfo.characteristic_client_configuration = 0;
    wifi_introducer_hostinfo.number_of_blinks                    = 0;

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WPRINT_APP_INFO( ( "connection_down  conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason ) );


    /* Resetting the device info */
    memset( wifi_introducer_state.remote_addr, 0, 6 );
    wifi_introducer_state.conn_id = 0;

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if ( wifi_introducer_state.flag_stay_connected )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WPRINT_APP_INFO( ( "wiced_bt_start_advertisements %d\n", result ) );
    }
    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status )
{
    is_connected = p_status->connected;
    if ( p_status->connected )
    {
        return wifi_introducer_gatt_server_connection_up( p_status );
    }

    return wifi_introducer_gatt_server_connection_down( p_status );
}

/*
 * Process GATT request from the peer
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_request_handler( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WPRINT_APP_INFO(( "wifi_introducer_gatt_server_request_handler. conn %d, type %d\n", p_data->conn_id, p_data->request_type ));

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = wifi_introducer_gatt_server_read_request_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = wifi_introducer_gatt_server_write_request_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = wifi_introducer_gatt_server_write_and_execute_request_handler( p_data->conn_id, p_data->data.exec_write );
        break;

    case GATTS_REQ_TYPE_MTU:
        result = wifi_introducer_gatt_server_mtu_request_handler( p_data->conn_id, p_data->data.mtu );
        break;

    case GATTS_REQ_TYPE_CONF:
        result = wifi_introducer_gatt_server_confirmation_handler( p_data->conn_id, p_data->data.handle );
        break;

   default:
        break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are ommitted.
 */
static wiced_bt_gatt_status_t wifi_introducer_gatt_server_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = wifi_introducer_gatt_server_connection_status_handler( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = wifi_introducer_gatt_server_request_handler( &p_data->attribute_request );
        break;

    default:
        break;
    }
    return result;
}
