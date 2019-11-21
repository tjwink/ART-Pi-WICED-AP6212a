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
 * BLE MESH Demo Application
 * The Application enables BIG as a Mesh Node
 * When the application comes up BIG will be ready to be provisioned.The WICED app connects to configured MQTT broker (AWS/Bluemix) or REST
 * 1> On Boot up, if MQTT broker is defined the WICED application subscribes to WICED_TOPIC_SUBSCRIBE. If REST is defined, application will start HTTP server
 * 2> Provision BIG and other mesh tags using  Mesh Android Test App/ Lighting App (Apks are located in peer app)
 * 3> The Android App can be used to control the mesh Tags through BIG using the appropriate MQTT broker / REST
 *
 *  To run the app, work through the following steps.
 *  Modify Wi-Fi configuration settings CLIENT_AP_SSID and CLIENT_AP_PASSPHRASE in wifi_config_dct.h to match your router settings.
 *
 *  REST SETUP
 *  1. Define CLOUD_CONFIG as WICED_REST in cloud_config.h
 *  2. Make sure Android phone and BIG are connected to same Wi-Fi
 *
 *  BLUEMIX SETUP
 *  1. Define CLOUD_CONFIG as WICED_BLUEMIX_BROKER in cloud_config.h
 *  2. Update the <org> in BLUEMIX MQTT broker address (MQTT_BROKER_ADDRESS), CLIENT_ID in cloud_config.h.
 *  3. Update CLIENT_ID, WICED_TOPIC_PUBLISH, WICED_TOPIC_SUBSCRIBE, PASSWORD if required.
 *
 *  AWS SETUP
 *  1. Define CLOUD_CONFIG as WICED_AWS_BROKER in cloud_config.h
 *  2. Update the AWS MQTT broker address (MQTT_BROKER_ADDRESS) if needed.
 *  3. Make sure AWS Root Certificate 'resources/apps/aws/iot/rootca.cer' is up to date while building the app.
 *  4. Copy client certificate and private key for the given AWS IOT user in resources/apps/aws/iot/mesh folder.
 *     Ensure that valid client certificates and private keys are provided for the AWS IOT user in resources/apps/aws/iot/mesh folder.
 *  5. Update the WICED_TOPIC_PUBLISH, WICED_TOPIC_SUBSCRIBE if required
 *
 *  Build and run this application.
 */

#include <string.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "mesh.h"
#include "cloud_config.h"
#include "resources.h"
#include "string.h"
#include "JSON.h"
#include "cloud_config.h"
#include "http_server.h"
#include "mesh_uri.h"
#include "wiced_hci_bt_mesh.h"
#include "mesh_dct.h"

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];
/******************************************************************************
 *                                Constants
******************************************************************************/

#define MQTT_PUBLISH_RETRY_COUNT            (3)

#define WICED_MQTT_TIMEOUT                  (5000)

#define WICED_MQTT_DELAY_IN_MILLISECONDS    (1000)

#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)

#define BIG_HTTP_SERVER_STACK_SIZE          ( 5000 )

#define WICED_APP_REBOOT_DELAY_MSECS        ( 500 )


/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
extern mesh_value_handle_t* create_mesh_value( const char* payload );
extern mesh_value_handle_t* reverse_mesh_byte_stream( mesh_value_handle_t* val );
int32_t process_send_mesh_pkt_rcvd( struct wiced_big_mqtt *big_mqtt , char *msg );
static wiced_result_t mesh_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_result_t mesh_write_dct( mesh_dct_t* dct_to_write, int id, uint8_t *packet, uint32_t packet_len );
static wiced_result_t mesh_read_dct();
void print_mesh_dct_info();
// TO DO : This logic needs to changed
void wiced_app_system_reboot(void);
extern void mesh_init_mqtt_transport( wiced_thread_arg_t arg );
wiced_semaphore_t big_mesh_init_semaphore;
wiced_thread_t mqtt_thread;
/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
mesh_dct_t mesh_dct_info;
static wiced_http_server_t http_server;

/* The order of the paths matters because of the use of wildcard character '*' */
static START_OF_HTTP_PAGE_DATABASE( web_pages )
MESH_URIS
END_OF_HTTP_PAGE_DATABASE();

wiced_result_t mesh_init_rest_transport( void )
{
    /* Start HTTP server */
    return wiced_http_server_start( &http_server, 80, 5, web_pages, WICED_STA_INTERFACE, BIG_HTTP_SERVER_STACK_SIZE );

}

wiced_result_t mesh_deinit_rest_transport( void )
{
    /* De-Init HTTP server */
    return wiced_http_server_stop( &http_server );
}


// Callback function which recieves proxy packet from the mesh core , this data should be published to cloud
static void proxy_send_cb(const uint8_t *packet, uint32_t packet_len)
{
    WPRINT_APP_INFO(("proxy_gatt_send_cb"));
}

static void mesh_write_nvram_data_cb(int id, uint8_t *packet, uint32_t packet_len )
{

    WPRINT_APP_INFO(("mesh_write_node_info , packet_len = %lu \n" , packet_len));
    mesh_write_dct(&mesh_dct_info, id, packet, packet_len);
}

// TO DO once provisioned 20706 needs to be rebooted so that proxy service is added
// current approach is to re-download the 20706 s/w which inturn reboots 20706
// restore the provisioning data throught NVM chunks
void wiced_app_system_reboot(void)
{
    wiced_deinit( );
    wiced_rtos_delay_milliseconds( WICED_APP_REBOOT_DELAY_MSECS );
    wiced_framework_reboot( );
}


// Callback function which notifies provisioning status.
void mesh_provision_end(uint32_t conn_id, uint8_t result)
{
    WPRINT_APP_INFO(("mesh_provision_end , status = %02X\n" , result));
    if(result == MESH_PROVISION_RESULT_SUCCESS)
    {
        mesh_dct_info.node_authenticated = MESH_NODE_PROVISIONED;
        wiced_dct_write( &mesh_dct_info, DCT_APP_SECTION, 0, sizeof(mesh_dct_t) );
        //wiced_app_system_reboot();
#if (CLOUD_CONFIG == WICED_REST)
                mesh_init_rest_transport();
#else
                // intializes MQTT transport to use with AWS or Bluemix
               // mesh_init_mqtt_transport( );
                wiced_rtos_delay_milliseconds( 100 );
                wiced_rtos_set_semaphore(&big_mesh_init_semaphore);
#endif
    }

}

// Callback function which notifies mesh status
void mesh_status_cb(uint8_t status)
{
    WPRINT_APP_INFO(("mesh status , status = %02X\n" , status));
    if(mesh_dct_info.node_authenticated == MESH_NODE_UNPROVISIONED)
    {
        WPRINT_APP_INFO(("Mesh Gateway Ready to be Provisioned \n"));
    }
    else
    {

        WPRINT_APP_INFO(("Mesh Gateway is already Provisioned and Mesh is started \n"));
    }
}


void application_start( void )
{

    wiced_init( );
    WPRINT_APP_INFO( ( "application_start ..\n" ) );

    wiced_init( );

    WPRINT_APP_INFO( ( "wiced_init is completed ..\n" ) );

    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
#if (CLOUD_CONFIG == WICED_AWS_BROKER)
    wiced_rtos_init_semaphore(&big_mesh_init_semaphore);
#endif

    wiced_bt_stack_init( mesh_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
#if (CLOUD_CONFIG == WICED_AWS_BROKER)
    wiced_rtos_get_semaphore(&big_mesh_init_semaphore , WICED_NEVER_TIMEOUT);
    wiced_rtos_create_thread(&mqtt_thread, WICED_DEFAULT_LIBRARY_PRIORITY, "MQTT-Thread", mesh_init_mqtt_transport, WICED_DEFAULT_APPLICATION_STACK_SIZE, NULL);
#endif

}

int32_t process_send_mesh_pkt_rcvd( struct wiced_big_mqtt *big_mqtt , char *msg )
{
    mesh_value_handle_t* value = create_mesh_value( msg );
    mesh_value_handle_t* reversed_value = reverse_mesh_byte_stream( value );
    WPRINT_APP_INFO(("process_send_mesh_pkt_rcvd len:, %lu\n" , reversed_value->length));
    wiced_bt_mesh_send_proxy_packet(reversed_value->value,reversed_value->length);
    free( value );
    free( reversed_value );
    return WICED_SUCCESS;
}


static wiced_result_t mesh_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    uint8_t i = 0;

    WPRINT_BT_APP_INFO(("mesh_management_cback: %x\n", event ));

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
            WPRINT_APP_INFO( ("mesh_management_cback: BTM_ENABLED_EVT \n") );
        mesh_read_dct();
        if(mesh_dct_info.node_authenticated == MESH_NODE_PROVISIONED)
        {
                WPRINT_APP_INFO( ("Device is already provisioned push NVM chunks " ) );
            for( i = 0; i < MESH_NV_DATA_MAX_ENTRIES ; i++){
                //TO DO : push nvm chunks with a delay as 20706 goes to bad state if data is pushed faster
                wiced_rtos_delay_milliseconds( 100 );
                    WPRINT_APP_INFO(("wiced_bt_mesh_push_nvram_data , index: %d, actual index:%d\n", i, mesh_dct_info.mesh_nv_data[ i ].index));
                    wiced_bt_mesh_push_nvram_data( mesh_dct_info.mesh_nv_data[ i ].data, mesh_dct_info.mesh_nv_data[ i ].len, mesh_dct_info.mesh_nv_data[ i ].index );

            }
            /* intialize mesh */

            wiced_bt_mesh_init(mesh_provision_end, proxy_send_cb, mesh_write_nvram_data_cb, mesh_status_cb);

#if (CLOUD_CONFIG == WICED_REST)
            mesh_init_rest_transport();
#else

            wiced_rtos_delay_milliseconds( 500 );
            wiced_rtos_set_semaphore(&big_mesh_init_semaphore);
            // intializes MQTT transport to use with AWS or Bluemix
          //  mesh_init_mqtt_transport();
#endif
        }
        else
        {
            // Device is not provisioned
            /* intialize mesh */
            wiced_bt_mesh_init(mesh_provision_end, proxy_send_cb, mesh_write_nvram_data_cb, mesh_status_cb);
        }

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

    case BTM_PAIRING_COMPLETE_EVT:
        p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WPRINT_BT_APP_INFO(( "Pairing Complete: %d",p_info->reason));
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WPRINT_BT_APP_INFO(( "Advertisement State Change: %d\n", *p_mode));
        break;

    default:
        break;
    }

    return result;
}


void print_mesh_dct_info()
{
    uint8_t i,j ;
    for (i = 0 ; i < MESH_NV_DATA_MAX_ENTRIES ; i++)
    {
        WPRINT_APP_INFO(("\n data_len = %d \n" , mesh_dct_info.mesh_nv_data[i].len ));
        WPRINT_APP_INFO( ("\n index = %d \n" , mesh_dct_info.mesh_nv_data[i].index ) );
        WPRINT_APP_INFO(("\n ====== \n"));
        for (j = 0 ; j < mesh_dct_info.mesh_nv_data[i].len ; j++)
        {
            WPRINT_APP_INFO(( "  %d " , mesh_dct_info.mesh_nv_data[i].data[j] ));
        }
        WPRINT_APP_INFO(("\n ====== \n"));
    }
}

static wiced_result_t mesh_read_dct()
{
    mesh_dct_t *dct = NULL;
    wiced_dct_read_lock( (void**) &dct, WICED_FALSE, DCT_APP_SECTION, 0, sizeof(*dct) );
    mesh_dct_info.node_authenticated = dct->node_authenticated;
    mesh_dct_info.index_used = dct->index_used;
    memcpy((uint8_t*)mesh_dct_info.mesh_nv_data , dct->mesh_nv_data ,sizeof(dct->mesh_nv_data));
    wiced_dct_read_unlock(dct, WICED_FALSE );
    while(0)
     print_mesh_dct_info();
    return WICED_SUCCESS;
}

int find_index(int id)
{
  int index = -1;
  int i;
  for ( i = 0; i < MESH_NV_DATA_MAX_ENTRIES; i++ )
  {
      if(mesh_dct_info.mesh_nv_data[ i ].index == id)
      {
          index = i;
      }

  }
  return index;
}

static wiced_result_t mesh_write_dct( mesh_dct_t* dct_to_write, int id, uint8_t *packet, uint32_t packet_len )
{
    int index = mesh_dct_info.index_used + 1;
    int dct_index = find_index(id);
    WPRINT_APP_INFO(("mesh_write_dct :\n"));

    if( dct_index != -1)
    {
        WPRINT_APP_INFO(("mesh_write_dct ,existing  index: %d index %d\n", id, dct_index));
        memcpy( (uint8_t*) mesh_dct_info.mesh_nv_data[ dct_index ].data, packet, packet_len );
        mesh_dct_info.mesh_nv_data[ dct_index ].len = (uint8_t) packet_len;
        mesh_dct_info.mesh_nv_data[ dct_index ].index = (uint8_t) id;

    }
    else
    {
        WPRINT_APP_INFO(("mesh_write_dct ,new  index: %d index %d\n", id, index));
        memcpy( (uint8_t*) mesh_dct_info.mesh_nv_data[ index ].data, packet, packet_len );
        mesh_dct_info.mesh_nv_data[ index ].len = (uint8_t) packet_len;
        mesh_dct_info.mesh_nv_data[ index ].index = (uint8_t) id;
        mesh_dct_info.index_used++;
    }


    return WICED_SUCCESS;
}


