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
 * * BLE Serial Over Gatt Sample Application.
 *
 * The sample app performs as a BLE Serial Gatt server (BSG), a Cypress
 * vendor specific service.
 *
 * The peerapps folder which contains peer application for the following platforms (WICED-Studio-x.x/common/peerapps/hci_serial_gatt_service):
 *  - Windows 10
 *  - iOS
 *  - Android
 *
 * * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer.
 * 2. Build and download the application (to the WICED board).
 * 3. On application start the device acts as a GATT server and advertises itself as Serial GATT.
 * 4. Once the device sends advertisements, a peer device (running a peer application
 *    listed above) can connect and use the Serial over Gatt service.
 * 5. Data can be sent to the Peer, using the command console.
 *    e.g. : send_data a b c 1 2 3 4
 *           send_data h e l l o W o r l d !
 * 6. Device Information can also be added using command console.
 *    e.g. : add_device_info Cypress 1.0 2.1 6.x
 *
 */


#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_gatt_db.h"
#include "wiced_bt_l2c.h"
#include "command_console.h"
#include "gki.h"

/******************************************************
 *                      Macros
 ******************************************************/
/* HCI Transport Header (Command  Code, Group Code, Length) */
#define HCI_HEADER_SIZE                                 4

#define SERIAL_GATT_SERVICE_MTU_NEGOTIATION_TIMEOUT     2000     // 2000ms -- 2sec
#define SERIAL_GATT_SERVICE_ACK_TIMEOUT                 500     // in milliseconds
#define SERIAL_GATT_SERVICE_IDLE_TIMEOUT                3000    // 3000ms -- 3sec
#define SERIAL_GATT_SERVICE_CONGESTION_TIMEOUT          50      // in milliseconds

#define SERIAL_GATT_SERVICE_RX_MTU                      512

#define SERIAL_GATT_SERVICE_TRANSPORT_BUFF_NB           2
#define SERIAL_GATT_SERVICE_TRANSPORT_BUFF_SIZE         (SERIAL_GATT_SERVICE_RX_MTU + \
                                                         HCI_HEADER_SIZE)

#define SERIAL_GATT_LARGE_BUFFER_POOL_ID                2       //Pool 2: Large Buffer Pool   - HCI ACL data messages

typedef enum
{
    SERIAL_GATT_SERVICE_CONN_IDLE = 0,
    SERIAL_GATT_SERVICE_CONN_ACTIVE,

} serial_gatt_service_conn_t;

/* BLE connection parameters when in Serial Service is 'Active' (data sent/received) */
#define SERIAL_GATT_SERVICE_ACTIVE_INT_MIN              6
#define SERIAL_GATT_SERVICE_ACTIVE_INT_MAX              6
#define SERIAL_GATT_SERVICE_ACTIVE_LATENCY              0
#define SERIAL_GATT_SERVICE_ACTIVE_CONN_TIMEOUT         200

/* BLE connection parameters when in Serial Service is 'Idle' (no data sent/received) */
#define SERIAL_GATT_SERVICE_IDLE_INT_MIN                60
#define SERIAL_GATT_SERVICE_IDLE_INT_MAX                100
#define SERIAL_GATT_SERVICE_IDLE_LATENCY                0
#define SERIAL_GATT_SERVICE_IDLE_CONN_TIMEOUT           2000

// Protocol definitions
#define SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT              0x01
#define SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT                 0x02
#define SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT                0x04

#define SERIAL_GATT_MAX_CREDITS                             255
#define SERIAL_GATT_DEFAULT_MTU                             23

/******************************************************
 *                External Definitions
 ******************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void                         serial_gatt_init                  ( void );
static wiced_bt_dev_status_t        serial_gatt_management_callback   ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t       serial_gatt_cback                 ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
static void                         serial_gatt_set_advertisement_data(void);
static wiced_result_t               serial_gatt_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t       serial_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t       serial_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static void                         serial_gatt_service_app_init();
static wiced_bt_gatt_status_t       serial_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req );
static wiced_result_t               serial_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete );
static wiced_bt_gatt_status_t       serial_gatt_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_req );
static wiced_bt_gatt_status_t       serial_gatt_mtu_handler(uint16_t conn_id, uint16_t mtu);
static void                         serial_gatt_service_send_data(wiced_bool_t return_credits);
static void                         serial_gatt_service_change_conn_param(serial_gatt_service_conn_t conn_state);
static uint16_t                     serial_gatt_service_header_write(uint8_t *p_buffer, uint8_t flags, uint8_t credits, uint16_t mtu);
static wiced_bt_gatt_status_t       serial_gatt_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_req);
static int                          serial_gatt_service_header_parse(uint8_t *p_credits, uint16_t *p_mtu, uint8_t **pp_data, uint8_t *p_buffer, uint16_t buffer_len);
static void                         serial_gatt_service_bsg_mtu_handler(uint16_t mtu);
static wiced_bool_t                 serial_gatt_service_process_forward_data(uint8_t *p_buffer, uint16_t offset,uint16_t length);
static void                         serial_gatt_service_mtu_timer_callback      (void *arg);
static void                         serial_gatt_service_ack_timer_callback      (void *arg);
static void                         serial_gatt_service_idle_timer_callback     (void *arg);
static void                         serial_gatt_service_mtu_timer_stop(void);
static void                         serial_gatt_service_ack_timer_stop(void);
static void                         serial_gatt_service_idle_timer_stop(void);
static int                          serial_gatt_service_process_data( int argc, char *argv[] );
static int                          serial_gatt_device_info_service( int argc, char *argv[] );

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    uint8_t in;                                             // In index (location to insert)
    uint8_t out;                                            // Out Index (location to extract)
    uint8_t credit[SERIAL_GATT_SERVICE_TRANSPORT_BUFF_NB];
} serial_gatt_credit_queue_t;

typedef struct
{
    uint8_t                         *p_transport_buffer;    // Rx Buffer allocated
    uint8_t                         nb_transport_allocated; // Number of transport allocated
    uint8_t                         credits;                // Nb 'Credit' in this buffer
    uint16_t                        length;                 // data length
    serial_gatt_credit_queue_t      credit_queue;           // Credit queue (to be acked)
} serial_gatt_rx_buffer_cb_t;

typedef struct
{
    BD_ADDR                         remote_addr;            /* remote peer device address */
    uint16_t                        conn_id;                /* connection ID referenced by the stack */
    uint16_t                        peer_mtu;               /* peer MTU */
    wiced_bool_t                    notifications_enabled;  // True if client enabled notifications
    uint8_t                         tx_credits;             // number of credits we have to send OTA data
    uint8_t                         rx_credits_max;         // maximum number of credits to give to peer
    uint8_t                         rx_credits;             // current number of credits to give to peer
    wiced_bool_t                    mtu_update;             // Indicate if MTU must be sent to peer (inband)
    uint16_t                        tx_len;                 // Length of the data in the TX buffer
    uint16_t                        tx_offset;              // Offset of the data in the TX buffer
    uint8_t*                        p_tx_buffer;            // Pointer to the TX buffer
    serial_gatt_rx_buffer_cb_t      rx_buffer_cb;           // Rx buffer control block
    serial_gatt_service_conn_t      conn_state;             // Active or Idle
} serial_gatt_state_t;

#define SERIAL_GATT_CONSOLE_COMMAND_HISTORY_LENGTH  (10)
#define MAX_SERIAL_GATT_COMMAND_LENGTH              (200)
#define SERIAL_GATT_CONSOLE_COMMANDS \
    { (char*) "send_data",          serial_gatt_service_process_data,    0, NULL, NULL, (char *)"send_data [1 byte data]... 30 bytes", (char *)"Send Data" }, \
    { (char*) "add_device_info",    serial_gatt_device_info_service,    0, NULL, NULL, (char *)"add_device_info [manufacturer_name][model_num][firmware_rev][software_rev]", (char *)"Add Device Information" }, \

const command_t serial_gatt_console_command_table[] =
{
    SERIAL_GATT_CONSOLE_COMMANDS
    CMD_TABLE_END
};

/******************************************************
 *               Variable Definitions
 ******************************************************/
static serial_gatt_state_t serial_gatt_state;
static char serial_gatt_command_buffer[MAX_SERIAL_GATT_COMMAND_LENGTH];
static char serial_gatt_command_history_buffer[MAX_SERIAL_GATT_COMMAND_LENGTH * SERIAL_GATT_CONSOLE_COMMAND_HISTORY_LENGTH];
static wiced_bool_t             is_connected = FALSE;

wiced_bool_t        serial_gatt_timer_initialized = WICED_FALSE;
wiced_timer_t       serial_gatt_mtu_timer;
wiced_timer_t       serial_gatt_ack_timer;
wiced_timer_t       serial_gatt_idle_timer;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    /* Initialize WICED platform */
    wiced_core_init();

    /* Initialize Bluetooth controller and host stack */
    wiced_bt_stack_init( serial_gatt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

static void serial_gatt_init( void )
{
    wiced_bt_gatt_status_t status;

    serial_gatt_service_app_init();

    serial_gatt_set_advertisement_data();

    /* Register for gatt event notifications */
    status = wiced_bt_gatt_register( &serial_gatt_cback );
    WPRINT_APP_INFO(("gatt register status: %d\n",status));

    /* Initialize GATT database */
    status = wiced_bt_gatt_db_init( (uint8_t *) gatt_server_db, gatt_server_db_len );
    WPRINT_APP_INFO(("gatt db init status: %d\n",status));

    /* start LE advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WPRINT_BT_APP_INFO( ("Waiting for Serial Gatt to connect...\n") );
}

static void serial_gatt_service_app_init()
{
    memset(&serial_gatt_state, 0, sizeof(serial_gatt_state));

    if (!serial_gatt_timer_initialized)
    {
        serial_gatt_timer_initialized = WICED_TRUE;

        if ( wiced_rtos_init_timer(&serial_gatt_mtu_timer, SERIAL_GATT_SERVICE_MTU_NEGOTIATION_TIMEOUT,
                serial_gatt_service_mtu_timer_callback,NULL) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("failed init mtu timer \n"));
        }
        if (wiced_rtos_init_timer(&serial_gatt_ack_timer, SERIAL_GATT_SERVICE_ACK_TIMEOUT,
                serial_gatt_service_ack_timer_callback, NULL) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("failed init rx timer \n"));
        }
        if (wiced_rtos_init_timer(&serial_gatt_idle_timer, SERIAL_GATT_SERVICE_IDLE_TIMEOUT,
                serial_gatt_service_idle_timer_callback, NULL) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("failed init idle timer \n"));
        }
    }
}

static void serial_gatt_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (char *)wiced_bt_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )wiced_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

static int serial_gatt_service_process_data( int argc, char *argv[] )
{
    uint8_t* data  = NULL;
    uint8_t* p     = NULL;
    uint16_t length  = 0;
    int i;

    if(is_connected)
    {
        length = argc-1;
        /* allocate memory for the data */
        data = (uint8_t*)calloc(length+4, sizeof(uint8_t));
        p = data+4;

        for(i=0;i<=length;i++)
        {
            p[i] = *argv[i+1];
        }

        serial_gatt_service_process_forward_data(data,4,length);
        free(data);

        return ERR_CMD_OK;
    }

    return ERR_UNKNOWN;
}

static int serial_gatt_device_info_service( int argc, char *argv[] )
{
    char *manufacturer_name =  argv[1];
    char *model_number =  argv[2];
    char *firmware_rev = argv[3];
    char *software_rev = argv[4];

    if( argc == 5 )
    {
        if(is_connected)
        {
            memset(device_information_manufacturer_name_string,0,16);
            memset(device_information_model_number_string,0,16);
            memset(device_information_firmware_revision_string,0,8);
            memset(device_information_software_revision_string,0,8);

            strcpy(device_information_manufacturer_name_string,manufacturer_name);
            strcpy(device_information_model_number_string,model_number);
            strcpy(device_information_firmware_revision_string,firmware_rev);
            strcpy(device_information_software_revision_string,software_rev);
            return ERR_CMD_OK;
        }
        return ERR_UNKNOWN;
    }
    return ERR_INSUFFICENT_ARGS;
}

/* Bluetooth management event handler */
static wiced_bt_dev_status_t serial_gatt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda;
    wiced_result_t result;

    switch ( event )
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth controller and host stack enabled */
            WPRINT_BT_APP_INFO( ("Bluetooth enabled (%s)\n", ((p_event_data->enabled.status == WICED_BT_SUCCESS) ? "success":"failure")) );

            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
                wiced_bt_dev_read_local_addr( bda );
                WPRINT_BT_APP_INFO( ("Local Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]) );

                /* Enable serial gatt */
                serial_gatt_init( );

                result = command_console_init(STDIO_UART, sizeof(serial_gatt_command_buffer), serial_gatt_command_buffer,
                        SERIAL_GATT_CONSOLE_COMMAND_HISTORY_LENGTH, serial_gatt_command_history_buffer, " ");
                if (result != WICED_SUCCESS)
                {
                    WPRINT_BT_APP_INFO(("Error starting the command console\r\n"));
                }
                console_add_cmd_table( serial_gatt_console_command_table );
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WPRINT_BT_APP_INFO( ("Security reqeust\n") );
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WPRINT_BT_APP_INFO( ("Pairing IO Capabilities reqeust\n") );
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE; /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND; /* Bonding required */
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WPRINT_BT_APP_INFO( ("Pairing complete %i.\n", p_event_data->pairing_complete.pairing_complete_info.ble.status) );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* adv state change callback */
            WPRINT_BT_APP_INFO( ("---->>> New ADV state: %d\n", p_event_data->ble_advert_state_changed) );
            break;

        default:
            WPRINT_BT_APP_INFO( ("Unhandled Bluetooth Management Event: 0x%x\n", event) );
            break;
    }

    return ( status );
}

/* GATT event handler */
static wiced_bt_gatt_status_t serial_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = serial_gatt_conn_status_callback( &p_event_data->connection_status );
            break;

        case GATT_OPERATION_CPLT_EVT:
                result = serial_gatt_operation_comp_cb( &p_event_data->operation_complete );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = serial_gatt_req_cb( &p_event_data->attribute_request );
            break;

        default:
            break;
    }

    return result;
}

static wiced_result_t serial_gatt_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
    is_connected = p_status->connected;
    if ( p_status->connected )
    {
        return serial_gatt_connection_up( p_status );
    }
    else
    {
        return serial_gatt_connection_down( p_status );
    }
}

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t serial_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WPRINT_BT_APP_INFO( ( "serial_gatt_connection_up  id:%d\n:", p_status->conn_id) );

    /* Update the connection handler.  Save address of the connected device. */
    serial_gatt_state.conn_id = p_status->conn_id;
    memcpy(serial_gatt_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));
    // We need to assume that MTU is 23 bytes and calculate Rx credit when timer expires
    serial_gatt_state.peer_mtu              = 0;
    serial_gatt_state.notifications_enabled = WICED_FALSE;

    if (wiced_rtos_start_timer( &serial_gatt_mtu_timer ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Err: failed to start mtu timer\n"));
    }

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    WPRINT_BT_APP_INFO( ( "Stopping Advertisements%d\n", result ) );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t serial_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WPRINT_BT_APP_INFO( ( "connection_down  conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason ) );

    /* Resetting the device info */
    memset( serial_gatt_state.remote_addr, 0, 6 );
    serial_gatt_state.conn_id = 0;

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    WPRINT_BT_APP_INFO( ( "wiced_bt_start_advertisements %d\n", result ) );

    /* Stop any active timer */
    serial_gatt_service_mtu_timer_stop();
    serial_gatt_service_ack_timer_stop();
    serial_gatt_service_idle_timer_stop();

    /* Free any pending Rx */
    if (serial_gatt_state.rx_buffer_cb.p_transport_buffer)
    {
        free(serial_gatt_state.rx_buffer_cb.p_transport_buffer);
        serial_gatt_state.rx_buffer_cb.p_transport_buffer = NULL;
        serial_gatt_state.rx_buffer_cb.credits = 0;
        serial_gatt_state.rx_buffer_cb.length = 0;
    }

    /* Clear all the Connection related information */
    serial_gatt_state.notifications_enabled = 0;
    serial_gatt_state.tx_credits= 0;
    serial_gatt_state.rx_credits_max = 0;
    serial_gatt_state.rx_credits = 0;
    serial_gatt_state.tx_len = 0;
    serial_gatt_state.tx_offset = 0;
    serial_gatt_state.peer_mtu = 0;
    serial_gatt_state.mtu_update = WICED_FALSE;
    serial_gatt_state.conn_id = 0;
    memset(serial_gatt_state.remote_addr, 0, BD_ADDR_LEN);
    memset(&serial_gatt_state.rx_buffer_cb, 0, sizeof(serial_gatt_state.rx_buffer_cb));
    serial_gatt_state.conn_state = SERIAL_GATT_SERVICE_CONN_IDLE;

    return WICED_BT_SUCCESS;
}

/*
 * serial_gatt_service_mtu_timer_stop
 */
static void serial_gatt_service_mtu_timer_stop(void)
{
    wiced_result_t result;
    result = wiced_rtos_is_timer_running(&serial_gatt_mtu_timer);
    if(result == WICED_SUCCESS)
    {
        result = wiced_rtos_stop_timer(&serial_gatt_mtu_timer);
    }
}

/*
 * serial_gatt_service_ack_timer_stop
 */
static void serial_gatt_service_ack_timer_stop(void)
{
    wiced_result_t result;
    result = wiced_rtos_is_timer_running(&serial_gatt_ack_timer);
    if(result == WICED_SUCCESS)
    {
        wiced_rtos_stop_timer(&serial_gatt_ack_timer);
    }
}

/*
 * serial_gatt_service_idle_timer_stop
 */
static void serial_gatt_service_idle_timer_stop(void)
{
    wiced_result_t result;
    result = wiced_rtos_is_timer_running(&serial_gatt_idle_timer);
    if( result == WICED_SUCCESS )
    {
        wiced_rtos_stop_timer(&serial_gatt_idle_timer);
    }
}

/*
 * Timer callback routine
 */
static void serial_gatt_service_mtu_timer_callback(void *arg)
{
    // master did not send MTU request, default is 23 bytes.
    serial_gatt_mtu_handler(serial_gatt_state.conn_id, GATT_DEF_BLE_MTU_SIZE);
}

/*
 * Ack timeout
 */
static void serial_gatt_service_ack_timer_callback(void *arg)
{
    // it's time to send RxCredit (if there are some)
    serial_gatt_service_send_data(WICED_TRUE);
}

/*
 * serial_gatt_service_idle_timer_callback
 * This timer means that no Serial data has been sent/received for some time.
 * We can reduce the connection Interval to save power
 */
static void serial_gatt_service_idle_timer_callback(void *arg)
{
    serial_gatt_service_change_conn_param(SERIAL_GATT_SERVICE_CONN_IDLE);
}

/*
 * This is a GATT request callback
 */
static wiced_bt_gatt_status_t serial_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;
    uint16_t               conn_id = p_req->conn_id;

    WPRINT_APP_INFO(( "GATT request conn_id:%d type:%d\n", conn_id, p_req->request_type ));

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = serial_gatt_read_handler( p_req->conn_id, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = serial_gatt_write_handler( p_req->conn_id, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            break;

        case GATTS_REQ_TYPE_MTU:
            result = serial_gatt_mtu_handler( p_req->conn_id, p_req->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            break;

        default:
            break;
    }

    return result;
}

/*
 * Find attribute description by handle
 */
static attribute_t * serial_gatt_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i < gatt_db_ext_attr_tbl_size; i++ )
    {
        if ( gatt_db_ext_attr_tbl[i].handle == handle )
        {
            return ( &gatt_db_ext_attr_tbl[i] );
        }
    }
    WPRINT_APP_INFO(( "attribute not found:%x\n", handle ));
    return NULL;
}

/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */

static wiced_bt_gatt_status_t serial_gatt_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t *p_read_req)
{
    wiced_bt_gatt_status_t  status;
    attribute_t             *puAttribute;
    int                     attr_len_to_copy;

    /* only readable attribute is characteristic client configuration descriptor */
    if (p_read_req->handle == HDLD_SERIAL_GATT_SERIAL_DATA_CLIENT_CONFIGURATION)
    {
        if (*p_read_req->p_val_len < 2)
        {
            status = WICED_BT_GATT_ILLEGAL_PARAMETER;
        }
        else
        {
            *p_read_req->p_val_len = 2;

            p_read_req->p_val[0]   = serial_gatt_state.notifications_enabled ? GATT_CLIENT_CONFIG_NOTIFICATION : GATT_CLIENT_CONFIG_NONE;
            p_read_req->p_val[1]   = 0;

            status = WICED_BT_GATT_SUCCESS;
        }
    }
    else if ( (( puAttribute = serial_gatt_get_attribute(p_read_req->handle) ) != NULL)
            || (p_read_req->handle == HDLC_BATTERY_SERVICE_BATTERY_LEVEL_VALUE) )
    {
        /* Dummy battery value read increment */
        if( p_read_req->handle == HDLC_BATTERY_SERVICE_BATTERY_LEVEL_VALUE)
        {
            if ( battery_service_battery_level++ > 99 )
            {
                battery_service_battery_level = 0;
            }
        }

        attr_len_to_copy = puAttribute->attr_len;

        WPRINT_BT_APP_INFO(("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy
        ));

        if ( p_read_req->offset >= puAttribute->attr_len )
        {
            attr_len_to_copy = 0;
        }

        if ( attr_len_to_copy != 0 )
        {
            uint8_t *from;
            int      to_copy = attr_len_to_copy - p_read_req->offset;


            if ( to_copy > *p_read_req->p_val_len )
            {
                to_copy = *p_read_req->p_val_len;
            }

            from = ((uint8_t *)puAttribute->p_attr) + p_read_req->offset;
            *p_read_req->p_val_len = to_copy;

            memcpy( p_read_req->p_val, from, to_copy);
        }

        return WICED_BT_GATT_SUCCESS;
    }
    else
    {
        WPRINT_APP_INFO(("This is invalid\n"));
        status = WICED_BT_GATT_INVALID_HANDLE;
    }
    return status;
}

/*
 * Handle peer request to set MTU
 */
static wiced_bt_gatt_status_t serial_gatt_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    serial_gatt_service_mtu_timer_stop();

    serial_gatt_state.peer_mtu = mtu;
    serial_gatt_state.mtu_update = WICED_TRUE;

    // MTU can be 23 - 512. We created a pool of X buffers of 512 bytes.
    // The nnumber of RxCredit is: (512 / mtu) + (X - 1)
    serial_gatt_state.rx_credits_max = SERIAL_GATT_SERVICE_RX_MTU / mtu;
    serial_gatt_state.rx_credits_max += SERIAL_GATT_SERVICE_TRANSPORT_BUFF_NB - 1;
    serial_gatt_state.rx_credits = serial_gatt_state.rx_credits_max;

    WPRINT_APP_INFO(("MTU handler mtu:%d num transport buffers:%d ccc:%d\n", mtu,
            serial_gatt_state.rx_credits_max, serial_gatt_state.notifications_enabled));

    // transmit initial number of credits
    serial_gatt_service_send_data(WICED_FALSE);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Handles Write Requests received from Client device
 */
static wiced_bt_gatt_status_t serial_gatt_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_req)
{
    wiced_bt_gatt_status_t  status = WICED_BT_GATT_INVALID_HANDLE;
    uint16_t                len = p_write_req->val_len;
    uint8_t                 credit;
    uint8_t*                p_data;
    int                     header_size;
    uint16_t                mtu;
    wiced_bool_t            notifications_enabled;

    // process characteristic client configuration descriptor which allows or disallows to send notifications
    if (p_write_req->handle == HDLD_SERIAL_GATT_SERIAL_DATA_CLIENT_CONFIGURATION)
    {
        if (p_write_req->val_len != 2)
        {
            return WICED_BT_GATT_CCC_CFG_ERR;
        }
        notifications_enabled = p_write_req->p_val[0] & GATT_CLIENT_CONFIG_NOTIFICATION;

        /* If peer enabled notification (i.e. Enable/Open BSG) */
        if (notifications_enabled)
        {
            /* GATT MTU must have been negotiated before BSG is opened */
            if (serial_gatt_state.peer_mtu == 0)
            {
                WPRINT_APP_INFO(("Warning: BSG Opened before MTU negotiation!\n"));
            }
            else
            {
                /* Peer opened/reopened BSG => send MTU & Initial Credit */
                serial_gatt_mtu_handler(conn_id, serial_gatt_state.peer_mtu);
            }

            /* If BSG was already opened */
            if (serial_gatt_state.notifications_enabled)
            {
                WPRINT_APP_INFO(("Warning: BSG Re-Opened!\n"));
            }

            // as client configuration descriptor changed, we might be able to send data
            serial_gatt_service_send_data(WICED_FALSE);
        }
        else
        {
            if (serial_gatt_state.p_tx_buffer != NULL)
            {
                free(serial_gatt_state.p_tx_buffer);
                serial_gatt_state.p_tx_buffer = NULL;
            }
        }

        serial_gatt_state.notifications_enabled = notifications_enabled;

        return WICED_BT_GATT_SUCCESS;
    }

    // the only characteristic this service allows to write to is Serial Data
    if (p_write_req->handle != HDLC_SERIAL_GATT_SERIAL_DATA_VALUE)
    {
        WPRINT_APP_INFO(("illegal write handle:%04x\n", p_write_req->handle));
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Parse the Header */
    header_size = serial_gatt_service_header_parse(&credit, &mtu, &p_data, p_write_req->p_val, len);
    if (header_size < 0)
    {
        return WICED_BT_GATT_ERROR;
    }

    /* Restart Idle timer */
    serial_gatt_service_idle_timer_stop();
    if (wiced_rtos_start_timer( &serial_gatt_idle_timer ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Err: failed to start Idle timer\n"));
    }
    /* Change Connection Parameters to Active. */
    serial_gatt_service_change_conn_param(SERIAL_GATT_SERVICE_CONN_ACTIVE);

    /* Skip Header */
    len -= header_size;

    if ((p_data != NULL) && (len > 0))
    {
        uint8_t *p_tmp = p_data;
        uint8_t len_tmp = len;
        WPRINT_APP_INFO(("received data: \n"));
        while(len_tmp)
        {
            WPRINT_APP_INFO(("%d ",p_tmp[len_tmp-1]));
            len_tmp--;
        }
        WPRINT_APP_INFO(("\n"));
        //rx credit is incremented here as the received packet is printed and discarded.
        //In real world app, the received data may be buffered and hence the rx credit shall be incremented as and when the buffer is consumed.
        serial_gatt_state.rx_credits ++;
        /* Send number of RxCredit to peer */
        serial_gatt_service_send_data(WICED_FALSE);
        // Restart the Ack timer
        serial_gatt_service_ack_timer_stop();
        if (wiced_rtos_start_timer( &serial_gatt_ack_timer ) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Err: failed to start Ack timer\n"));
        }
    }

    /* If BSG MTU received */
    if (mtu)
    {
        serial_gatt_service_bsg_mtu_handler(mtu);
    }

    if (credit)
    {
        WPRINT_APP_INFO(("Received %d TxCredits New TxCredit:%d\n", credit,
                serial_gatt_state.tx_credits + credit));
    }

    // process number of credits if received from the peer
    if (credit + serial_gatt_state.tx_credits > SERIAL_GATT_MAX_CREDITS)
    {
        WPRINT_APP_INFO(("Err: illegal credits cur: len:%d\n", len));
        return WICED_BT_GATT_ERROR;
    }

    // tx_credits indicates how many packets we can send to the peer.
    serial_gatt_state.tx_credits += credit;

#if defined (SERIAL_GATT_TX_CREDITS_LIMIT)
    // Debug: Limit TxCredit
    if (serial_gatt_state.tx_credits > SERIAL_GATT_TX_CREDITS_LIMIT)
    {
        serial_gatt_state.tx_credits = SERIAL_GATT_TX_CREDITS_LIMIT;
        WPRINT_APP_INFO(("Limit TxCredits to %d\n", SERIAL_GATT_TX_CREDITS_LIMIT));
    }
#endif

    // as we got more credits, we might be able to send data out, or may need to send credits.
    serial_gatt_service_send_data(WICED_FALSE);

    return status;
}

/*
 * Operation complete received from the GATT server
 */
static wiced_result_t serial_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete )
{
    uint16_t conn_id = p_complete->conn_id;
    WPRINT_APP_INFO(("[%s] conn_id: %x operation_type:%x\n",__func__,conn_id,p_complete->op));
    return ( WICED_SUCCESS );
}

/*
 * Perform check on availability of enough buffers to avoid congestion
 */

static wiced_bool_t serial_gatt_proceed_on_congestion_chk ()
{
    uint16_t buffs_used;
    buffs_used = GKI_poolutilization (SERIAL_GATT_LARGE_BUFFER_POOL_ID);

    if(buffs_used > 50)
    {
        return WICED_FALSE;
    }

        return WICED_TRUE;
}

/*
 * if we have credits and there is a tx buffer, send it now.  Return TRUE if buffer was sent out.
 * Parameter return_credits is set to TRUE if it is time to send credits to peer
 */
static void serial_gatt_service_send_data(wiced_bool_t return_credits)
{
    wiced_bt_gatt_status_t status;
    uint16_t bytes_to_send;
    uint8_t flags;
    const int MTU_OVERHEAD = 3;

    if (!serial_gatt_state.notifications_enabled)
    {
        return;
    }

    if(!serial_gatt_proceed_on_congestion_chk())
    {
         WPRINT_APP_INFO(("serial_gatt_proceed_on_congestion_chk() starting timer\n"));
        // Reuse Ack timer for congestion
        serial_gatt_service_ack_timer_stop();
        if (wiced_rtos_start_timer( &serial_gatt_ack_timer ) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Err: failed to start congestion timer\n"));
        }
        return;
    }

    /* Restart Idle timer */
    serial_gatt_service_idle_timer_stop();
    if (wiced_rtos_start_timer( &serial_gatt_idle_timer ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Err: failed to start Idle timer\n"));
    }
    /* Change Connection Parameters to Active. */
    serial_gatt_service_change_conn_param(SERIAL_GATT_SERVICE_CONN_ACTIVE);

    // if tx buffer is empty OR if No Tx Credit, we may still need to send credits
    if ((serial_gatt_state.p_tx_buffer == NULL) || (serial_gatt_state.tx_credits == 0))
    {
        if ((serial_gatt_state.rx_credits != 0) &&
            (return_credits || (serial_gatt_state.rx_credits >= (serial_gatt_state.rx_credits_max / 2))))
        {
            uint8_t buffer[4];

            if (serial_gatt_state.mtu_update)
            {
                WPRINT_APP_INFO(("BSG Send Credit:%d and MTU:%d\n",
                        serial_gatt_state.rx_credits, serial_gatt_state.peer_mtu));

                flags = SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT |
                        SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT;
            }
            else
            {
                WPRINT_APP_INFO(("BSG Send Credit:%d\n", serial_gatt_state.rx_credits));
                flags = SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT;
            }

            /* Write packet header */
            bytes_to_send = serial_gatt_service_header_write(buffer, flags,
                    serial_gatt_state.rx_credits, serial_gatt_state.peer_mtu);

            status = wiced_bt_gatt_send_notification(serial_gatt_state.conn_id,
                    HDLC_SERIAL_GATT_SERIAL_DATA_VALUE, bytes_to_send, buffer);
            if (status != WICED_BT_GATT_SUCCESS)
            {
                WPRINT_APP_INFO(("err:wiced_bt_gatt_send_notification (1) failed %d", status));
                return;
            }
            serial_gatt_state.rx_credits = 0;
            serial_gatt_state.mtu_update = WICED_FALSE;
        }
        return;
    }

    while ((serial_gatt_state.tx_credits > 0) && (serial_gatt_state.p_tx_buffer != NULL))
    {
        // send up to peer MTU number of bytes
        uint8_t *p_tmp;
        uint16_t header_size;

        if (serial_gatt_state.rx_credits != 0)
        {
            if (serial_gatt_state.mtu_update)
            {
                /* need to add 4 bytes header (flags, credits and MTU) */
                bytes_to_send = serial_gatt_state.tx_len < serial_gatt_state.peer_mtu - (4 + MTU_OVERHEAD) ?
                     serial_gatt_state.tx_len : serial_gatt_state.peer_mtu - (4 + MTU_OVERHEAD);

                WPRINT_APP_INFO(("BSG Send Credit:%d MTU:%d Data len:%d\n",
                        serial_gatt_state.rx_credits, serial_gatt_state.peer_mtu, bytes_to_send));

                flags = SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT |
                        SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT    |
                        SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT;

                p_tmp = &serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 4];
            }
            else
            {
                /* need to add 2 bytes header (flags, credits) */
                bytes_to_send = serial_gatt_state.tx_len < serial_gatt_state.peer_mtu - (2 + MTU_OVERHEAD) ?
                     serial_gatt_state.tx_len : serial_gatt_state.peer_mtu - (2 + MTU_OVERHEAD);

                WPRINT_APP_INFO(("BSG Send Credit:%d and Data len:%d\n",
                        serial_gatt_state.rx_credits, bytes_to_send));

                flags = SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT |
                        SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT;

                p_tmp = &serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 2];
            }

            /* Write packet header */
            header_size = serial_gatt_service_header_write(p_tmp, flags,
                    serial_gatt_state.rx_credits, serial_gatt_state.peer_mtu);

            /* Send the notification */
            status = wiced_bt_gatt_send_notification(serial_gatt_state.conn_id,
                    HDLC_SERIAL_GATT_SERIAL_DATA_VALUE, bytes_to_send + header_size, p_tmp);
            if (status != WICED_BT_GATT_SUCCESS)
            {
                WPRINT_APP_INFO(("err:wiced_bt_gatt_send_notification (2) failed %d", status));
                return;
            }
        }
        else
        {
            if (serial_gatt_state.mtu_update)
            {
                /* need to add 3 bytes header (flags and MTU) */
                bytes_to_send = serial_gatt_state.tx_len < serial_gatt_state.peer_mtu - (3 + MTU_OVERHEAD) ?
                     serial_gatt_state.tx_len : serial_gatt_state.peer_mtu - (3 + MTU_OVERHEAD);

                WPRINT_APP_INFO(("BSG Send MTU:%d Data len:%d\n",
                        serial_gatt_state.peer_mtu, bytes_to_send));

                flags = SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT    |
                        SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT;

                p_tmp = &serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 3];
            }
            else
            {
                /* need to add 1 byte header (flags) */
                bytes_to_send = serial_gatt_state.tx_len < serial_gatt_state.peer_mtu - (1 + MTU_OVERHEAD) ?
                     serial_gatt_state.tx_len : serial_gatt_state.peer_mtu - (1 + MTU_OVERHEAD);

                WPRINT_APP_INFO(("BSG Send Data len:%d\n", bytes_to_send));

                flags = SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT;

                p_tmp = &serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 1];
            }

            /* Write packet header */
            header_size = serial_gatt_service_header_write(p_tmp, flags,
                    serial_gatt_state.rx_credits, serial_gatt_state.peer_mtu);

            /* Send the notification */
            status = wiced_bt_gatt_send_notification(serial_gatt_state.conn_id,
                    HDLC_SERIAL_GATT_SERIAL_DATA_VALUE, bytes_to_send + header_size, p_tmp);
            if (status != WICED_BT_GATT_SUCCESS)
            {
                WPRINT_APP_INFO(("err:wiced_bt_gatt_send_notification (3) failed %d", status));
                return;
            }
        }

        /* One notification sent. Update Credits (Rx/Tx), RemainingLen, Offset and MTU update */
        serial_gatt_state.tx_credits--;
        //("TxCredit:%d", serial_gatt_state.tx_credits);
        serial_gatt_state.tx_len    -= bytes_to_send;
        serial_gatt_state.tx_offset += bytes_to_send;
        serial_gatt_state.rx_credits = 0;
        serial_gatt_state.mtu_update = WICED_FALSE;

        // if we are done with this buffer, send notification to the MCU and release the buffer
        if (serial_gatt_state.tx_len == 0)
        {
            if (serial_gatt_state.p_tx_buffer != NULL)
            {
                free(serial_gatt_state.p_tx_buffer);
                serial_gatt_state.p_tx_buffer = NULL;
            }
        }
    }
}

/*
 * serial_gatt_service_change_conn_param
 */
static void serial_gatt_service_change_conn_param(serial_gatt_service_conn_t conn_state)
{
    wiced_bool_t res;

    /* If same connection state, do nothing */
    if (serial_gatt_state.conn_state == conn_state)
    {
        return;
    }

    /* Save new connection State */
    serial_gatt_state.conn_state = conn_state;

    /* If Serial Connection is active (data sent/received) */
    if (conn_state == SERIAL_GATT_SERVICE_CONN_ACTIVE)
    {
        WPRINT_APP_INFO(("BSG Active. Reduce Connection Interval\n"));
        res = wiced_bt_l2cap_update_ble_conn_params(serial_gatt_state.remote_addr,
                SERIAL_GATT_SERVICE_ACTIVE_INT_MIN,
                SERIAL_GATT_SERVICE_ACTIVE_INT_MAX,
                SERIAL_GATT_SERVICE_ACTIVE_LATENCY,
                SERIAL_GATT_SERVICE_ACTIVE_CONN_TIMEOUT);
    }
    else
    {
        WPRINT_APP_INFO(("BSG Idle. Increase Connection Interval\n"));
        res = wiced_bt_l2cap_update_ble_conn_params(serial_gatt_state.remote_addr,
                SERIAL_GATT_SERVICE_IDLE_INT_MIN,
                SERIAL_GATT_SERVICE_IDLE_INT_MAX,
                SERIAL_GATT_SERVICE_IDLE_LATENCY,
                SERIAL_GATT_SERVICE_IDLE_CONN_TIMEOUT);
    }
    if (res != WICED_TRUE)
        WPRINT_APP_INFO(("Err: L2CA_UpdateBleConnParams failed"));
}

/*
 * serial_gatt_service_header_write
 * Write packet header (flags, credit, mtu)
 */
static uint16_t serial_gatt_service_header_write(uint8_t *p_buffer, uint8_t flags,
                    uint8_t credits, uint16_t mtu)
{
    uint8_t *p = p_buffer;

    UINT8_TO_STREAM(p, flags);

    /* If Credit flag set */
    if (flags & SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT)
    {
        UINT8_TO_STREAM(p, credits);        /* Insert Credits field */
    }
    /* If MTU flag set */
    if (flags & SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT)
    {
        UINT16_TO_STREAM(p, mtu);           /* Insert MTU field */
    }

    /* Return number of bytes written */
    return (uint16_t)(p - p_buffer);
}

/*
 * serial_gatt_service_header_parse
 */
static int serial_gatt_service_header_parse(uint8_t *p_credits, uint16_t *p_mtu,
        uint8_t **pp_data, uint8_t *p_buffer, uint16_t buffer_len)
{
    uint8_t *p = p_buffer;
    uint8_t flags;

    /* Received buffer must contain at least two bytes  */
    if (buffer_len < 2)
    {
        WPRINT_APP_INFO(("Err: BSG Illegal len:%d\n", buffer_len));
        return -1;
    }

    STREAM_TO_UINT8(flags, p);
    buffer_len--;

    /* If No flag set, this is an invalid packet */
    if (flags == 0)
    {
        WPRINT_APP_INFO(("Err: BSG no Flag set\n"));
        return -1;
    }

    /* If Credit flag set */
    if (flags & SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT)
    {
        STREAM_TO_UINT8(*p_credits, p);        /* Extract Credits field */
        buffer_len--;
    }
    else
    {
        *p_credits = 0;
    }

    /* If MTU flag set */
    if (flags & SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT)
    {
        /* MTU is two bytes  */
        if (buffer_len < 2)
        {
            WPRINT_APP_INFO(("Err: BSG Illegal MTU len\n"));
            return -1;
        }
        STREAM_TO_UINT16(*p_mtu, p);           /* Extract MTU field */
        buffer_len -= 2;
    }
    else
    {
        *p_mtu = 0;
    }

    /* If Data flag set */
    if (flags & SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT)
    {
        *pp_data = p;
    }
    else
    {
        *pp_data = NULL;
    }

    /* Return number header size */
    return (p - p_buffer);
}

/*
 * Handle peer request to set BSG MTU
 */
static void serial_gatt_service_bsg_mtu_handler(uint16_t mtu)
{
    WPRINT_APP_INFO(("BSG MTU handler mtu:%d\n", mtu));

    if (mtu > serial_gatt_state.peer_mtu)
    {
        WPRINT_APP_INFO(("Err: BSG Rx MTU (%d) is bigger than GATT MTU (%d)\n",
                mtu, serial_gatt_state.peer_mtu));
    }
}

/*
 * Handle forward data packet
 */
static wiced_bool_t serial_gatt_service_process_forward_data(uint8_t *p_buffer, uint16_t offset,
        uint16_t length)
{
    WPRINT_APP_INFO(("BSG Data Tx len:%d\n", length));

    if (!serial_gatt_state.notifications_enabled)
    {
        WPRINT_APP_INFO(("Err: fwd data when BSG closed\n"));
        return WICED_TRUE;
    }

    if (serial_gatt_state.p_tx_buffer != NULL)
    {
        WPRINT_APP_INFO(("Err: fwd data with buffer present\n"));
        return WICED_TRUE;
    }

    serial_gatt_state.p_tx_buffer = p_buffer;
    serial_gatt_state.tx_offset   = offset;
    serial_gatt_state.tx_len      = length;

    serial_gatt_service_send_data(WICED_FALSE);
    return WICED_FALSE;
}
