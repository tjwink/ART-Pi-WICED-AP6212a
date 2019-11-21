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
 */

#include <stdio.h>
#include <string.h>
#include "wwd_debug.h"
#include "command_console.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_hal_rand.h"
#include "wiced_utilities.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define UUID_BLE_THROUGHPUT_SERVICE                    0x24, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0xff
#define UUID_BLE_THROUGHPUT_TEST      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8e
#define TEST_VECTOR_SIZE 20

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    HANDLE_BLE_THROUGHPUT_SERVICE = 0x90, HANDLE_BLE_THROUGHPUT_TEST, //char value handle
    HANDLE_BLE_THROUGHPUT_TEST_VAL, //char value handle

} target_device_bt_db_tags;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void* p_attr;
} attribute_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

wiced_bt_gatt_status_t target_device_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
wiced_bt_gatt_status_t target_device_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
wiced_bt_gatt_status_t target_device_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
void target_device_set_advertisement_data( void );
wiced_bt_gatt_status_t target_device_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static uint32_t ble_throughput_test_val = 0;
static uint16_t conn_id;
static int user_packet_cnt = 1, pkt_received = 0;
static uint8_t *remote_bd_addr;
static wiced_timer_t tick_timer;
static uint8_t* test_char_notify_value;

const uint8_t target_device_gatt_database[ ] =
{
// Declare mandatory GATT service
        PRIMARY_SERVICE_UUID128( HANDLE_BLE_THROUGHPUT_SERVICE, UUID_BLE_THROUGHPUT_SERVICE ),

        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_BLE_THROUGHPUT_TEST, HANDLE_BLE_THROUGHPUT_TEST_VAL, UUID_BLE_THROUGHPUT_TEST, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

};

attribute_t gauAttributes[ ] =
{
{ HANDLE_BLE_THROUGHPUT_TEST_VAL, 1, &ble_throughput_test_val } };

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Sets advertisement data for the target device
 */
void target_device_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[ 3 ];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t ble_throughput_service_uuid[ LEN_UUID_128 ] =
    { UUID_BLE_THROUGHPUT_SERVICE };

    char * device_name = "BLE Test";

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[ num_elem ].len = sizeof(uint8_t);
    adv_elem[ num_elem ].p_data = &flag;
    num_elem++;

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[ num_elem ].len = LEN_UUID_128;
    adv_elem[ num_elem ].p_data = ble_throughput_service_uuid;
    num_elem++;

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[ num_elem ].len = strlen( device_name );
    adv_elem[ num_elem ].p_data = (uint8_t*) device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );
}

/*
 * Initialize GATT database and start advertisements
 */

int initialize_gatt_db_and_advertise( )
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result;

    gatt_status = wiced_bt_gatt_register( target_device_gatts_callback );

    if ( WICED_BT_GATT_SUCCESS == gatt_status )
        WPRINT_APP_INFO( ( "GATT Register : OK \n") );
    else
    {
        WPRINT_APP_INFO( ( "GATT Register : FAILED\n" ) );
        return gatt_status;
    }

    gatt_status = wiced_bt_gatt_db_init( target_device_gatt_database, sizeof( target_device_gatt_database ) );

    /* Set the advertising data and make the device discoverable */
    target_device_set_advertisement_data( );
    result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );

    return result;
}

/*
 * Find attribute description by handle
 */
static attribute_t * target_device_get_attribute( uint16_t handle, uint16_t conn_id )
{
    int i;
    for ( i = 0; i < ARRAY_SIZE( gauAttributes ); i++ )
    {
        if ( gauAttributes[ i ].handle == handle )
        {
            return ( &gauAttributes[ i ] );
        }
    }

    WPRINT_APP_INFO( ( "attribute not found:%x\n", handle ) );
    return NULL;
}

wiced_bt_gatt_status_t target_device_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = target_device_gatts_req_write_handler( p_data->conn_id, &( p_data->data.write_req ) );
            break;
        case GATTS_REQ_TYPE_MTU:
            WPRINT_APP_INFO( ( "MTU size requested %d\n", p_data->data.mtu) );
            result = WICED_BT_GATT_SUCCESS;
            break;
        default:
            WPRINT_APP_INFO( ( "Uhhandled : target_device_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type ) );
            break;
    }

    return result;
}
wiced_bt_gatt_status_t target_device_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        WPRINT_APP_INFO( ("CONNECTED %d\n", p_status->conn_id) );
        remote_bd_addr = p_status->bd_addr;
        conn_id = p_status->conn_id;
        return WICED_BT_GATT_SUCCESS;
    }
    conn_id = 0;
    return WICED_BT_GATT_SUCCESS;
}

void target_device_process_data_from_slave( int len, uint8_t *data )
{
    if ( pkt_received++ % 100 == 0 )
        WPRINT_APP_DEBUG(("NOTIFY:%d\n",pkt_received));
}

wiced_bt_gatt_status_t target_device_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data )
{
    switch ( p_data->op )
    {
        case GATTC_OPTYPE_NOTIFICATION:
            target_device_process_data_from_slave( p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
            break;
    }
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t target_device_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = target_device_gatts_conn_status_cb( &p_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = target_device_gatts_req_cb( &p_data->attribute_request );
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = target_device_gatt_op_comp_cb( &p_data->operation_complete );
            break;

        default:
            break;
    }
    return result;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t target_device_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    attribute_t *puAttribute;
    int attr_len_to_copy;

    WPRINT_APP_INFO( ("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len ) );
    if ( ( puAttribute = target_device_get_attribute( p_data->handle, conn_id ) ) == NULL )
    {
        WPRINT_APP_INFO( ("write_hndlr attr not found hdl:%x\n", p_data->handle ) );
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->attr_len;
    if ( p_data->offset == 0 )
        memset( (uint8_t *) puAttribute->p_attr, 0, attr_len_to_copy );
    if ( p_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }
    if ( attr_len_to_copy != 0 )
    {
        uint8_t *to;
        int to_copy = attr_len_to_copy - p_data->offset;
        if ( to_copy > p_data->val_len )
        {
            to_copy = p_data->val_len;
        }
        to = ( (uint8_t *) puAttribute->p_attr ) + p_data->offset;
        memcpy(to, p_data->p_val, to_copy);
    }
    switch ( p_data->handle )
    {

        case HANDLE_BLE_THROUGHPUT_TEST_VAL:
            WPRINT_APP_INFO( ("### HANDLE_BLE_THROUGHPUT_TEST_VAL ### \n %ld \n", ble_throughput_test_val) );
            break;
        default:
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    return result;
}

/*
 * Fine timeout callback
 */
void target_device_fine_timeout( void* arg )
{
    if ( conn_id )
    {
       wiced_bt_gatt_send_notification( conn_id, HANDLE_BLE_THROUGHPUT_TEST_VAL, (TEST_VECTOR_SIZE * user_packet_cnt), test_char_notify_value );
    }
}

int test_bt_tx( int argc, char* argv[ ] )
{
    int fine_timeout = atoi( argv[ 1 ] );
    wiced_result_t result;
    WPRINT_APP_INFO( ("%s argument count %d\n",__func__, argc) );

    if ( ( BTM_BLE_ADVERT_OFF == wiced_bt_ble_get_current_advert_mode( ) ) && conn_id )
        initialize_gatt_db_and_advertise( );
    else
        WPRINT_APP_INFO( ("Device already advertised\n") );

    if ( argc > 2 )
        user_packet_cnt = atoi( argv[ 2 ] );

    if(test_char_notify_value != NULL)
        free(test_char_notify_value);

    test_char_notify_value = (uint8_t *) malloc(TEST_VECTOR_SIZE * user_packet_cnt);

    memset(test_char_notify_value, 5, TEST_VECTOR_SIZE * user_packet_cnt);

    result = wiced_rtos_init_timer( &tick_timer, fine_timeout, (timer_handler_t) target_device_fine_timeout, NULL );

    if ( result != 0 )
    {
        WPRINT_APP_ERROR( "Failed to create timer" );
        return ERR_ADDRESS;
    }

    wiced_rtos_start_timer( &tick_timer );
    WPRINT_APP_INFO( ("Fine Timeout :: %d ms Packet Count :: %d\n", fine_timeout, user_packet_cnt) );
    return ERR_CMD_OK;
}

int start_bt_advertise( int argc, char* argv[ ] )
{
    WPRINT_APP_INFO( ("%s\n",__func__) );

    if ( BTM_BLE_ADVERT_OFF == wiced_bt_ble_get_current_advert_mode( ) )
    {
        return initialize_gatt_db_and_advertise( );
    }
    else
    {
        WPRINT_APP_INFO( ("Device already advertised\n") );
    }

    return ERR_CMD_OK;
}

int stop_bt_tx_rx( int argc, char* argv[ ] )
{
    WPRINT_APP_INFO( ("%s\n",__func__) );

    if(test_char_notify_value != NULL)
    {
        free(test_char_notify_value);
        test_char_notify_value = NULL;
    }

    wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    wiced_bt_gatt_cancel_connect( remote_bd_addr, WICED_TRUE );
    wiced_bt_gatt_deregister( );
    wiced_rtos_stop_timer( &tick_timer );

    return ERR_CMD_OK;
}

