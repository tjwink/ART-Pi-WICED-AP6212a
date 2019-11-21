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

/** @file ble_gatt_db.c
 *
 * BLE GATT DB Application
 *
 * This application demonstrates how we can add entries to GATT database dynamically.
 * Once the entries are made into the GATT_DB, the device behaves and as a GATT Server.
 *
 * Features demonstrated
 *  - GATT database dynamic initialization
 *  - Registration with LE stack for various events
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer
 * 2. Build and download the application (to the WICED board)
 * 3. On application start the device boots up and waits for the user to add details into the GATT database.
 *    * Command Line support is enabled to make entries into the GATT DB.
 *    * To get the list of commands supported by this app, type help.
 * 4. Usage of the Commands supported and the syntax:
 *    ***) To add a Primary/Secondary/Included Service:
 *          add_service [service_type] [handle] [UUID]
 *              * service_type: primary/secondary/included
 *              * handle: 2 byte value
 *              * UUID: 16bit value/128bit value
 *
 *    ***) To add a Characteristic:
 *          add_charactecteristic [handle] [UUID] [handle_value] [property] [permission]
 *              * handle: 2 byte value
 *              * UUID: 16bit value/128bit value
 *              * handle_value: 2 byte value
 *              * property: 1 byte value
 *              * permission: 1 byte value
 *
 *    ***) To add a Descriptor:
 *          add_descriptor [handle] [UUID] [permission]
 *              * handle: 2 byte value
 *              * UUID: 16bit value/128bit value
 *              * permission: 1 byte value
 *
 *    ***) To start/stop Advertisements:
 *          advertise [value]
 *              * value: 0- adv off; 1-adv start
 *
 *  NOTE: While entering the 128 bit UUID, the UUID must be split into 2 bytes each followed by a space and entered in the reverse order.
 *            e.g. If AE5D1E47-5C13-43A0-8635-82AD38A1381F is the 128 bit UUID,
 *            is should be entered as 1f38 a138 ad82 3586 a043 135c 471e 5dae.
 * 5. Once the GATT DB is successfully entered, advertisements can be started.
 * 6. On advertisement start the device acts as a GATT server and advertises itself as GATT_DB
 * 7. Connect to GATT server using one of the LE clients (LEExplorer(android)) or (BLE Utility(Apple Store))
 *
 * Here are few examples on how to add the data into the GATT_DB:
 *      ** Add a primary service and characteristic with UUID 16 bit.
 *      e.g. *) add_service primary 01 1801         (GATT Service)
 *           *) add_service primary 014 1800        (GAP Service)
 *               *) add_characteristic 15 2A00 16 02 02
 *               *) add_characteristic 17 2A01 18 02 02
 *      ** Add a primary service and characteristic with UUID 128 bit
 *      e.g. *) add_service primary 28 2320 567c 05cf 6eb4 c341 7728 5182 7e1b
 *               *) add_characteristic 29 26f6 6991 68ee c2be 444d b95c 3f2d c38a 2A 10 02
 */

#include <string.h>
#include "wiced_bt_gatt_db_helper.h"
#include "wiced_bt_dev.h"
#include "bt_target.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced.h"
#include "wiced_bt_stack.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "command_console.h"
#include "wwd_debug.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"

/******************************************************************************
 *                           External Definitions
******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/
static wiced_result_t         gatt_db_app_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   gatt_db_application_init( void );
static wiced_bt_gatt_status_t gatt_db_app_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   gatt_db_app_set_advertisement_data(void);
static wiced_bt_gatt_status_t gatt_db_app_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t gatt_db_app_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t gatt_db_app_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t gatt_db_app_attribute_request_handler( wiced_bt_gatt_attribute_request_t *p_data );

/* Console Command Handling */
static int                    gatt_db_app_add_service                        ( int argc, char *argv[] );
static int                    gatt_db_app_add_characteristic                 ( int argc, char *argv[] );
static int                    gatt_db_app_add_descriptor                     ( int argc, char *argv[] );
static int                    gatt_db_app_start_stop_advertisments           ( int argc, char *argv[] );

/******************************************************************************
 *                             Macro Definitions
******************************************************************************/
#define GATT_DB_APP_CONSOLE_COMMAND_HISTORY_LENGTH  (10)
#define MAX_GATT_DB_APP_COMMAND_LENGTH              (115)
#define GATT_DB_APP_CONSOLE_COMMANDS \
    { (char*) "add_service",              gatt_db_app_add_service,          0, NULL, NULL, (char *)"[service_type] [handle] [UUID16/UUD32]",                                (char *)"Add Service -- Primary/Secondary/Included" }, \
    { (char*) "add_characteristic",       gatt_db_app_add_characteristic,   0, NULL, NULL, (char *)"[handle] [UUID] [handle_value] [property] [permission]", (char *)"Add Characteristic" }, \
    { (char*) "add_descriptor",           gatt_db_app_add_descriptor,       0, NULL, NULL, (char *)"[handle] [UUID] [permission]", (char *)"Add Descriptor" }, \
    { (char*) "advertise",                gatt_db_app_start_stop_advertisments,  0, NULL, NULL, (char *)"[value]", (char *)"Start or Stop Advertisements" }, \

/******************************************************************************
 *                             Constants
******************************************************************************/
const command_t gatt_db_app_console_command_table[] =
{
    GATT_DB_APP_CONSOLE_COMMANDS
    CMD_TABLE_END
};

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                         Variables Definitions
 ******************************************************************************/
static char                  gatt_db_app_command_buffer[MAX_GATT_DB_APP_COMMAND_LENGTH];
static char                  gatt_db_app_command_history_buffer[MAX_GATT_DB_APP_COMMAND_LENGTH * GATT_DB_APP_CONSOLE_COMMAND_HISTORY_LENGTH];

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    wiced_core_init();
    WPRINT_BT_APP_INFO( ( "GATT DB Application start\n" ) );

    /* Register call back and configuration with stack */
    wiced_bt_stack_init( gatt_db_app_management_callback ,
                    &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void gatt_db_application_init( void )
{
    wiced_bt_gatt_status_t  gatt_status;

    WPRINT_BT_APP_INFO( ( "hello_sensor_application_init\n" ) );

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( gatt_db_app_gatts_callback );
    WPRINT_BT_APP_INFO(( "\n wiced_bt_gatt_register: %d\n", gatt_status ));

    /* Set the advertising parameters */
    gatt_db_app_set_advertisement_data();
    /* Start the advertisements only after gatt db is registered, wait until then */
}

static void gatt_db_app_set_advertisement_data(void)
{
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_bt_cfg_settings.device_name);
    WPRINT_BT_APP_INFO( ("wiced_bt_cfg_settings.device_name:%s\n", wiced_bt_cfg_settings.device_name));
    adv_elem[num_elem].p_data       = (uint8_t *)wiced_bt_cfg_settings.device_name;

    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    WPRINT_BT_APP_INFO( ( "wiced_bt_ble_set_advertisement_data %d\n", result ) );
}

/*
 * hello_sensor bt/ble link management callback
 */
static wiced_result_t gatt_db_app_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                   result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t* p_info;
    wiced_bt_ble_advert_mode_t*      p_mode;

    WPRINT_BT_APP_INFO(("gatt_db_app_management_callback: %x\n", event ));

    switch( event )
    {
    /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
        {
            gatt_db_application_init( );

            /* Initialize command console */
            result = command_console_init(STDIO_UART, sizeof(gatt_db_app_command_buffer), gatt_db_app_command_buffer,
                    GATT_DB_APP_CONSOLE_COMMAND_HISTORY_LENGTH, gatt_db_app_command_history_buffer, " ");
            if (result != WICED_SUCCESS)
            {
                WPRINT_BT_APP_INFO(("Error starting the command console\r\n"));
            }
            console_add_cmd_table( gatt_db_app_console_command_table );
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

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are ommitted.
 */
static wiced_bt_gatt_status_t gatt_db_app_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = gatt_db_app_connection_status_handler( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = gatt_db_app_attribute_request_handler( &p_data->attribute_request );
        break;

    default:
        break;
    }

    return result;
}

/*
 * Connection up/down event
 */
static wiced_bt_gatt_status_t gatt_db_app_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return gatt_db_app_connection_up( p_status );
    }

    return gatt_db_app_connection_down( p_status );
}

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t gatt_db_app_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result = WICED_BT_GATT_SUCCESS;

    WPRINT_BT_APP_INFO( ( "gatt_db_conn_up  id:%d\n:", p_status->conn_id) );

    return result;
}


/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t gatt_db_app_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WPRINT_BT_APP_INFO( ( "connection_down  conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason ) );

    return WICED_BT_SUCCESS;
}


/*
 * Process GATT request from the peer
 */
static wiced_bt_gatt_status_t gatt_db_app_attribute_request_handler( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WPRINT_BT_APP_INFO(( "gatt_db_app_attribute_request_handler. conn %d, type %d\n", p_data->conn_id, p_data->request_type ));

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        WPRINT_APP_INFO(("GATT READ REQUEST\n"));
        break;

    case GATTS_REQ_TYPE_WRITE:
        WPRINT_APP_INFO(("GATT WRITE REQUEST\n"));
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        WPRINT_APP_INFO(("GATT WRITE EXEC REQUEST\n"));
        break;

    case GATTS_REQ_TYPE_MTU:
        WPRINT_APP_INFO(("GATT REQUEST TYPE MTU\n"));
        break;

    case GATTS_REQ_TYPE_CONF:
        WPRINT_APP_INFO(("GATT TYPE CONF\n"));
        break;

   default:
        break;
    }
    return result;
}

static int gatt_db_app_add_service ( int argc, char *argv[] )
{
    uint16_t handle;
    wiced_bt_uuid_t uuid;
    char *command1 = argv[1];
    uint8_t length = 0;
    uint16_t        service_handle;
    uint16_t        end_group_handle;
    int i,j;
    uint16_t temp;

    if( (strcmp( command1,"primary" ) == 0) || (strcmp( command1,"secondary" ) == 0) )
    {
        if( argc == 4 )
        {
            /* Fill the uuid length and uuid */
            WPRINT_APP_INFO((" [%s]service: uuid16 bit\n",command1));
            handle = strtol(argv[2],NULL,16);

            length = strlen(argv[3]);
            if( length == 4 )
            {
                uuid.uu.uuid16 = strtol(argv[3],NULL,16);
                uuid.len = LEN_UUID_16;
            }
            else
            {
                WPRINT_APP_INFO(("Format not correct: UUID16 should be entered in xxxx format\n"));
                return ERR_BAD_ARG;
            }

            if( strcmp( command1,"primary" ) == 0 )
            {
                if (wiced_bt_gatt_db_primary_service_add(handle, &uuid) == WICED_FALSE)
                {
                    return ERR_OUT_OF_HEAP;
                }
            }
            else if(strcmp( command1,"secondary" ) == 0)
            {
                if (wiced_bt_gatt_db_secondary_service_add(handle, &uuid) == WICED_FALSE)
                {
                    return ERR_OUT_OF_HEAP;
                }
            }
            return ERR_CMD_OK;
        }// End of Primary and Secondary Service 16 bit parsing

        if( argc == 11 )
        {
            WPRINT_APP_INFO((" [%s]service: uuid128 bit\n",command1));
            handle = strtol(argv[2],NULL,16);

            length = strlen(argv[3]) + strlen(argv[4]) + strlen(argv[5]) + strlen(argv[6]) + strlen(argv[7]) + strlen(argv[8]) + strlen(argv[9]) + strlen(argv[10]);
            if( length == 32 )
            {

                for( i=0,j=3; (i<LEN_UUID_128 && j<11); i++,j++ )
                {
                    temp = strtol(argv[j],NULL,16);
                    uuid.uu.uuid128[i] = ((temp & 0xff00) >> 8);
                    i++;
                    uuid.uu.uuid128[i] = (temp & 0x00ff);
                }
                uuid.len = LEN_UUID_128;
            }
            else
            {
                WPRINT_APP_INFO(("Format not correct: UUID128 should be entered in xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx format\n"));
                return ERR_BAD_ARG;
            }

            if( strcmp( command1,"primary" ) == 0 )
            {
                if (wiced_bt_gatt_db_primary_service_add(handle, &uuid) == WICED_FALSE)
                {
                    WPRINT_APP_INFO(("Failure\n"));
                    return ERR_OUT_OF_HEAP;
                }
            }
            else if(strcmp( command1,"secondary" ) == 0)
            {
                if (wiced_bt_gatt_db_secondary_service_add(handle, &uuid) == WICED_FALSE)
                {
                    WPRINT_APP_INFO(("Failure\n"));
                    return ERR_OUT_OF_HEAP;
                }
            }
            return ERR_CMD_OK;
        }// End of Primary and Secondary Service 128 bit parsing
        else
        {
            return ERR_INSUFFICENT_ARGS;
        }
    } // End of Check for Primary and Secondary Service
    else if(strcmp( command1,"included" ) == 0)
    {
        if( argc == 6 )
        {
            WPRINT_APP_INFO((" [%s]service: uuid16 bit\n",command1));
            handle = strtol(argv[2],NULL,16);

            length = strlen(argv[3]);
            if( length == 4 )
            {
                uuid.uu.uuid16 = strtol(argv[3],NULL,16);
                uuid.len = LEN_UUID_16;
            }
            else
            {
                WPRINT_APP_INFO(("Format not correct: UUID16 should be entered in xxxx format\n"));
                return ERR_BAD_ARG;
            }
            service_handle = strtol( argv[4],NULL,16 );
            end_group_handle = strtol( argv[5],NULL,16 );


            if (wiced_bt_gatt_db_included_service_add(handle, service_handle, end_group_handle, &uuid) == WICED_FALSE)
            {
                return ERR_OUT_OF_HEAP;
            }
            return ERR_CMD_OK;
        }// End of 16 bit UUID parsing for Included Service

        if( argc == 13 )
        {
            WPRINT_APP_INFO((" [%s]service: uuid128 bit\n",command1));
            handle = strtol(argv[2],NULL,16);

            length = strlen(argv[3]) + strlen(argv[4]) + strlen(argv[5]) + strlen(argv[6]) + strlen(argv[7]) + strlen(argv[8]) + strlen(argv[9]) + strlen(argv[10]);
            if( length == 32 )
            {

                for( i=0,j=3; (i<LEN_UUID_128 && j<11); i++,j++ )
                {
                    temp = strtol(argv[j],NULL,16);
                    uuid.uu.uuid128[i] = ((temp & 0xff00) >> 8);
                    i++;
                    uuid.uu.uuid128[i] = (temp & 0x00ff);
                }
                uuid.len = LEN_UUID_128;
            }
            else
            {
                WPRINT_APP_INFO(("Format not correct: UUID128 should be entered in xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx format\n"));
                return ERR_BAD_ARG;
            }
            service_handle = strtol( argv[11],NULL,16 );
            end_group_handle = strtol( argv[12],NULL,16 );

            if (wiced_bt_gatt_db_included_service_add(handle, service_handle, end_group_handle, &uuid) == WICED_FALSE)
            {
                return ERR_OUT_OF_HEAP;
            }
            return ERR_CMD_OK;
        }// End of Included Service 16 and 128 bit UUID parsing
        else
        {
            return ERR_INSUFFICENT_ARGS;
        }
    }// End of Included Service Parsing

    return ERR_BAD_ARG;

}

static int gatt_db_app_add_characteristic( int argc, char *argv[] )
{
    uint16_t handle;
    uint16_t handle_value;
    uint8_t  property;
    uint8_t  permission;
    wiced_bt_uuid_t uuid;
    uint8_t length = 0;
    int i,j;
    uint16_t temp;

    if( argc == 6 )
    {
        WPRINT_APP_INFO((" characteristic-uuid16 bit\n"));
        handle = strtol(argv[1],NULL,16);

        length = strlen(argv[2]);
        if(length == 4)
        {
            uuid.uu.uuid16 = strtol(argv[2],NULL,16);
            uuid.len = LEN_UUID_16;
        }
        else
        {
            WPRINT_APP_INFO(("Format not correct: UUID16 should be entered in xxxx format\n"));
            return ERR_BAD_ARG;
        }

        handle_value = strtol(argv[3],NULL,16);
        property = strtol(argv[4],NULL,16);
        permission = strtol(argv[5],NULL,16);


        if (wiced_bt_gatt_db_characteristic_add(handle, handle_value, &uuid, property, permission) == WICED_FALSE )
        {
            return ERR_OUT_OF_HEAP;
        }

        return ERR_CMD_OK;
    }

    if( argc == 13 )
    {
        WPRINT_APP_INFO((" characteristic-uuid128 bit\n"));
        handle = strtol(argv[1],NULL,16);

        length = strlen(argv[2]) + strlen(argv[3]) + strlen(argv[4]) + strlen(argv[5]) + strlen(argv[6]) + strlen(argv[7]) + strlen(argv[8]) + strlen(argv[9]);
        if( length == 32 )
        {

            for( i=0,j=2; (i<LEN_UUID_128 && j<11); i++,j++ )
            {
                temp = strtol(argv[j],NULL,16);
                uuid.uu.uuid128[i] = ((temp & 0xff00) >> 8);
                i++;
                uuid.uu.uuid128[i] = (temp & 0x00ff);
            }
            uuid.len = LEN_UUID_128;
        }
        else
        {
            WPRINT_APP_INFO(("Format not correct: UUID128 should be entered in xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx format\n"));
            return ERR_BAD_ARG;
        }

        handle_value = strtol(argv[10],NULL,16);
        property = strtol(argv[11],NULL,16);
        permission = strtol(argv[12],NULL,16);


        if (wiced_bt_gatt_db_characteristic_add(handle, handle_value, &uuid, property, permission) == WICED_FALSE )
        {
            return ERR_OUT_OF_HEAP;
        }
        return ERR_CMD_OK;
    }

    return ERR_BAD_ARG;
}

static int gatt_db_app_add_descriptor ( int argc, char *argv[] )
{
    uint16_t handle;
    uint8_t  permission;
    wiced_bt_uuid_t uuid;
    uint8_t length = 0;
    int i,j;
    uint16_t temp;

    if( argc == 4 )
    {
        WPRINT_APP_INFO((" descriptor-uuid16 bit\n"));
        handle = strtol(argv[1],NULL,16);

        length = strlen(argv[2]);
        if(length == 4)
        {
            uuid.uu.uuid16 = strtol(argv[2],NULL,16);
            uuid.len = LEN_UUID_16;
        }
        else
        {
            WPRINT_APP_INFO(("Format not correct: UUID16 should be entered in xxxx format\n"));
            return ERR_BAD_ARG;
        }

        permission = strtol(argv[3],NULL,16);

        if (wiced_bt_gatt_db_descriptor_add(handle, &uuid, permission) == WICED_FALSE )
        {
            return ERR_OUT_OF_HEAP;
        }

        return ERR_CMD_OK;
    }

    if( argc == 11 )
    {
        WPRINT_APP_INFO((" descriptor-uuid128 bit\n"));
        handle = strtol(argv[1],NULL,16);

        length = strlen(argv[3]) + strlen(argv[4]) + strlen(argv[5]) + strlen(argv[6]) + strlen(argv[7]) + strlen(argv[8]) + strlen(argv[9]) + strlen(argv[10]);

        if( length == 32 )
        {

            for( i=0,j=3; (i<LEN_UUID_128 && j<11); i++,j++ )
            {
                temp = strtol(argv[j],NULL,16);
                uuid.uu.uuid128[i] = ((temp & 0xff00) >> 8);
                i++;
                uuid.uu.uuid128[i] = (temp & 0x00ff);
            }
            uuid.len = LEN_UUID_128;
        }
        else
        {
            WPRINT_APP_INFO(("Format not correct: UUID128 should be entered in xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx format\n"));
            return ERR_BAD_ARG;
        }

        permission = strtol(argv[11],NULL,16);

        if (wiced_bt_gatt_db_descriptor_add(handle, &uuid, permission) == WICED_FALSE )
        {
            return ERR_OUT_OF_HEAP;
        }

        return ERR_CMD_OK;
    }

    return ERR_BAD_ARG;
}

static int gatt_db_app_start_stop_advertisments ( int argc, char *argv[] )
{
    uint8_t enable;
    if( argc != 2 )
    {
        return ERR_INSUFFICENT_ARGS;
    }
    else
    {
        enable = atoi(argv[1]);
        if(enable == 1)
        {
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        }
        else
        {
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
        }
        return ERR_CMD_OK;
    }
}
