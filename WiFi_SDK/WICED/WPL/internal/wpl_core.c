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
#include "wiced.h"
#include "wpl_packet.h"
#include "wpl_core.h"
#include "wpl_console.h"
#include "wpl_transport.h"
#include "wiced_low_power.h"
//#include "typedefs.h"
#include "wpl_cpl.h"
#include "platform_wpl.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MAX_CPL_LIST        10

//Request packet Macros
#define PACKET_SYNC_WORD_SIZE              2 //bytes
#define PACKET_CMD_SIZE                    1 //byte
#define PACKET_COUNT_SIZE                  1 //byte
#define PACKET_START_TIMESTAMP_SIZE        4 //bytes
#define PACKET_STOP_TIMESTAMP_SIZE         4 //bytes
#define PACKET_EACH_LOG_EVENT_SIZE         7 //bytes

#define PID            1 //byte //Processor ID size
#define EID            1 //byte // Event ID Size
#define EDID           1 //byte //Event Descriptor Size

//Response packet Macros
#define PACKET_RESP_ERROR                1
#define PACKET_RESP_SUCCESS              0

//Console command Macros
#define PACKET_CMD_OFFSET                1
#define PACKET_COUNT_OFFSET              2

#define POLL_TIME_MARGIN_PERCENTAGE             10

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
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t handle_target_detection_cmd( void );
static wiced_result_t handle_get_version_cmd( void );
static wiced_result_t handle_request_log_cmd( void );
static wiced_result_t handle_get_processor_list_cmd( void );
static wiced_result_t handle_get_events_cmd( wpl_packet_t* request_packet );
static wiced_result_t handle_get_event_descriptor_cmd( wpl_packet_t* request_packet );
static wiced_result_t handle_start_log_cmd( wpl_packet_t* request_packet );
static wiced_result_t handle_stop_log_cmd( wpl_packet_t* request_packet );
static wiced_result_t handle_log_poll_period_cmd( wpl_packet_t* request_packet );
static wiced_bool_t cpl_has_proc_id( cpl_data_t* cpl, uint8_t proc_id );


/******************************************************
 *                Function Declarations
 ******************************************************/
wiced_result_t wpl_transport_console_handler( wpl_packet_t** packet, int argc, char* argv[] );

/******************************************************
 *               Variable Declarations
 ******************************************************/
extern wiced_semaphore_t          wpl_stdio_uart_rx_semaphore;

/******************************************************
 *               Variable Definitions
 ******************************************************/
 //AON variables
static wiced_time_t WPL_DEEP_SLEEP_SAVED_VAR( log_start_timestamp );
static uint32_t     WPL_DEEP_SLEEP_SAVED_VAR( log_sent_time_offset );
static wpl_mode_t WPL_DEEP_SLEEP_SAVED_VAR( wplCommunicationMode );
uint16_t WPL_DEEP_SLEEP_SAVED_VAR( wpl_log_poll_time );


static cpl_data_t* cpl_list[MAX_CPL_LIST];
static uint8_t registered_cpl_count;
static uint8_t registered_processor_count;
static uint32_t max_power_log_size;
static wiced_bool_t wplStarted;
wiced_bool_t deep_sleep_enter = WICED_FALSE;
uint8_t wpl_console_print_status;

static wiced_mutex_t wpl_handle_request_lock;

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wpl_register_cpl( cpl_data_t* cpl )
{
    /* TODO: Check valid processor id and not existing already */

    cpl_list[registered_cpl_count] = cpl;
    registered_cpl_count++;
    /* Each CPL can have multple processor ids */
    registered_processor_count += cpl->proc_count;
    max_power_log_size += cpl->log_size;

    return WICED_SUCCESS;
}

wiced_result_t wpl_unregister_cpl( uint8_t processor_id )
{
    return WICED_SUCCESS;
}

wiced_result_t wpl_init( )
{
    wiced_rtos_init_semaphore( &wpl_stdio_uart_rx_semaphore );

    wiced_rtos_init_mutex( &wpl_handle_request_lock );

    if ( !WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        WPRINT_LIB_DEBUG( ( "COLDBOOT: Setting to default console mode\n" ) );
        wpl_set_mode( WPL_MODE_CONSOLE );
    }
    else
    {
        WPRINT_LIB_DEBUG( ( "WARMBOOT: mode: %d\n", wpl_get_mode( ) ) );
    }
    return WICED_SUCCESS;
}

void wpl_start( )
{
    /* Protect wpl start */
    if ( wplStarted )
        return;
    wpl_init( );
    wpl_cpl_init( );
    WPRINT_LIB_DEBUG( ( "registered_cpl_count : %d\n",registered_cpl_count ) );

#ifdef    WPRINT_ENABLE_LIB_DEBUG
    uint8_t i;
    for ( i = 0; i < registered_cpl_count; i++ )
    {
        uint8_t j = cpl_list[i]->proc_count;
        WPRINT_LIB_DEBUG( ( "cpl_list[%d]->proc_count : %d\n", i, cpl_list[i]->proc_count ) );
        /* Each CPL can have more than one Processors */
        for ( j = 0; j < cpl_list[i]->proc_count; j++ )
        {
            WPRINT_LIB_DEBUG( ( "cpl_list[%d]->proc_id[%d] : 0x%x\n", i, j, *( cpl_list[i]->proc_id+j ) ) );
        }
    }
#endif
    wpl_transport_init( UART_TRANSPORT, packets_from_wpl_host_handler );
    wplStarted = WICED_TRUE;
}

wiced_result_t packets_from_wpl_host_handler( wpl_packet_t* packet )
{
    /* Check if packets to be served by */
    wiced_result_t    result = WICED_SUCCESS;
    /* Route the packet to the proper handler */
    if ( deep_sleep_enter )
    {
        wpl_free_packet( packet );
        return WICED_SUCCESS;
    }

    wiced_rtos_lock_mutex( &wpl_handle_request_lock );

    switch ( packet->packet_id )
    {
        case CMD_TARGET_DETECTION:
        {
            result = handle_target_detection_cmd( );
            break;
        }
        case CMD_GET_TARGET_WPL_VERSION:
        {
            result = handle_get_version_cmd( );
            break;
        }
        case CMD_GET_PROCESSOR_LIST:
        {
            result = handle_get_processor_list_cmd( );
            break;
        }
        case CMD_LOG_REQUEST:
        {
            result = handle_request_log_cmd( );
            break;
        }
        case CMD_GET_EVENTS_LIST:
        {
            result = handle_get_events_cmd( packet );
            break;
        }
        case CMD_GET_EVENT_DESCRIPTOR_LIST:
        {
            result = handle_get_event_descriptor_cmd( packet );
            break;
        }
        case CMD_START_LOGGING:
        {
            result = handle_start_log_cmd( packet );
            break;
        }
        case CMD_STOP_LOGGING:
        {
            result = handle_stop_log_cmd( packet );
            break;
        }
        case CMD_LOG_POLL_PERIOD:
        {
            result = handle_log_poll_period_cmd( packet );
            break;
        }
        case CMD_STOP_PAD:
        {
            result = handle_stop_log_cmd( packet );
            wpl_set_mode( WPL_MODE_CONSOLE );
            break;
        }
        default:
            result =  WICED_UNSUPPORTED;
    }

    /* WPL packet to be freed here */
    wpl_free_packet( packet );

    wiced_rtos_unlock_mutex( &wpl_handle_request_lock );

    return result;
}

wiced_result_t handle_target_detection_cmd( void )
{
    wiced_result_t      result;
    wpl_packet_t*       out_packet;
    uint8_t                payload_len = strlen( PLATFORM );

    /* Create the wpl response packet. Add one for the length of the Platfor m name */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_TARGET_DETECTION, payload_len + PACKET_COUNT_SIZE );
    if ( result != WICED_SUCCESS )
        return result;
    memcpy( out_packet->payload_start, &payload_len, PACKET_COUNT_SIZE );
    memcpy( out_packet->payload_start + PACKET_COUNT_SIZE, PLATFORM, payload_len );
    wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

wiced_result_t handle_get_version_cmd( void )
{
    wiced_result_t      result;
    wpl_packet_t*       out_packet;
    uint8_t                payload_len = strlen( TARGET_WPL_VERSION );

    /* Create the wpl response packet. Add one for the length of the version string */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_TARGET_WPL_VERSION, payload_len + PACKET_COUNT_SIZE );
    if ( result != WICED_SUCCESS )
        return result;
    memcpy( out_packet->payload_start, &payload_len, PACKET_COUNT_SIZE );
    memcpy( out_packet->payload_start + PACKET_COUNT_SIZE, TARGET_WPL_VERSION, payload_len );
    wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

wiced_result_t handle_get_processor_list_cmd( void )
{
    wiced_result_t      result;
    wpl_packet_t*       out_packet;
    uint32_t            payload_len = registered_processor_count + PACKET_COUNT_SIZE; /* Added one for count */
    uint8_t             i;
    uint32_t            payload_offset=0;

    /* Create the wpl response packet */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_PROCESSOR_LIST, payload_len );
    if ( result != WICED_SUCCESS )
        return result;
    memcpy( out_packet->payload_start + payload_offset, &registered_processor_count, PACKET_COUNT_SIZE );
    payload_offset++;

    for ( i = 0; i < registered_cpl_count; i++ )
    {
        uint8_t j = 0;
        for ( j = 0; j < cpl_list[i]->proc_count; j++ )
        {
            memcpy( out_packet->payload_start + payload_offset, cpl_list[i]->proc_id + j, PACKET_COUNT_SIZE );
            payload_offset++;
        }
    }
    wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

static wiced_bool_t time_to_send_log_data(void)
{
    /* Log request can come much earlier than the registered log poll time. Serve it if it is under 10% margin */
    if ( ( wpl_get_time_stamp() - ( log_sent_time_offset + log_start_timestamp ) ) < ( wpl_log_poll_time - (wpl_log_poll_time/POLL_TIME_MARGIN_PERCENTAGE) ) )
    {
//        WPRINT_LIB_INFO( ( "time_to_send_log_data: Early to send, refuse\n") );
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

wiced_result_t handle_request_log_cmd( void )
{
    uint8_t i;
    uint32_t count=0;
    wiced_result_t   result;
    wpl_packet_t*    out_packet = NULL;
    uint32_t data_copied = 0;
    uint8_t     event_copied = 0;
    uint8_t* logstart_ptr =NULL;
    wiced_time_t current_time_stamp = 0;

    if ( !wplStarted )
        return WICED_NOTUP;

    //Wait for the poll time to maintain log consistency
    if ( !time_to_send_log_data() )
        return WICED_ABORTED;

    if ( !wpl_log_poll_time )
        return WICED_NOTUP;

    /* Create wpl response packet with log size + 2 timestamps + count of packets */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_LOG_REQUEST, ( max_power_log_size * PACKET_EACH_LOG_EVENT_SIZE + PACKET_START_TIMESTAMP_SIZE + PACKET_STOP_TIMESTAMP_SIZE + PACKET_COUNT_SIZE ) );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG( ( "handle_request_log_cmd: Packet allocation problem\n" ) );
        return result;
    }
    WPRINT_LIB_DEBUG( ( "max_power_log_size: %lu\n", max_power_log_size ) );

    /*Copy the start timestamp */
    memcpy( out_packet->payload_start, ( uint8_t* )&log_sent_time_offset, sizeof( log_sent_time_offset ) );
    WPRINT_LIB_DEBUG( ( "StartTimestamp: %lu\n", log_sent_time_offset ) );
    logstart_ptr = out_packet->payload_start + PACKET_START_TIMESTAMP_SIZE + PACKET_STOP_TIMESTAMP_SIZE + PACKET_COUNT_SIZE;

    for ( i = 0; i < registered_cpl_count; i++ )
    {
        if ( cpl_list[i]->cpl_callback )
        {
            /* Infor m CPL to send the event log data */
            result = cpl_list[i]->cpl_callback( CPL_SEND_LOG_REQUEST, &count, logstart_ptr+data_copied, NULL );
            if ( ( result != WICED_SUCCESS ) )
                continue;
            WPRINT_LIB_DEBUG( ( "count:%lu data_copied:%lu\n",count, data_copied ) );
            /* CPL would send the event log count pushed by it */
            if ( count )
            {
                data_copied += count*PACKET_EACH_LOG_EVENT_SIZE;
            }
        }
    }
    current_time_stamp = wpl_get_time_stamp( );
    log_sent_time_offset = current_time_stamp - log_start_timestamp;

    /*Copy the end timestamp */
    memcpy( out_packet->payload_start+ PACKET_STOP_TIMESTAMP_SIZE, ( uint8_t* )&log_sent_time_offset, sizeof( log_sent_time_offset ) );
    WPRINT_LIB_DEBUG( ( "StopTimestamp: %lu\n", log_sent_time_offset ) );

    /* Copy the number of events */
    event_copied = data_copied / PACKET_EACH_LOG_EVENT_SIZE;
    memcpy( out_packet->payload_start+PACKET_START_TIMESTAMP_SIZE + PACKET_STOP_TIMESTAMP_SIZE, ( uint8_t* )&event_copied, sizeof( event_copied ) );
    WPRINT_LIB_DEBUG( ( "Events: %d\n", event_copied ) );
    if ( event_copied )
        wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

wiced_result_t handle_get_events_cmd( wpl_packet_t* request_packet )
{
    uint8_t i;
    wiced_result_t    result = WICED_SUCCESS;
    wpl_packet_t*    out_packet = NULL;
    uint32_t count = PID;
    uint8_t proc_id = *( request_packet->payload_start );
    uint8_t* out_data;

    for ( i = 0; i < registered_cpl_count; i++ )
    {
        uint8_t j = 0;

        /* Check if this CPL manages the proc_id in request */
        for ( j = 0; j < cpl_list[i]->proc_count; j++ )
        {
            uint8_t cpl_proc = cpl_list[i]->proc_id[j];
            if ( proc_id != cpl_proc )
                continue;

            /* Get the list of events from CPL */
            result = cpl_list[i]->cpl_callback( CPL_GET_EVENTS_LIST, &count, &cpl_proc, &out_data );

            if ( result != WICED_SUCCESS )
                return result;
            /* CPL would give the number of events and list of events */
            result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_EVENTS_LIST, count + PID + EID );
            if ( result != WICED_SUCCESS )
                return result;
            /* Copy back the PID in response */
            memcpy( out_packet->payload_start, &cpl_list[i]->proc_id[j], PID );//TBD &cpl_proc gives zero always so changed
            //memcpy( out_packet->payload_start, &cpl_proc, 1 );//TBD &cpl_proc gives zero always so changed
            /* Copy count of events  */
            memcpy( out_packet->payload_start + PID, &count, PACKET_COUNT_SIZE );

            /* Copy the list of events  */
            memcpy( out_packet->payload_start + PID + EID, out_data, count );

            /* Send the packet to PAD. The packet will be freed by the transport */
            wpl_transport_send_packet( out_packet );
            return WICED_SUCCESS;
        }
    }
    /* No event list found, send empty response ( Processor Id and 0 count */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_EVENTS_LIST, PID + EID );
    if ( result != WICED_SUCCESS )
        return result;
    memcpy( out_packet->payload_start, &proc_id, PID );
    count = 0;
    memcpy( out_packet->payload_start + PACKET_COUNT_SIZE, &count, PACKET_COUNT_SIZE );
    wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

wiced_result_t handle_get_event_descriptor_cmd( wpl_packet_t* request_packet )
{

    uint8_t i;
    wiced_result_t    result = WICED_SUCCESS;
    wpl_packet_t*    out_packet = NULL;
    uint32_t count = PID + EID;
    uint8_t proc_id = *( request_packet->payload_start );
    uint8_t* out_data;

    for ( i = 0; i < registered_cpl_count; i++ )
    {
        uint8_t j = 0;

        /* Check if this CPL manages the proc_id in request */
        for ( j = 0; j < cpl_list[i]->proc_count; j++ )
        {
            uint8_t cpl_proc = cpl_list[i]->proc_id[j];

            if ( proc_id != cpl_proc )
                continue;

            /* Get the list of events descriptors from CPL */
            result = cpl_list[i]->cpl_callback( CPL_GET_EVENTS_DESC_LIST, &count, request_packet->payload_start, &out_data );
            if ( result != WICED_SUCCESS )
                return result;

            /* CPL would give the number of events descriptors and list */
            result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_EVENT_DESCRIPTOR_LIST, count + PID + EID + EDID );
            if ( result != WICED_SUCCESS )
                return result;
            /* Copy back the PID and Event Id in response */
            memcpy( out_packet->payload_start, request_packet->payload_start, PID + EID );
            /* Copy count of events  */
            memcpy( out_packet->payload_start + PID + EID, &count, PACKET_COUNT_SIZE );

            /* Copy the list of events  */
            memcpy( out_packet->payload_start + PID + EID + EDID, out_data, count );

            /* Send the packet to PAD. The packet will be freed by the transport */
            wpl_transport_send_packet( out_packet );
            return WICED_SUCCESS;
        }
    }
    /* No event list found, send empty response ( Processor Id and 0 count */
    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_GET_EVENT_DESCRIPTOR_LIST, PID + EID + EDID );
    if ( result != WICED_SUCCESS )
        return result;
    memcpy( out_packet->payload_start, request_packet->payload_start, PID + EID );
    count = 0;
    memcpy( out_packet->payload_start + PID + EID, &count, PACKET_COUNT_SIZE );
    wpl_transport_send_packet( out_packet );
    return WICED_SUCCESS;
}

wiced_bool_t cpl_has_proc_id( cpl_data_t* cpl, uint8_t proc_id )
{
    uint8_t i = 0;
    for ( i = 0; i < cpl->proc_count; i++ )
    {
        /* Check the proc id in the list of CPL's proc_id */
        if ( *( cpl->proc_id+i ) == proc_id )
            return WICED_TRUE;
    }
    return WICED_FALSE;
}

wiced_result_t handle_start_log_cmd( wpl_packet_t* request_packet )
{
    uint8_t i;
    wiced_result_t    result = WICED_SUCCESS;
    wpl_packet_t*    out_packet = NULL;
    uint8_t count = *( request_packet->payload_start );
    WPRINT_LIB_DEBUG( ( "WPL: START command\n" ) );

    //Already in start state, nothing to do
    if ( log_start_timestamp )
        return WICED_SUCCESS;

    /* Reset the timestamp if logging is not already going on*/
    log_start_timestamp = wpl_get_time_stamp( );
    log_sent_time_offset = 0;

    if ( count == 0 )
    {
        /* Start logging all the events of all the proc id */
        for ( i = 0; i < registered_cpl_count; i++ )
        {
            result = cpl_list[i]->cpl_callback( CPL_START_LOG, NULL, NULL, NULL );
            if ( result != WICED_SUCCESS )
                goto startlog_error;
        }
    }
    else
    {
        uint8_t j =0;
        /* Start logging selected events of selected proc id */
        for ( j = 0; j < count; j++ )
        {
            /* Get Proc Id */
            uint8_t* pid_loc = ( request_packet->payload_start + PACKET_COUNT_SIZE + j * ( PID + EID ) );
            wiced_bool_t pid_found = WICED_FALSE;
            for ( i = 0; i < registered_cpl_count; i++ )
            {
                if ( cpl_has_proc_id( cpl_list[i], *pid_loc ) )
                {
                    /* CPL supports this proc id, send the start command with proc_id+event id location*/
                    result = cpl_list[i]->cpl_callback( CPL_START_LOG, NULL, pid_loc, NULL );
                    if ( result != WICED_SUCCESS )
                        goto startlog_error;
                    pid_found = WICED_TRUE;
                }
            }
            if ( !pid_found )
            {
                /* Send error if any of the sent PIDs not found */
                result = WICED_ERROR;
                goto startlog_error;
            }
        }
    }

startlog_error:
    if ( result != WICED_SUCCESS )
        count = PACKET_RESP_ERROR;  // Send Error
    else
        count = PACKET_RESP_SUCCESS;

    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_START_LOGGING, PACKET_COUNT_SIZE );
    if ( result != WICED_SUCCESS )
        return result;

    memcpy( out_packet->payload_start, &count, PACKET_COUNT_SIZE );
    wpl_transport_send_packet( out_packet );

    return result;
}

wiced_result_t handle_log_poll_period_cmd( wpl_packet_t* request_packet )
{
    wiced_result_t    result = WICED_SUCCESS;
    wpl_packet_t*    out_packet = NULL;
    uint8_t poll_period = *( request_packet->payload_start );
    uint8_t count;
    WPRINT_LIB_DEBUG( ( "WPL: START command\n" ) );

    wpl_log_poll_time = poll_period * WPL_POLL_MULTIPLY_FACTOR;

    count = PACKET_RESP_SUCCESS;

    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_LOG_POLL_PERIOD, PACKET_COUNT_SIZE );
    if ( result != WICED_SUCCESS )
        return result;

    memcpy( out_packet->payload_start, &count, PACKET_COUNT_SIZE );
    wpl_transport_send_packet( out_packet );

    return result;
}

wiced_result_t handle_stop_log_cmd( wpl_packet_t* request_packet )
{
    uint8_t i;
    wiced_result_t    result = WICED_SUCCESS;
    wpl_packet_t*    out_packet = NULL;
    uint8_t count = *( request_packet->payload_start );
    WPRINT_LIB_DEBUG( ( "WPL: STOP command\n" ) );
    if ( count == 0 )
    {
        /* Stop logging all the events of all the proc id */
        for ( i=0; i < registered_cpl_count; i++ )
            cpl_list[i]->cpl_callback( CPL_STOP_LOG, NULL, NULL, NULL );
    }
    else
    {
        uint8_t j = 0;
        /* Stop logging selected events of selected proc id */
        for ( j=0; j < count; j++ )
        {
            /* Get Proc Id */
            uint8_t* pid_loc = ( request_packet->payload_start +  PACKET_COUNT_SIZE + j * ( PID + EID ) );
             wiced_bool_t pid_found = WICED_FALSE;
            for ( i = 0; i < registered_cpl_count; i++ )
            {
                if ( cpl_has_proc_id( cpl_list[i], *pid_loc ) )
                {
                    /* CPL supports this proc id, send the STOP command with event id location*/
                    result = cpl_list[i]->cpl_callback( CPL_STOP_LOG, NULL, pid_loc, NULL );
                    if ( result != WICED_SUCCESS )
                        goto stoplog_error;
                    pid_found = WICED_TRUE;
                }
            }
            if ( !pid_found )
            {
                /* Send error if any of the sent PIDs not found */
                result = WICED_ERROR;
                goto stoplog_error;
            }
        }
        /* TODO: Check with CPLs if all the logging has stopped */
    }

stoplog_error:
    if ( result != WICED_SUCCESS )
        count = PACKET_RESP_ERROR;  // Send Error
    else
        count = PACKET_RESP_SUCCESS;

    result = wpl_dynamic_allocate_packet( &out_packet, WPL_PAD_RESPONSE, CMD_STOP_LOGGING, PACKET_COUNT_SIZE );
    if ( result != WICED_SUCCESS )
        return result;

    memcpy( out_packet->payload_start, &count, PACKET_COUNT_SIZE );
    wpl_transport_send_packet( out_packet );
    log_start_timestamp = 0;
    return result;
}

void wpl_set_mode( wpl_mode_t mode )
{
    wplCommunicationMode = mode;
    return;
}

wpl_mode_t wpl_get_mode( void )
{
    return wplCommunicationMode ;
}

uint8_t wpl_get_console_prints_status( void )
{
    return wpl_console_print_status;
}

void wpl_process_enable_console_prints( int argc, char* argv[] )
{
    uint8_t enable;
    enable = atoi( argv[1] );

    if ( enable )
        wpl_console_print_status = 1;
    else
        wpl_console_print_status = 0;
}

void console_print_version( void )
{
    WPRINT_LIB_INFO( ( "%s\n", TARGET_WPL_VERSION ) );
    return;
}

void console_print_target( void )
{
    WPRINT_LIB_INFO( ( "%s\n", PLATFORM ) );
    return;
}

wiced_result_t wpl_process_console_command( int argc, char* argv[] )
{
    wpl_packet_t* packet = NULL;
    if ( wpl_get_mode( ) != WPL_MODE_CONSOLE )
    {
        return WICED_ERROR;
    }
    wpl_transport_console_handler( &packet, argc, argv );

    if ( packet )
       packets_from_wpl_host_handler( packet );
    return WICED_SUCCESS;
}

wiced_result_t wpl_transport_console_handler( wpl_packet_t** packet, int argc, char* argv[] )
{
    uint8_t packet_byte=0;
    wiced_result_t    result;
    uint8_t payload_len =  0;

    /* Get the packet type */
    packet_byte = atoi( argv[PACKET_CMD_OFFSET] );

    switch ( packet_byte )
    {
        case CMD_TARGET_DETECTION:
        case CMD_GET_TARGET_WPL_VERSION:
        case CMD_GET_PROCESSOR_LIST:
        case CMD_LOG_REQUEST:
        {
            payload_len = 0;
            break;
        }
        case CMD_GET_EVENTS_LIST:
        {
            payload_len = PID;
            break;
        }
        case CMD_GET_EVENT_DESCRIPTOR_LIST:
        {
            payload_len = PID + EID;
            break;
        }
        case CMD_START_LOGGING:
        case CMD_STOP_LOGGING:
        {
            payload_len = argc - PACKET_COUNT_OFFSET; // Variable number of arguments
            break;
        }
        case CMD_LOG_POLL_PERIOD:
            payload_len = PACKET_COUNT_SIZE;
            break;
        default:
            return WICED_UNSUPPORTED;
    }

    /* Check if number of arguments is proper */
    if ( ( argc - PACKET_COUNT_OFFSET ) != ( payload_len ) )
    {
        WPRINT_LIB_INFO( ( "Argument error\n" ) );
        return WICED_ERROR;
    }

    result = wpl_dynamic_allocate_packet( packet, WPL_PAD_REQUEST, packet_byte, payload_len );
    if ( result != WICED_SUCCESS )
        return result;

    /* Retrieve the payload data */
    if ( payload_len )
    {
        uint8_t i = 0;
        for ( i = 0; i < payload_len; i++ )
        {
            uint8_t byte = atoi( argv[i + PACKET_COUNT_OFFSET] );
            memcpy( ( *packet )->payload_start+i, &byte, 1 );
        }
    }
    return WICED_SUCCESS;
}

wiced_result_t wpl_send_pad_cmd( uint8_t packet_byte )
{
    wiced_result_t      result = WICED_SUCCESS;
    uint32_t payload_len =    0;
    wpl_packet_t* packet;

    if(!wpl_logging_status())
        return WICED_NOTUP;

    /* Check if it is time to send the log request */
    if ( ( packet_byte==CMD_LOG_REQUEST ) && ( !time_to_send_log_data() ) )
        return WICED_ABORTED;

    result = wpl_dynamic_allocate_packet( &packet, WPL_PAD_REQUEST, packet_byte, payload_len );
    if ( result != WICED_SUCCESS ) {
           return WICED_ERROR;
      }

    result = packets_from_wpl_host_handler( packet );

    return result;
}

wiced_bool_t wpl_logging_status( void )
{
    return  log_start_timestamp ? WICED_TRUE : WICED_FALSE;
}

void wpl_set_deep_sleep_state( wiced_bool_t state )
{
    deep_sleep_enter = state;
    if(state == WICED_TRUE)
        wpl_transport_deinit();
}
