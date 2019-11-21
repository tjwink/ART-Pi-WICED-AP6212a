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
 *  Connection functions
 *
 *  Provides connection methods for use by applications
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_connection.h"
#include "amqp_manager.h"
#include "amqp_open.h"
#include "amqp_sasl.h"
/******************************************************
 *                      Macros
 ******************************************************/

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_amqp_init( const wiced_ip_address_t *address, wiced_interface_t interface, const wiced_amqp_callbacks_t *callbacks, wiced_amqp_connection_instance *conn_instance, const wiced_amqp_security_t *security )
{
    uint16_t port_number = security == NULL ? AMQP_CONNECTION_DEFAULT_PORT : AMQP_CONNECTION_SECURITY_PORT;

    wiced_rtos_init_semaphore( &conn_instance->conn->semaphore );
    wiced_rtos_set_semaphore( &conn_instance->conn->semaphore );
    conn_instance->conn->callbacks = *callbacks;

    return amqp_network_init( address, port_number, interface, conn_instance, &conn_instance->conn->socket, security );
}

wiced_result_t wiced_amqp_deinit( wiced_amqp_connection_t *conn )
{
    wiced_rtos_deinit_semaphore( &conn->semaphore );
    return amqp_network_deinit( &conn->socket );
}

/******************************************************
 *      Backend functions called from amqp_queue
 ******************************************************/
wiced_result_t amqp_connection_backend_put_protocol_header( const wiced_amqp_protocol_header_arg_t *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_HDR_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_protocol_header( &frame, args );
    return amqp_frame_send( &frame, &conn_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_get_protocol_header( wiced_amqp_protocol_header_arg_t *args, wiced_amqp_connection_instance *conn_instance )
{
    if ( args->protocol_id == WICED_AMQP_PROTOCOL_ID_SASL )
    {
        return amqp_manager( WICED_AMQP_EVENT_SASL_HDR_RCVD, args, 0, conn_instance );
    }
    else if ( args->protocol_id == WICED_AMQP_PROTOCOL_ID_OPEN )
    {
        return amqp_manager( WICED_AMQP_EVENT_HDR_RCVD, args, 0, conn_instance );
    }
    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_open_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_OPEN_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_OPEN_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_open_performative( wiced_amqp_connection_instance* conn_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_OPEN_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_open( &frame, conn_instance );
    return amqp_frame_send( &frame, &conn_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_get_close_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_CLOSE_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_CLOSE_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_close_performative( wiced_amqp_connection_instance* conn_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_CLOSE_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_close_performative( &frame, conn_instance );
    return amqp_frame_send( &frame, &conn_instance->conn->socket );
}
