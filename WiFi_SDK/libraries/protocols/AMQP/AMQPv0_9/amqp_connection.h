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
 *  AMQP internal APIs.
 *
 *  Internal, not to be used directly by applications.
 */
#pragma once

#include "wiced.h"
#include "amqp_frame.h"
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define XSTR(S) #S
#define STR(S) XSTR(S)

/******************************************************
 *                    Constants
 ******************************************************/
#define AMQP_PROTOCOL_REPLY_SUCCESS     (200)

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
 *      Backend functions called from amqp_queue
 ******************************************************/
/*
 * Connection and channel methods functions.
 *
 * Internal not to be used directly by user applications.
 */
wiced_result_t amqp_connection_backend_put_protocol_header  (                   const wiced_amqp_protocol_header_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_protocol_header  (                         wiced_amqp_protocol_header_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_start_ok         (                                                                    wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_tune_ok          (                   const wiced_amqp_connection_tune_ok_arg_t  *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_open             (                                                                    wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_close            (                   const wiced_amqp_connection_close_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_close            ( uint16_t channel,       wiced_amqp_connection_close_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_start            ( uint16_t channel,       wiced_amqp_connection_start_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_tune             ( uint16_t channel,       wiced_amqp_connection_tune_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_open_ok          ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_close_ok         ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_close                (                                                                    wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_get_heartbeat        ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_heartbeat        (                                                                    wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_put_close_ok         (                                                                    wiced_amqp_connection_t *conn );
wiced_result_t amqp_connection_backend_closed               (                                                                    wiced_amqp_connection_t *conn );

wiced_result_t amqp_channel_backend_put_open                ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_get_open_ok             ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_put_close               ( uint16_t channel, const wiced_amqp_channel_close_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_get_close_ok            ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_put_close_ok            ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_get_close               ( uint16_t channel,       wiced_amqp_channel_close_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_put_flow                ( uint16_t channel, const wiced_amqp_channel_flow_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_get_flow                ( uint16_t channel,       wiced_amqp_channel_flow_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_put_flow_ok             ( uint16_t channel, const wiced_amqp_channel_flow_ok_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_backend_get_flow_ok             ( uint16_t channel,       wiced_amqp_channel_flow_ok_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_exchange_backend_put_declare    ( uint16_t channel, const wiced_amqp_exchange_declare_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_exchange_backend_get_declare_ok ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_exchange_backend_put_delete     ( uint16_t channel, const wiced_amqp_exchange_delete_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_exchange_backend_get_delete_ok  ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_put_declare       ( uint16_t channel, const wiced_amqp_queue_declare_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_get_declare_ok    ( uint16_t channel,       wiced_amqp_queue_declare_ok_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_put_bind          ( uint16_t channel, const wiced_amqp_queue_bind_arg_t          *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_get_bind_ok       ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_put_unbind        ( uint16_t channel, const wiced_amqp_queue_unbind_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_get_unbind_ok     ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_put_purge         ( uint16_t channel, const wiced_amqp_queue_purge_arg_t         *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_get_purge_ok      ( uint16_t channel,       wiced_amqp_queue_purge_ok_arg_t      *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_put_delete        ( uint16_t channel, const wiced_amqp_queue_delete_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_queue_backend_get_delete_ok     ( uint16_t channel,       wiced_amqp_queue_delete_ok_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_publish       ( uint16_t channel, const wiced_amqp_basic_publish_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_return        ( uint16_t channel,       wiced_amqp_basic_return_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_deliver       ( uint16_t channel,       wiced_amqp_basic_deliver_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_qos           ( uint16_t channel, const wiced_amqp_basic_qos_arg_t           *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_qos_ok        ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_consume       ( uint16_t channel, const wiced_amqp_basic_consume_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_consume_ok    ( uint16_t channel,       wiced_amqp_basic_consume_ok_arg_t    *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_ack           ( uint16_t channel, const wiced_amqp_basic_ack_arg_t           *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_reject        ( uint16_t channel, const wiced_amqp_basic_reject_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_cancel        ( uint16_t channel, const wiced_amqp_basic_cancel_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_cancel_ok     ( uint16_t channel,       wiced_amqp_basic_cancel_ok_arg_t     *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_get           ( uint16_t channel, const wiced_amqp_basic_get_arg_t           *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_get_ok        ( uint16_t channel,       wiced_amqp_basic_get_ok_arg_t        *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_get_empty     ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_recover       ( uint16_t channel, const wiced_amqp_basic_recover_arg_t       *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_put_recover_async ( uint16_t channel, const wiced_amqp_basic_recover_async_arg_t *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_basic_backend_get_recover_ok    ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_put_select           ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_get_select_ok        ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_put_commit           ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_get_commit_ok        ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_put_rollback         ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_tx_backend_get_rollback_ok      ( uint16_t channel,                                                   wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_content_backend_put_header      ( uint16_t channel, const wiced_amqp_content_header_arg_t      *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_content_backend_get_header      ( uint16_t channel,       wiced_amqp_content_header_arg_t      *args, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_content_backend_put_content     (                         wiced_amqp_frame_t                   *frame, wiced_amqp_connection_t *conn );
wiced_result_t amqp_channel_content_backend_get_content     ( uint16_t channel,       wiced_amqp_frame_t                   *frame, wiced_amqp_connection_t *conn );

#ifdef __cplusplus
} /* extern "C" */
#endif
