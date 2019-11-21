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
 *  I/O functions
 *
 *  Provides functions for sending and receiving to the network for use by
 *  framing layer.
 */
#pragma once

#include "wiced.h"
#include "amqp_link.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct amqp_delivery_s
{
    linked_list_node_t this_node;
    uint32_t delivery_id;
    uint64_t delivery_tag;
    void* callback_context;
    void* link;
} amqp_delivery_t;

typedef struct amqp_transfer_s
{
    uint32_t handle;
    uint32_t delivery_id;
    uint64_t delivery_tag;
    wiced_amqp_message_t* message;
    wiced_bool_t settle;
    wiced_bool_t more;
    wiced_amqp_link_instance* link;
} amqp_transfer_t;

typedef struct amqp_disposition_s
{
    uint32_t first;
    uint32_t last;
    wiced_bool_t settled;
    wiced_amqp_link_instance* link;
} amqp_disposition_t;

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

/*
 * Network functions.
 *
 * Internal functions not to be used by user applications.
 */
wiced_result_t amqp_connection_backend_put_transfer_performative( amqp_transfer_t* transfer_instance );
wiced_result_t amqp_connection_backend_put_disposition_performative( amqp_disposition_t* transfer_instance );
wiced_result_t amqp_frame_put_transfer( wiced_amqp_frame_t *frame,  void* arg );
wiced_result_t amqp_frame_put_disposition( wiced_amqp_frame_t *frame,  void* arg );
wiced_result_t amqp_frame_get_disposition_performative( wiced_amqp_frame_t *frame, void *arg );
wiced_result_t amqp_frame_get_transfer_performative( wiced_amqp_frame_t *frame, void *arg );
wiced_result_t amqp_connection_backend_get_disposition_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance );
wiced_result_t amqp_connection_backend_get_transfer_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance );

#ifdef __cplusplus
} /* extern "C" */
#endif
