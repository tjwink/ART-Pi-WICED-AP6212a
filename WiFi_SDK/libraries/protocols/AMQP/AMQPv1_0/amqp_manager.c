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
 *  Connection manager
 *
 *  Handles connection state and server exceptions.
 */

#include "wiced.h"
#include "amqp_internal.h"
#include "amqp_connection.h"
#include "amqp_frame.h"
#include "amqp_manager.h"

#include "amqp_session.h"
#include "amqp_link.h"
#include "amqp_transfer.h"
#include "amqp_flow.h"
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

wiced_result_t amqp_manager( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_instance *connection_instance )
{
    wiced_result_t result = WICED_SUCCESS;
    (void) channel;

    wiced_rtos_get_semaphore( &connection_instance->conn->semaphore, NEVER_TIMEOUT );
    switch ( event )
    {
        case WICED_AMQP_EVENT_SASL_HDR_SENT:
        {
            wiced_amqp_connection_instance *connection_instance_sent = args;
            result = amqp_connection_backend_put_protocol_header( &connection_instance_sent->header_args, connection_instance_sent );

            if ( result == WICED_SUCCESS )
            {
                connection_instance_sent->current_connection_state = AMQP_CONNECTION_SASL_HDR_SENT;
                WPRINT_LIB_DEBUG( ( "[AMQP LIB ] : Success in sending protocol header  = [%d]\r\n", result ) );
            }
            else
            {
                connection_instance_sent->current_connection_state = AMQP_CONNECTION_DISCARDING;
                WPRINT_LIB_ERROR( ( "[AMQP LIB ERROR] : Error in sending protocol header  = [%d]\r\n", result ) );
            }

            break;
        }
        case WICED_AMQP_EVENT_SASL_HDR_RCVD:
        {
            wiced_amqp_protocol_header_arg_t *header_args = args;

            if ( header_args->major == WICED_AMQP_MAJOR_NUMBER && header_args->minor == WICED_AMQP_MINOR_NUMBER && header_args->revision == WICED_AMQP_REVISION_NUMBER && header_args->protocol_id == WICED_AMQP_PROTOCOL_ID_SASL )
            {
                connection_instance->current_connection_state = AMQP_CONNECTION_SASL_HDR_EXCH;
            }
            else
            {
                return WICED_ERROR;
            }
            break;
        }
        case WICED_AMQP_EVENT_SASL_SRV_MECH_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            uint32_t i;
            connection_instance->current_connection_state = AMQP_CONNECTION_SASL_NEGOTIATION_MECH_RCVD;

            WPRINT_LIB_DEBUG(("[ AMQP LIB ]list of supported SASL server mechanisms  :\r\n"));
            for ( i = 0; i < arg->args.mech_args.count; i++ )
            {
                WPRINT_LIB_DEBUG((" \t%.*s\r\n",(int)arg->args.mech_args.list[i].size, arg->args.mech_args.list[i].item));
                if ( !strncmp( (char*) arg->args.mech_args.list[ i ].item, SASL_METHOD_PLAIN, (uint8_t) arg->args.mech_args.list[ i ].size ) )
                {
                    WPRINT_LIB_DEBUG(( "[AMQP LIB : ] Server is supporting Client SASL mechanism(SASL_METHOD_PLAIN)\r\n" ));

                    if ( connection_instance->is_sasl == 1 )
                    {
                        if ( connection_instance->plain_config.authcid == NULL || connection_instance->plain_config.passwd == NULL )
                        {
                            return WICED_ERROR;
                        }
                        else
                        {
                            /*send SASL INIT*/
                            connection_instance->current_connection_state = AMQP_CONNECTION_SASL_NEGOTIATION_MECH_RCVD;

                            result = amqp_connection_backend_put_sasl_init( connection_instance );
                            if ( result == WICED_SUCCESS )
                            {
                                connection_instance->current_connection_state = AMQP_CONNECTION_SASL_INIT_SENT;
                                WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Success in sending sasl init  = [%d]\r\n", result ) );
                            }
                            else
                            {
                                connection_instance->current_connection_state = AMQP_CONNECTION_DISCARDING;
                                WPRINT_LIB_ERROR( ( "[ AMQP LIB ERROR ] : Error in sending sasl init = [%d]\r\n", result ) );
                            }
                        }
                    }
                    break;
                }
                else
                {
                    if ( i == ( arg->args.mech_args.count - 1 ) )
                    {
                        WPRINT_APP_INFO(( "[AMQP LIB : ] Server is not supporting Client SASL mechanism\r\n" ));
                        return WICED_ERROR;
                    }
                }
            }

            break;
        }
        case WICED_AMQP_EVENT_SASL_INIT_SENT:
        {
            break;
        }
        case WICED_AMQP_EVENT_SASL_OUTCOME_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            connection_instance->current_connection_state = AMQP_CONNECTION_SASL_OUTCOME_RCVD;
            if ( arg->args.outcome_args.code == WICED_SUCCESS )
            {
                wiced_amqp_protocol_header_arg_t header_args;
                header_args.major = WICED_AMQP_MAJOR_NUMBER;
                header_args.minor = WICED_AMQP_MINOR_NUMBER;
                header_args.protocol_id = WICED_AMQP_PROTOCOL_ID_OPEN;
                header_args.revision = WICED_AMQP_REVISION_NUMBER;
                result = amqp_connection_backend_put_protocol_header( &header_args, connection_instance );

                if ( result == WICED_SUCCESS )
                {
                    connection_instance->current_connection_state = AMQP_CONNECTION_HDR_SENT;
                    WPRINT_LIB_DEBUG( ( "[AMQP LIB ] : Success in sending protocol header  = [%d]\r\n", result ) );
                }
                else
                {
                    connection_instance->current_connection_state = AMQP_CONNECTION_DISCARDING;
                    WPRINT_LIB_ERROR( ( "[AMQP LIB ERROR] : Error in sending protocol header  = [%d]\r\n", result ) );
                }
            }
            else
            {
                WPRINT_LIB_DEBUG(( "[AMQP LIB : ] SASL outcome status failed error code : %d\r\n",arg->args.outcome_args.code ));
                connection_instance->current_connection_state = AMQP_CONNECTION_DISCARDING;
                return WICED_ERROR;
            }
            break;
        }
        case WICED_AMQP_EVENT_HDR_SENT:
        {
            wiced_amqp_connection_instance *connection_instance_sent = args;
            result = amqp_connection_backend_put_protocol_header( &connection_instance_sent->header_args, connection_instance_sent );

            if ( result == WICED_SUCCESS )
            {
                connection_instance_sent->current_connection_state = AMQP_CONNECTION_HDR_SENT;
                WPRINT_LIB_DEBUG( ( "[AMQP LIB ] : Success in sending protocol header  = [%d]\r\n", result ) );
            }
            else
            {
                connection_instance_sent->current_connection_state = AMQP_CONNECTION_DISCARDING;
                WPRINT_LIB_ERROR( ( "[AMQP LIB ERROR] : Error in sending protocol header  = [%d]\r\n", result ) );
            }

            break;
        }
        case WICED_AMQP_EVENT_HDR_RCVD:
        {
            wiced_amqp_protocol_header_arg_t *header_args = args;

            if ( header_args->major == WICED_AMQP_MAJOR_NUMBER && header_args->minor == WICED_AMQP_MINOR_NUMBER && header_args->revision == WICED_AMQP_REVISION_NUMBER && header_args->protocol_id == WICED_AMQP_PROTOCOL_ID_OPEN )
            {
                connection_instance->current_connection_state = AMQP_CONNECTION_HDR_EXCH;

                result = amqp_connection_backend_put_open_performative( connection_instance );
                if ( result == WICED_SUCCESS )
                {
                    connection_instance->current_connection_state = AMQP_CONNECTION_OPEN_SENT;
                    WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Success in sending open performative  = [%d]\r\n", result ) );
                }
                else
                {
                    connection_instance->current_connection_state = AMQP_CONNECTION_DISCARDING;
                    WPRINT_LIB_ERROR( ( "[ AMQP LIB ERROR ] : Error in sending open performative  = [%d]\r\n", result ) );
                }
            }

            break;
        }

        case WICED_AMQP_EVENT_OPEN_SENT:
        {

            break;
        }
        case WICED_AMQP_EVENT_OPEN_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            connection_instance->remote_channel_max = arg->args.open_args.channel_max;
            connection_instance->remote_max_frame_size = arg->args.open_args.max_frame_size;
            connection_instance->remote_idle_timeout = arg->args.open_args.idle_timeout;
            connection_instance->current_connection_state = AMQP_CONNECTION_OPENED;
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : received  open performative  = [%d]\r\n", result ) );
            break;
        }
        case WICED_AMQP_EVENT_BEGIN_SENT:
        {
            wiced_amqp_session_instance_t* session = (wiced_amqp_session_instance_t*) args;
            result = amqp_connection_backend_put_begin_performative( session );
            break;
        }
        case WICED_AMQP_EVENT_BEGIN_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            amqp_channel_endpoint_t* current_node;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == arg->args.begin_args.remote_channel)
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            endpoint->incoming_channel = arg->fixed_frame.channel;
            session = (wiced_amqp_session_instance_t*) endpoint->context;

            session->begin_args.remote_incoming_window = arg->args.begin_args.remote_incoming_window;
            session->begin_args.remote_outgoing_window = arg->args.begin_args.remote_outgoing_window;
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : received  open performative  = [%d]\r\n", result ) );
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : session->remote_incoming_window  = [%ld]\r\n", (long unsigned int)session->begin_args.remote_incoming_window ) );
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : session->remote_outgoing_window  = [%ld]\r\n", (long unsigned int)session->begin_args.remote_outgoing_window ) );
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : session->next_outgoing_id  = [%d]\r\n", (int)session->begin_args.next_outgoing_id ) );
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : session->outgoing_window  = [%d]\r\n", (int)session->begin_args.outgoing_window ) );
            WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : session->incoming_window  = [%d]\r\n", (int)session->begin_args.incoming_window ) );
            break;
        }
        case WICED_AMQP_EVENT_ATTACH_SENT:
        {
            wiced_amqp_link_instance* link = (wiced_amqp_link_instance*) args;

            link->link_state = AMQP_LINK_STATE_INITIAL;

            result = amqp_connection_backend_put_attach_performative( link );
            if ( result == WICED_SUCCESS )
            {
                link->link_state = AMQP_LINK_STATE_HALF_ATTACHED;
            }
            else
            {
                link->link_state = AMQP_LINK_STATE_ERROR;
            }
            break;

            }
        case WICED_AMQP_EVENT_ATTACH_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            amqp_channel_endpoint_t* current_node = NULL;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session = NULL;

            wiced_amqp_link_instance* link = NULL;
            amqp_link_endpoints* link_current_node = NULL;
            amqp_link_endpoints* link_endpoint = NULL;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == arg->fixed_frame.channel )
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* find particular link endpoints from session */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );
            while ( link_current_node != NULL )
            {
                link_endpoint = (amqp_link_endpoints*) link_current_node;
                if ( !memcmp( (char*) link_endpoint->name, (char*) arg->args.link_args.name, strlen( (char*) link_endpoint->name ) ) )
                {
                    break;
                }
                link_current_node = (amqp_link_endpoints*) link_endpoint->this_node.next;
            }

            link_endpoint->input_handle = arg->args.link_args.handle;
            link = (wiced_amqp_link_instance*) link_endpoint->context;

            link->link_state = AMQP_LINK_STATE_ATTACHED;

            /*TODO : check all arguments recieved  and use it */

            if ( link->link_args.role == WICED_AMQP_ROLE_RECEIVER )
            {
                wiced_amqp_flow_instance flow;
                flow.link_instance = link;
                flow.flow_args.outgoing_window = link->session->begin_args.outgoing_window;
                flow.flow_args.incoming_window = link->session->begin_args.incoming_window;
                flow.flow_args.link_credit = link->link_credit;
                flow.flow_args.delivery_count = link->delivery_count;
                flow.flow_args.handle = link->link_args.handle;
                flow.flow_args.next_incoming_id = session->begin_args.next_incoming_id;
                flow.flow_args.next_outgoing_id = session->begin_args.next_outgoing_id;

                result = amqp_connection_backend_put_flow_performative( &flow );
            }

            break;
        }
        case WICED_AMQP_EVENT_FLOW_SENT:
        {
            wiced_amqp_flow_instance* flow_args = (wiced_amqp_flow_instance*) args;
            result = amqp_connection_backend_put_flow_performative( flow_args );
            if ( result == WICED_SUCCESS )
            {
                if ( connection_instance->conn->callbacks.connection_event != NULL )
                {
                    connection_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_FLOW_SENT, NULL, connection_instance );
                }
            }
            break;
        }
        case WICED_AMQP_EVENT_FLOW_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            amqp_channel_endpoint_t* current_node = NULL;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session = NULL;

            wiced_amqp_link_instance* link = NULL;
            amqp_link_endpoints* link_current_node = NULL;
            amqp_link_endpoints* link_endpoint = NULL;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == arg->fixed_frame.channel )
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* find particular link endpoints from session */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );
            while ( link_current_node != NULL )
            {
                link_endpoint = (amqp_link_endpoints*) link_current_node;

                if ( link_endpoint->output_handle == arg->args.flow_args.handle )
                {

                    break;
                }
                link_current_node = (amqp_link_endpoints*) link_endpoint->this_node.next;
            }

            link_endpoint->input_handle = arg->args.link_args.handle;
            link = (wiced_amqp_link_instance*) link_endpoint->context;

            /* update the values received */
            link->link_credit = arg->args.flow_args.link_credit;
            //update delivery count in link
            session->begin_args.next_incoming_id = arg->args.flow_args.next_outgoing_id;
            session->begin_args.remote_incoming_window = arg->args.flow_args.incoming_window;
            session->begin_args.remote_outgoing_window = arg->args.flow_args.outgoing_window;
            break;
        }
        case WICED_AMQP_EVENT_TRANSFER_SENT:
        {
            amqp_transfer_t* transfer = (amqp_transfer_t*) args;
            result = amqp_connection_backend_put_transfer_performative( transfer );
            if ( transfer->settle == WICED_TRUE )
            {
                /* give application callback directly for QOS 0 */
                if ( result == WICED_SUCCESS )
                  {
                      if ( connection_instance->conn->callbacks.connection_event != NULL )
                      {
                          connection_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_DISPOSITION_RCVD, NULL, connection_instance );
                      }
                  }
            }
            break;
        }
        case WICED_AMQP_EVENT_TRANSFER_RCVD:
 {
            wiced_amqp_packet_content* arg = args;
            amqp_channel_endpoint_t* current_node;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session;
            amqp_link_endpoints* current_link_endpoint;
            wiced_amqp_link_instance* link = NULL;
            amqp_disposition_t disposition_instance;
            wiced_amqp_transfer_performative_args* transfer_performative;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == arg->fixed_frame.channel)
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* traverse through all the links and find out pending deliveries */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &current_link_endpoint );
            while ( current_link_endpoint != NULL )
            {
                link = (wiced_amqp_link_instance*) current_link_endpoint->context;
                if ( link->link_endpoint->output_handle == arg->args.transfer_args.handle )
                {
                    break;
                }
                current_link_endpoint = (amqp_link_endpoints*) current_link_endpoint->this_node.next;
            }

            /* Upon receiving transfer, receiving endpoint will increment the next incoming id, decrement remote-outgoing window */
            session->begin_args.next_incoming_id++;
            session->begin_args.remote_outgoing_window--;
            session->begin_args.incoming_window--;

            /* decrement link credit and increment delivery count */
            link->link_credit--;
            link->delivery_count++;

            /* send disposition */
            transfer_performative = &arg->args.transfer_args;
            /* QOS 1 [ At least once ] need to send disposition back to acknowledge */
            if ( transfer_performative->message.settle == WICED_FALSE )
            {
                disposition_instance.link = link;
                disposition_instance.first = transfer_performative->delivery_id;
                disposition_instance.last = transfer_performative->delivery_id;
                disposition_instance.settled = transfer_performative->message.settle;
                result = amqp_connection_backend_put_disposition_performative( &disposition_instance );
            }
            else
            {
                /* QOS 0 [ At most once ] no need to send disposition back to acknowledge */
                return WICED_SUCCESS;
            }
            break;
        }
        case WICED_AMQP_EVENT_DISPOSITION_SENT:
        case WICED_AMQP_EVENT_DISPOSITION_RCVD:
        {
            wiced_amqp_packet_content *arg = args;
            amqp_channel_endpoint_t* current_node;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session;
            amqp_link_endpoints* current_link_endpoint;
            wiced_amqp_link_instance* link;
            amqp_delivery_t* current_delivery;
            amqp_delivery_t* delivery;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == arg->fixed_frame.channel)
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* traverse through all the links and find out pending deliveries */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &current_link_endpoint );
            while ( current_link_endpoint != NULL )
            {
                link = (wiced_amqp_link_instance*) current_link_endpoint->context;

                linked_list_get_front_node( &link->message_delivery_list, (linked_list_node_t**) &current_delivery );
                while ( current_delivery != NULL )
                {
                    delivery = (amqp_delivery_t*) current_delivery;

                    if ( delivery->delivery_id == arg->args.disposition_args.first && delivery->delivery_id == arg->args.disposition_args.last )
                    {
                        arg->args.disposition_args.delivery_id = delivery->delivery_id;
                        arg->args.disposition_args.delivery_tag = delivery->delivery_tag;
                        linked_list_remove_node (&link->message_delivery_list, &delivery->this_node );
                        free (delivery);
                        break;
                    }

                    current_delivery = (amqp_delivery_t*) current_delivery->this_node.next;
                }

                current_link_endpoint = (amqp_link_endpoints*) current_link_endpoint->this_node.next;
            }

            break;
        }
        case WICED_AMQP_EVENT_DETACH_SENT:
        {
            wiced_amqp_delink_instance* delink_instance = args;
            result = amqp_connection_backend_put_detach_performative( delink_instance );
            if ( result == WICED_SUCCESS )
            {
                delink_instance->link->link_state = AMQP_LINK_STATE_DETACHED;
                WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Success in sending detach performative  = [%d]\r\n", result ) );
            }
            else
            {
                delink_instance->link->link_state = AMQP_LINK_STATE_ERROR;
                WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : error in sending detach performative  = [%d]\r\n", result ) );

            }

            break;
        }
        case WICED_AMQP_EVENT_DETACH_RCVD:
        {
            wiced_amqp_packet_content* packet_content = (wiced_amqp_packet_content*)args;
            amqp_channel_endpoint_t* current_node = NULL;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session = NULL;

            wiced_amqp_link_instance* link = NULL;
            amqp_link_endpoints* link_current_node = NULL;
            amqp_link_endpoints* link_endpoint = NULL;

            amqp_delivery_t *current_delivery_node = NULL;
            amqp_delivery_t *delivery = NULL;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == packet_content->fixed_frame.channel )
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* find particular link endpoints from session */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );
            while ( link_current_node != NULL )
            {
                link_endpoint = (amqp_link_endpoints*) link_current_node;
                if ( link_endpoint->output_handle == packet_content->args.detach_args.handle )
                {
                    break;
                }
                link_current_node = (amqp_link_endpoints*) link_endpoint->this_node.next;
            }

            /* remove all delivery nodes from the link */
            link = (wiced_amqp_link_instance*) link_endpoint->context;
            linked_list_get_front_node( &link->message_delivery_list, (linked_list_node_t**) &current_delivery_node );
            while ( current_delivery_node != NULL )
            {
                delivery = (amqp_delivery_t*) current_delivery_node;
                current_delivery_node = (amqp_delivery_t*) delivery->this_node.next;
                linked_list_remove_node( &link->message_delivery_list, &delivery->this_node);
                free( delivery );
            }

            /* delete the link node from session */
            linked_list_remove_node( &session->link_endpoint, &link_endpoint->this_node );
            free( link_endpoint );

            break;
        }
        case WICED_AMQP_EVENT_END_SENT:
        {
            wiced_amqp_session_instance_t* session_instance = args;

            result = amqp_connection_backend_put_end_performative( session_instance );
            if ( result == WICED_SUCCESS )
            {
                WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Success in sending end performative  = [%d]\r\n", result ) );
            }

            break;

        }
        case WICED_AMQP_EVENT_END_RCVD:
        {
            wiced_amqp_packet_content* packet_content = (wiced_amqp_packet_content*) args;
            amqp_channel_endpoint_t* current_node = NULL;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session = NULL;

            wiced_amqp_link_instance* link = NULL;
            amqp_link_endpoints* link_current_node = NULL;
            amqp_link_endpoints* link_endpoint = NULL;

            amqp_delivery_t *current_delivery_node = NULL;
            amqp_delivery_t *delivery = NULL;

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                if ( endpoint->outgoing_channel == packet_content->fixed_frame.channel )
                {
                    break;
                }
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
            }

            session = (wiced_amqp_session_instance_t*) endpoint->context;

            /* find particular link endpoints from session */
            linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );
            while ( link_current_node != NULL )
            {

                link_endpoint = (amqp_link_endpoints*) link_current_node;
                link_current_node = (amqp_link_endpoints*) link_endpoint->this_node.next;

                /* remove all delivery nodes from the link */
                link = (wiced_amqp_link_instance*) link_endpoint->context;
                linked_list_get_front_node( &link->message_delivery_list, (linked_list_node_t**) &current_delivery_node );
                while ( current_delivery_node != NULL )
                {
                    delivery = (amqp_delivery_t*) current_delivery_node;
                    current_delivery_node = (amqp_delivery_t*) delivery->this_node.next;
                    linked_list_remove_node( &link->message_delivery_list, &delivery->this_node );
                    free( delivery );
                }
                /* delete the link node from session */
                linked_list_remove_node( &session->link_endpoint, &link_endpoint->this_node );
                free( link_endpoint );
                linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );

            }

            /*  remove session from channel endpoint linked list*/

            //session = (wiced_amqp_session_instance_t*) endpoint->context;
            /* delete the session node connection */
            linked_list_remove_node( &connection_instance->channel_endpoint, &endpoint->this_node );
            free( endpoint );

            break;
        }
        case WICED_AMQP_EVENT_CLOSE_SENT:
        {
            if ( connection_instance->current_connection_state == AMQP_CONNECTION_OPENED )
            {
                result = amqp_connection_backend_put_close_performative( connection_instance );
                if ( result == WICED_SUCCESS )
                {
                    connection_instance->current_connection_state = AMQP_CONNECTION_CLOSE_SENT;
                    WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Success in sending close performative  = [%d]\r\n", result ) );
                }
            }
            break;
        }
        case WICED_AMQP_EVENT_CLOSE_RCVD:
        {

            amqp_channel_endpoint_t* current_node = NULL;
            amqp_channel_endpoint_t* endpoint = NULL;
            wiced_amqp_session_instance_t* session = NULL;

            wiced_amqp_link_instance* link = NULL;
            amqp_link_endpoints* link_current_node = NULL;
            amqp_link_endpoints* link_endpoint = NULL;

            amqp_delivery_t *current_delivery_node = NULL;
            amqp_delivery_t *delivery = NULL;

            if(connection_instance->current_connection_state == AMQP_CONNECTION_CLOSE_RCVD)
            {
                /* Ignore connection close when the current state is already closed */
                break;
            }

            /* find particular endpoint from connection and related session */
            linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            while ( current_node != NULL )
            {
                endpoint = (amqp_channel_endpoint_t*) current_node;
                current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;

                session = (wiced_amqp_session_instance_t*) endpoint->context;

                /* find particular link endpoints from session */
                linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );
                while ( link_current_node != NULL )
                {

                    link_endpoint = (amqp_link_endpoints*) link_current_node;
                    link_current_node = (amqp_link_endpoints*) link_endpoint->this_node.next;

                    /* remove all delivery nodes from the link */
                    link = (wiced_amqp_link_instance*) link_endpoint->context;
                    linked_list_get_front_node( &link->message_delivery_list, (linked_list_node_t**) &current_delivery_node );
                    while ( current_delivery_node != NULL )
                    {
                        delivery = (amqp_delivery_t*) current_delivery_node;
                        current_delivery_node = (amqp_delivery_t*) delivery->this_node.next;
                        linked_list_remove_node( &link->message_delivery_list, &delivery->this_node );
                        free( delivery );
                    }
                    /* delete the link node from session */
                    linked_list_remove_node( &session->link_endpoint, &link_endpoint->this_node );
                    free( link_endpoint );
                    linked_list_get_front_node( &session->link_endpoint, (linked_list_node_t**) &link_current_node );

                }

                /*  remove session from channel endpoint linked list*/

                //session = (wiced_amqp_session_instance_t*) endpoint->context;
                /* delete the session node connection */
                linked_list_remove_node( &connection_instance->channel_endpoint, &endpoint->this_node );
                free( endpoint );
                linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
            }

            connection_instance->channel_endpoint.front = NULL;

            connection_instance->channel_endpoint.rear = NULL;
            connection_instance->channel_endpoint.count = 0;

            if ( connection_instance->current_connection_state == AMQP_CONNECTION_CLOSE_SENT )
            {
                connection_instance->current_connection_state = AMQP_CONNECTION_CLOSE_RCVD;
            }
            result = amqp_network_disconnect( &connection_instance->conn->socket );
            break;
        }
        case WICED_AMQP_EVENT_UNKNOWN_EVENT:
        case WICED_AMQP_EVENT_CONNECTION_ERROR:
        default:
            break;

    }
    wiced_rtos_set_semaphore( &connection_instance->conn->semaphore );
    return result;
}
