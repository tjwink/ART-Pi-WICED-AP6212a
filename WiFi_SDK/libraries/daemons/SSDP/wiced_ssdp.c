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
 *
 */

#include <stdlib.h>
#include <string.h>

#include "wiced.h"
#include "wiced_ssdp.h"
#include "wiced_ssdp_internal.h"


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
 *               Function Declarations
 ******************************************************/
wiced_ssdp_internal_t* g_internal = NULL;;

/******************************************************
 *               Variables Definitions
 ******************************************************/
static const char ssdp_http_error_header_template[] = ""
                                                      "HTTP/1.1 %d %s\r\n"
                                                      "Content-Type: text/plain\r\n"
                                                      "Content-Length: %d\r\n"
                                                      "Connection: close\r\n\r\n";

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_ssdp_get_uuid(char *uuid_string, uint16_t uuid_string_len)
{
    char mac_str[16];
    wiced_mac_t mac;

    wwd_wifi_get_mac_address( &mac, WICED_STA_INTERFACE );
    sprintf(mac_str, "%02x%02x%02x%02x%02x%02x", mac.octet[0], mac.octet[1], mac.octet[2], mac.octet[3], mac.octet[4], mac.octet[5]);

    if ((uuid_string == NULL) || (uuid_string_len < WICED_SSDP_UUID_MAX))
    {
        return WICED_BADARG;
    }

    strcpy(uuid_string, SSDP_UUID_TEMPLATE);
    strcat(uuid_string, mac_str);

    return WICED_SUCCESS;
}

wiced_result_t wiced_ssdp_v4_ip_addr_to_string(wiced_ip_address_t *addr, char* buff, uint16_t buff_len)
{
    /* sanity check */
    if ((addr == NULL) || (buff == NULL) || (buff_len < SSDP_INET_ADDRSTRLEN))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_v4_ip_addr_to_string() bad args!\n"));
        return WICED_BADARG;
    }

    memset(buff, 0, buff_len);

    if (addr->version == WICED_IPV4)
    {
        sprintf(buff, "%d.%d.%d.%d",
                (uint8_t)((addr->ip.v4 >> 24) & 0xff), (uint8_t)((addr->ip.v4 >> 16) & 0xff),
                (uint8_t)((addr->ip.v4 >>  8) & 0xff), (uint8_t)((addr->ip.v4 >>  0) & 0xff));
    }
    else
    {
        strcpy(buff, "Invalid IP addr");
    }

    return WICED_SUCCESS;
}

static wiced_result_t wiced_ssdp_send_packet_to_originator(wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t* cb_info, wiced_udp_socket_t *socket)
{
    /* out packet info */
    wiced_packet_t* out_packet = NULL;
    uint8_t*        out_data;
    uint16_t        out_data_len;

    /* send the reply here ! */
    if (cb_info->reply_length == 0)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_packet_to_originator() reply length == 0!\r\n"));
        return WICED_ERROR;
    }

    if (wiced_packet_create_udp( socket, cb_info->reply_length,
                                 &out_packet, &out_data, &out_data_len ) != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_packet_to_originator() wiced_packet_create_udp() error\r\n"));
        return WICED_ERROR;
    }

    /* copy over reply data */
    if (cb_info->reply_length > out_data_len)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_packet_to_originator() reply TOO long! %d > %d \r\n", cb_info->reply_length, out_data_len));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    /* start with a clean buffer */
    memset(out_data, 0, out_data_len);

    /* copy the output data into the packet buffer and set the end of the data */
    memcpy(out_data, internal->ssdp_reply_buffer, cb_info->reply_length);
    if ( wiced_packet_set_data_end( out_packet, (uint8_t *)&out_data[cb_info->reply_length] ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_packet_to_originator() wiced_packet_set_data_end() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    /* send it to the address the request was sent from */
    if ( wiced_udp_reply( socket, cb_info->packet, out_packet ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_packet_to_originator() wiced_udp_reply() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_send_packet_to_originator(len:%d)!\n\n", cb_info->reply_length));
    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("[%s]\n", internal->ssdp_reply_buffer));

    return WICED_SUCCESS;
}

/* Parse HTTP request, fill in request_info structure. */
static wiced_result_t wiced_ssdp_parse_packet(wiced_ssdp_callback_info_t* cb_info)
{
    wiced_result_t  result;
    uint16_t        total_data_length;
    char*           buff_ptr;

    /* this is really an error ! */
    wiced_assert("wiced_ssdp_parse_packet() bad args! ", (cb_info != NULL) );

    result = wiced_udp_packet_get_info( cb_info->packet, &cb_info->peer_ip_addr, &cb_info->peer_port );
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_parse_packet() wiced_udp_packet_get_info() error!"));
        return WICED_ERROR;
    }

    result = wiced_packet_get_data( cb_info->packet, 0, (uint8_t**)&cb_info->request_buffer, &cb_info->request_length,
                                    &total_data_length );
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_parse_packet() wiced_packet_get_data() error!"));
        return WICED_ERROR;
    }
    if (cb_info->request_length != total_data_length)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("Fragmented packet received!"));
        return WICED_ERROR;
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("SSDP: parse packet len:%d\n",  cb_info->request_length));
    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("[%s]\n", cb_info->request_buffer));

    /* Check we have enough data to identify the response number */
    if (cb_info->request_length < 12)
    {
        return WICED_ERROR;
    }

    /* get HTTP method  */
    cb_info->request_type = http_determine_method(cb_info->packet);

    /* Find the HTTP/x.x part*/
    buff_ptr = strstr( (const char*)cb_info->request_buffer, "HTTP/" );
    if (buff_ptr == NULL)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_parse_packet() can't find HTTP/!\n" ));
        result = WICED_ERROR;
        goto _parse_error_exit;
    }

    /* point to body */
    http_get_body( cb_info->packet, (uint8_t**)&cb_info->request_body, &cb_info->request_body_length );

    /* process the headers */
    cb_info->request_num_headers = SSDP_HTTP_HEADERS_MAX;
    http_process_headers_in_place( cb_info->packet, cb_info->request_headers, &cb_info->request_num_headers );

_parse_error_exit:

    return result;
}

static wiced_result_t wiced_ssdp_build_error_reply(wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t* cb_info)
{
    char*           reason;             /* error reason                     */
    char*           output_ptr;         /* ptr into reply_buffer            */
    uint16_t        body_size;          /* Body output size :: 0 == no body */
    uint16_t        sprintf_len;        /* written in last sprintf call     */
    uint16_t        remaining_size;     /* remaing data in reply_buffer     */
    uint8_t*        in_data;            /* point to input packet's data     */
    uint16_t        frag_data_length;   /* input packet's fragment data length (if fragmented, it's less than total) */
    uint16_t        total_data_length;  /* input packet's total data length */

    wiced_assert("wiced_ssdp_build_error_reply() internal or cb_info == NULL!", (internal != NULL) && (cb_info != NULL));

    if ( wiced_packet_get_data( cb_info->packet, 0, &in_data, &frag_data_length, &total_data_length ) == WICED_SUCCESS)
    {
        return WICED_ERROR;
    }
    if ( frag_data_length != total_data_length )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("Fragmented packet received!"));
        return WICED_ERROR;
    }

    /* determine body size */
    body_size = 0;
    reason = "Error";   /* default */
    switch (cb_info->reply_status)
    {
        case HTTP_NOT_FOUND:
            reason = "Not Found";
            body_size = strlen("Error    : ") + 2;  /* crlf */
            body_size += strlen(reason);
            body_size += strlen(reason);
            break;
        case HTTP_BAD_REQUEST:
            reason = "Bad Request";
            body_size = strlen("Error    : ") + 2;  /* crlf */
            body_size += strlen(reason);
            body_size += strlen("Cannot parse HTTP request: []") + frag_data_length;
            break;
        default:
            body_size = 0;
    }

    /* build the Content headers */
    remaining_size =  sizeof(internal->ssdp_reply_buffer) - 1;
    output_ptr = internal->ssdp_reply_buffer;
    sprintf_len = snprintf(output_ptr, remaining_size, ssdp_http_error_header_template, cb_info->reply_status, reason, body_size);
    output_ptr[sprintf_len] = 0;
    output_ptr += sprintf_len;
    remaining_size -= sprintf_len;

    /* add the body (if wanted) */
    if (body_size > 0)
    {
        sprintf_len = snprintf(output_ptr, remaining_size, "Error %d: %s\n", cb_info->reply_status, reason);
        output_ptr[sprintf_len] = 0;
        output_ptr += sprintf_len;
        remaining_size -= sprintf_len;

        switch (cb_info->reply_status)
        {
            case 400:
                sprintf_len = snprintf(output_ptr, remaining_size, reason);
                output_ptr[sprintf_len] = 0;
                output_ptr += sprintf_len;
                remaining_size -= sprintf_len;
                break;
            case 404:
                sprintf_len = snprintf(output_ptr, remaining_size, "Cannot parse HTTP request: [%.*s]", frag_data_length, in_data);
                output_ptr[sprintf_len] = 0;
                output_ptr += sprintf_len;
                remaining_size -= sprintf_len;
                break;
            default:
                break;
        }
    }

    cb_info->reply_length =  sizeof(internal->ssdp_reply_buffer) - remaining_size;

    return WICED_SUCCESS;
}

/* process the packet here !
 *
 * Check if it is a valid packet - send error if not & exit
 *
 * */
void wiced_ssdp_process_packet(wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t * cb_info )
{
    wiced_result_t              result;

    wiced_assert("wiced_ssdp_process_packet BAD ARGS!!!", (internal != NULL) && (cb_info != NULL));

    /* these fields pre-filled by mcast or server functions
     *      packet          - the received packet pointer
     *      mcast_packet    - shows which sub system called us
     *
     * these fields filled in by parse_packet()
     *      request_buffer
     *      request_length
     *      request_type
     *      request_num_headers
     *      request_headers
     *      request_body
     *
     * these fields filled by Application, default reply, or error reply
     *      reply_length
     *      reply_status
     *
     * this filled in just before we call the app callback
     *      user_data
     *
     * these fields filled here
     *      local_ip_addr
     *      local_ip_address_string
     *      local_port
     *      reply_buffer
     *      reply_buffer_size
     *
     */

    /* start with an empty reply buffer and OK status code */
    cb_info->reply_length       = 0;
    cb_info->reply_status       = HTTP_RESPONSE_OK;
    memset(internal->ssdp_reply_buffer, 0, sizeof(internal->ssdp_reply_buffer));

    cb_info->local_ip_addr     = internal->local_ip_address;
    memcpy(cb_info->local_ip_address_string, internal->local_ip_address_string, SSDP_INET_ADDRSTRLEN);
    cb_info->local_port        = internal->server_port;

    /* verify and parse packet ! */
    result = wiced_ssdp_parse_packet(cb_info);
    if( result != WICED_SUCCESS)
    {
        /* bad packet */
        cb_info->reply_status = HTTP_BAD_REQUEST;
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_process_packet() HTTP ERROR 400 (HTTP_BAD_REQUEST)\n"));
    }
    else
    {
        result = wiced_ssdp_mcast_build_default_reply(internal, cb_info);
    }

    /* Check if we need to build an error reply */
    if ((cb_info->do_not_reply == WICED_FALSE) && (cb_info->reply_status != HTTP_RESPONSE_OK))
    {
        result = wiced_ssdp_build_error_reply(internal, cb_info);
        if (result != WICED_SUCCESS)
        {
            WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_process_packet(): wiced_ssdp_build_error_reply() %d !\r\n", result));
        }
    }

    /* send the reply */
    if ((cb_info->do_not_reply == WICED_FALSE) && (result == WICED_SUCCESS))
    {
        result = wiced_ssdp_send_packet_to_originator(internal, cb_info, &internal->mcast_udp_socket);
        WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("wiced_ssdp_process_packet(): wiced_ssdp_send_packet_to_originator() return %d !\r\n", result));
    }
    else if (cb_info->do_not_reply == WICED_FALSE)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_process_packet(): reply not sent! result:%d status:%d!\r\n", result, cb_info->reply_status));
    }

    /* we're done with the in packet - delete it */
    wiced_packet_delete( cb_info->packet );
}

void wiced_ssdp_worker_thread(uint32_t arg)
{
    wiced_ssdp_request_info_t   request_info;   /* we only handle 1 request at a time */
    wiced_result_t              result;
    uint32_t                    events;
    wiced_ssdp_internal_t*      internal;

    wiced_assert("wiced_ssdp_worker_thread() ARG == NULL!", (arg != 0));
    internal = (wiced_ssdp_internal_t*)arg;

    while(1)
    {

        events = 0;
        result = wiced_rtos_wait_for_event_flags(&internal->ssdp_flags, SSDP_EVENT_WORKER_THREAD_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, SSDP_FLAGS_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & SSDP_EVENT_WORKER_PULL_PACKET)
        {
            /* copy request from queue */
            while (wiced_rtos_is_queue_empty( &internal->ssdp_queue ) != WICED_SUCCESS)
            {
                result = wiced_rtos_pop_from_queue(&internal->ssdp_queue, &request_info, SSDP_QUEUE_POP_TIMEOUT );
                if (result == WICED_SUCCESS)
                {
                    /* set up callback info ! */
                    wiced_ssdp_callback_info_t cb_info;
                    memset(&cb_info, 0x00, sizeof(wiced_ssdp_callback_info_t));
                    cb_info.packet            = request_info.packet;

                    internal->debug_queue_count--;
                    wiced_ssdp_process_packet(internal, &cb_info);
                }
                else if(result == WICED_TIMEOUT)
                {
                    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("SSDP: wiced_rtos_pop_from_queue(%d) Failed by timeout - retry!\n", internal->debug_queue_count));
                }
                else
                {
                    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("SSDP: wiced_rtos_pop_from_queue(%d) Failed!%d\n", internal->debug_queue_count, result));
                    break;
                }
            }
        }

        /* we want to service packets before exiting */
        if (events & SSDP_EVENT_WORKER_THREAD_SHUTDOWN)
        {
            break;
        }

        if (events & SSDP_EVENT_MCAST_SEND_NOTIFY)
        {
            /* this is a timer event to send the notify - don't say byebye! */
            wiced_ssdp_multicast_send_notify(internal, WICED_FALSE);
        }
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("SSDP: Worker thread shutting down!\n"));

    /* Signal worker thread we are done */
    internal->ssdp_worker_thread_ptr = NULL;
    wiced_rtos_set_event_flags(&internal->ssdp_flags, SSDP_EVENT_WORKER_THREAD_DONE);

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("SSDP: Worker thread DONE\n"));
}


/** Initialize the SSDP sub system
 *
 * @param ssdp_params
 * @param ssdp_info
 * @return
 */
wiced_result_t wiced_ssdp_init( wiced_ssdp_params_t *ssdp_params, void** ssdp_info, wiced_interface_t interface )
{
    wiced_result_t          result;
    wiced_ssdp_internal_t*  internal;

    if ((ssdp_params == NULL) || (ssdp_info == NULL))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_initialise() WICED_BADARG!\r\n"));
        return WICED_BADARG;
    }

    /* assume failure */
    *ssdp_info = NULL;

    /* allocate our internal data structure */
    internal = malloc(sizeof(wiced_ssdp_internal_t));
    if (internal == NULL)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_initialise() WICED_OUT_OF_HEAP_SPACE!\r\n"));
        return WICED_OUT_OF_HEAP_SPACE;
    }

    memset(internal, 0, sizeof(wiced_ssdp_internal_t));
    internal->tag = SSDP_TAG_INVALID;                   /* until we finish init */

    /* get our local IP address */
    if (wiced_ip_get_ipv4_address(internal->interface, &internal->local_ip_address) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_initialise() wiced_ip_get_ipv4_address() FAILED! DID YOU wiced_network_up()?\r\n"));
        goto _init_error_exit;
    }
    /* get a string version of the IP address for various uses */
    wiced_ssdp_v4_ip_addr_to_string(&internal->local_ip_address, internal->local_ip_address_string, sizeof(internal->local_ip_address_string));

    /* set multicast constants SSDP */
    internal->mcast_port        = SSDP_MULTICAST_PORT;
    SET_IPV4_ADDRESS(internal->mcast_ip_address, SSDP_MULTICAST_IPV4_ADDRESS);
    /* get a string version of the multicast IP address for various uses */
    wiced_ssdp_v4_ip_addr_to_string(&internal->mcast_ip_address, internal->mcast_ip_address_string, sizeof(internal->mcast_ip_address_string));

    /* copy over caller's params */
    internal->log_level = ssdp_params->log_level;
    if (ssdp_params->serve_page_path != NULL)
    {
        internal->serve_page_path = ssdp_params->serve_page_path;
    }
    else
    {
        internal->serve_page_path = SSDP_DEFAULT_PAGE_PATH_NAME;
    }

    if (ssdp_params->server_port > 0)
    {
        internal->server_port = ssdp_params->server_port;
    }
    else
    {
        internal->server_port = SSDP_MULTICAST_PORT + 1;
    }

    if (ssdp_params->notify_server_type != NULL)
    {
        internal->notify_server_type = ssdp_params->notify_server_type;
    }
    else
    {
        internal->notify_server_type = SSDP_DEFAULT_SERVER_TYPE;
    }

    if (ssdp_params->notify_usn_type != NULL)
    {
        internal->notify_usn_type = ssdp_params->notify_usn_type;
    }
    else
    {
        internal->notify_usn_type = SSDP_DEFAULT_USN_TYPE;
    }

    if (ssdp_params->notify_time > 0)
    {
        internal->mcast_notify_time = ssdp_params->notify_time;
        if (internal->mcast_notify_time < WICED_SSDP_MINIMUM_NOTIFY_TIME)
        {
            internal->mcast_notify_time = WICED_SSDP_MINIMUM_NOTIFY_TIME;
        }
        else if (internal->mcast_notify_time > WICED_SSDP_NOTIFY_DEFAULT_TIME)
        {
            internal->mcast_notify_time = WICED_SSDP_NOTIFY_DEFAULT_TIME;
        }
    }
    else
    {
        internal->mcast_notify_time = WICED_SSDP_NOTIFY_DEFAULT_TIME;
    }

    /* get the system UUID */
    if (ssdp_params->uuid != NULL)
    {
        strncpy(internal->uuid, ssdp_params->uuid, sizeof(internal->uuid));
        internal->uuid[sizeof(internal->uuid) - 1] = 0;
    }
    else
    {
        wiced_ssdp_get_uuid(internal->uuid, sizeof(internal->uuid));
    }

    /* initialize packet queue */
    result = wiced_rtos_init_queue( &internal->ssdp_queue, "SSDP_packet_queue", sizeof(wiced_ssdp_request_info_t), APP_QUEUE_MAX_ENTRIES );
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP: wiced_rtos_init_queue() failed:%d\r\n", result));
        goto _init_error_exit;
    }

    /* init our signal flags */
    result = wiced_rtos_init_event_flags(&internal->ssdp_flags);
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP: init_event_flags() failed:%d\r\n", result));
        goto _init_error_exit;
    }

    if( wiced_ssdp_internal_mcast_start(internal, interface) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_initialise() wiced_ssdp_internal_mcast_start() FAILED!\r\n"));
        goto _init_error_exit;
    }

    /* Start worker thread */
    result = wiced_rtos_create_thread( &internal->ssdp_worker_thread, WICED_SSDP_WORKER_THREAD_PRIORITY, "SSDP worker",
                                       wiced_ssdp_worker_thread, WICED_SSDP_WORKER_STACK_SIZE, internal);
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_rtos_create_thread(worker) failed:%d\r\n", result));
        goto _init_error_exit;
    }
    else
    {
        internal->ssdp_worker_thread_ptr = &internal->ssdp_worker_thread;
    }

    /* give pointer back to caller */
    internal->tag = SSDP_TAG_VALID;
    g_internal = internal;
    *ssdp_info = internal;

    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mulitcast on %s:%d\r\n", internal->mcast_ip_address_string, internal->mcast_port));

    return WICED_SUCCESS;

_init_error_exit:
    wiced_ssdp_deinit( *ssdp_info );
    return WICED_ERROR;
}

wiced_result_t wiced_ssdp_deinit( void* ssdp_info )
{
    uint32_t                    events;
    wiced_ssdp_request_info_t   request_info;
    wiced_ssdp_internal_t*      internal;

    /* sanity check */
    internal = (wiced_ssdp_internal_t*)ssdp_info;
    if ((internal == NULL) || (internal->tag != SSDP_TAG_VALID))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_deinit() internal %p tag invalid! 0x%lx\r\n", internal, (internal == NULL) ? 0xffffffff : internal->tag));
        return WICED_BADARG;
    }

    /* stop mcast() */
    wiced_ssdp_internal_mcast_stop(internal);

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_deinit() stopping worker\n"));
    wiced_rtos_set_event_flags(&internal->ssdp_flags, SSDP_EVENT_WORKER_THREAD_SHUTDOWN);

    /* Wait until worker thread shut down */
    events = 0;
    while (internal->ssdp_worker_thread_ptr != NULL)
    {
        wiced_result_t result;

        result = wiced_rtos_wait_for_event_flags(&internal->ssdp_flags, SSDP_EVENT_WORKER_THREAD_DONE, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, SSDP_THREAD_SHUTDOWN_WAIT);

        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & SSDP_EVENT_WORKER_THREAD_DONE)
        {
            break;
        }
    }
    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_deinit() worker stopped\n"));
    wiced_rtos_delete_thread(&internal->ssdp_worker_thread);


    /* destroy signal flags */
    wiced_rtos_deinit_event_flags(&internal->ssdp_flags);

    /* empty the queue and deinit */
    while (wiced_rtos_is_queue_empty( &internal->ssdp_queue ) != WICED_SUCCESS)
    {
        wiced_rtos_pop_from_queue(&internal->ssdp_queue, &request_info, SSDP_QUEUE_POP_TIMEOUT );
    }
    wiced_rtos_deinit_queue( &internal->ssdp_queue );

    /* mark as invalid */
    internal->tag = SSDP_TAG_INVALID;

    /* free our structure */
    free(g_internal);
    g_internal = NULL;

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_deinit() done!\r\n"));

    return WICED_ERROR;
}

wiced_result_t wiced_ssdp_notify_register_callback( void *ssdp_info, wiced_ssdp_notify_callback_t callback, void *data )
{
    wiced_ssdp_internal_t*  internal;

    /* sanity check */
    internal = (wiced_ssdp_internal_t*)ssdp_info;
    if ((internal == NULL) || (internal->tag != SSDP_TAG_VALID))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_notify_register_callback() internal %p tag invalid! 0x%lx 0x%lx\r\n", internal, (internal == NULL) ? 0xffffffff : internal->tag, SSDP_TAG_VALID));
        return WICED_BADARG;
    }

    internal->notify_callback      = callback;
    internal->notify_callback_data = data;

    return WICED_SUCCESS;
}


wiced_result_t wiced_ssdp_set_log_level( void* ssdp_info, WICED_SSDP_LOG_LEVEL_T  log_level )
{
    wiced_ssdp_internal_t*  internal;

    /* sanity check */
    internal = (wiced_ssdp_internal_t*)ssdp_info;
    if ((internal == NULL) || (internal->tag != SSDP_TAG_VALID))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_debug_dump_internal_vars() internal %p tag invalid! 0x%lx 0x%lx\r\n", internal, (internal == NULL) ? 0xffffffff : internal->tag, SSDP_TAG_VALID));
        return WICED_BADARG;
    }

    internal->log_level = log_level;

    return WICED_SUCCESS;
}


wiced_result_t wiced_ssdp_dump_callback_info( wiced_ssdp_callback_info_t* cb_info )
{
    uint16_t    i;
    /* sanity check */
    if (cb_info == NULL )
    {
        return WICED_BADARG;
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("CALLBACK INFO %p \r\n", cb_info));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    user data            :%p\r\n", cb_info->user_data));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    packet               :%p\r\n", cb_info->packet));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    local IP             :%s\r\n", cb_info->local_ip_address_string));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    local port           :%d\r\n", cb_info->local_port));

    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request buffer       :%p\r\n", cb_info->request_buffer));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request buffer len   :%d\r\n", cb_info->request_length));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request type         :%d\r\n", cb_info->request_type));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request header count :%d\r\n", cb_info->request_num_headers));
    for( i = 0; i < cb_info->request_num_headers; i++)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("            name     :%s\r\n", cb_info->request_headers[i].name));
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("            value    :%s\r\n", cb_info->request_headers[i].value));

    }
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request body         :%p\r\n", cb_info->request_body));
    WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("    request body len     :%ld\r\n", cb_info->request_body_length));

    return WICED_SUCCESS;
}

wiced_result_t wiced_ssdp_dump_debug_info( void* ssdp_info )
{
    wiced_ssdp_internal_t*  internal;

    /* sanity check */
    internal = (wiced_ssdp_internal_t*)ssdp_info;
    if ((internal == NULL) || (internal->tag != SSDP_TAG_VALID))
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("wiced_ssdp_debug_dump_internal_vars() internal %p tag invalid! 0x%lx 0x%lx\r\n", internal, (internal == NULL) ? 0xffffffff : internal->tag, SSDP_TAG_VALID));
        return WICED_BADARG;
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("wiced_ssdp_internal_t %p tag:0x%lx\r\n", internal, internal->tag));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  interface                  : %s\r\n", ((internal->interface == WICED_STA_INTERFACE) ? "STA" :
                                                                              (internal->interface == WICED_AP_INTERFACE) ? "AP" : "UNKNOWN")) );
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  worker_thread_ptr          : %p\r\n", internal->ssdp_worker_thread_ptr));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  server_port                : %d\r\n", internal->server_port));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  serve_page_path            : %s\r\n", internal->serve_page_path));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  IP address                 : %s\r\n", internal->local_ip_address_string));

    /* for multicasting SSDP */
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_ip_address           : %s\r\n", internal->mcast_ip_address_string));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_port                 : %d\r\n", internal->mcast_port));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_socket_opened        : %d\r\n", internal->mcast_socket_opened));

    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  notify_server_type         : %s\r\n", internal->notify_server_type));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  notify_usn_type            : %s\r\n", internal->notify_usn_type));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_notify_time          : %d\r\n", internal->mcast_notify_time));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_notify_timer_ms      : %ld\r\n", internal->mcast_notify_timer_ms));
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  mcast_notify_count         : %d\r\n", internal->mcast_notify_count));


    /* created/used internally */
    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  uuid                       : %s\r\n", internal->uuid));

    WPRINT_SSDP_DEBUG(SSDP_LOG_OFF,("  debug_queue_max            : %d\r\n", internal->debug_queue_max));

    return WICED_SUCCESS;
}
