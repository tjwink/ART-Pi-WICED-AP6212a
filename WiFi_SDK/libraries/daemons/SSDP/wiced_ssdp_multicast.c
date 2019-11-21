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
#include "wiced_crypto.h"

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const char ssdp_msearch_template[] =
{
    "M-SEARCH * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "MAN: ssdp:discover\r\n"
    "MX: %s\r\n"                                    /* msearch_scan_time:       num seconds to wait for responses (1-5, 3 suggested) */
    "ST: %s\r\n"                                    /* msearch_search_target:   search_target eq: "ssdp:all" */
    "USER-AGENT: %s UDAP/2.0\r\n"                   /* msearch_user_agent:      user_agent eq: "WICED/1.0" */
    "\r\n"
};

static const char ssdp_msearch_reply_template[] =
{
    "HTTP/1.1 200 OK\r\n"
    "CACHE-CONTROL: max-age=%s\r\n"         /* mcast_notify_time        default is 1800 */
    "EXT:\r\n"                              /* do not put a value here - MUST be empty          */
    "LOCATION: http://%s:%s/%s\r\n"         /* local_ip_address : local_port : serve_page_path  */
    "SERVER: %s\r\n"                        /* msearch_server_type:     server type - default "quick_ssdp/1.0"          */
    "ST: %s\r\n"                            /* msearch_usn_type:        get usn from application - default "ssdp:rootdevice" */
    "USN: uuid:%s::urn:%s\r\n"              /* uuid: msearch_usn_type   get UUID from system, usn same as ST: value */
    "\r\n"
};

static const char ssdp_notify_template[] =
{
    "NOTIFY * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "CACHE-CONTROL: max-age=%s\r\n"             /* mcast_notify_time        default is 1800 */
    "LOCATION: http://%s:%s/%s\r\n"             /* local_ip_addr local_port serve_page */
    "NT: %s\r\n"                                /* notify_usn_type:         default "ssdp:rootdevice" */
    "NTS: ssdp:alive\r\n"
    "SERVER: %s\r\n"                            /* notify_server_type:      server type - default "quick_ssdp/1.0"          */
    "USN: uuid:%s::urn:%s\r\n"                  /* uuid: notify_usn_type    get UUID from system, usn same as ST: value */

    "\r\n"
};

static const char ssdp_notify_byebye_template[] =
{

    "NOTIFY * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "NT: %s\r\n"                                /* notify_usn_type:         default "ssdp:rootdevice" */
    "NTS: ssdp:byebye\r\n"
    "USN: uuid:%s::urn:%s\r\n"                  /* uuid: notify_usn_type    get UUID from system, usn same as ST: value */
    "\r\n"
};

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_ssdp_mcast_collate_mcast_responses(wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t * cb_info)
{
    uint16_t                        i;
    wiced_ssdp_msearch_response_t   temp_response;
    wiced_bool_t                    save_this_one;

    if( internal->msearch_active != WICED_TRUE )
    {
        return WICED_ERROR;
    }


    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("      internal->response_array_size (%d) > (%d) internal->responses_received ?\n", internal->response_array_size, internal->responses_received));

    internal->responses_received++;

    memset(&temp_response, 0, sizeof(wiced_ssdp_msearch_response_t));
    /* go through the headers and get the values */
    temp_response.ip = cb_info->peer_ip_addr;
    wiced_ssdp_v4_ip_addr_to_string(&temp_response.ip, temp_response.ip_string, sizeof(temp_response.ip_string));

    for( i = 0; i < cb_info->request_num_headers; i++)
    {
        if (strcasecmp(cb_info->request_headers[i].name, "CACHE-CONTROL") == 0)
        {
            strncpy(temp_response.cache_control, cb_info->request_headers[i].value, sizeof(temp_response.cache_control));
        }
        else if (strcasecmp(cb_info->request_headers[i].name, "LOCATION") == 0)
        {
            strncpy(temp_response.location, cb_info->request_headers[i].value, sizeof(temp_response.location));
        }
        else if (strcasecmp(cb_info->request_headers[i].name, "ST") == 0)
        {
            strncpy(temp_response.st, cb_info->request_headers[i].value, sizeof(temp_response.st));
        }
        else if (strcasecmp(cb_info->request_headers[i].name, "USN") == 0)
        {
            strncpy(temp_response.usn, cb_info->request_headers[i].value, sizeof(temp_response.usn));
        }
    }

    save_this_one = WICED_TRUE;
    if (internal->unique_responses_received > 0)
    {
        /* check for a duplicate of one we already have */
        for (i = 0; i < internal->unique_responses_received; i++)
        {
            wiced_ssdp_msearch_response_t   *response = &internal->responses[i];
            if(memcmp(&temp_response, response, sizeof(temp_response)) == 0)
            {
                /* we already have this one! */
                save_this_one = WICED_FALSE;
                break;
            }
        }
    }

    if (save_this_one == WICED_TRUE)
    {
        memcpy(&internal->responses[internal->unique_responses_received], &temp_response, sizeof(temp_response));
        internal->unique_responses_received++;
    }

    return WICED_SUCCESS;
}

/* actually send the msearch here */
wiced_result_t wiced_ssdp_send_msearch(wiced_ssdp_internal_t* internal, wiced_ssdp_msearch_params_t *in_params)
{
    /* out packet info */
    wiced_packet_t*         out_packet = NULL;
    uint8_t*                out_data;
    uint16_t                out_data_len;
    uint16_t                sprintf_len;
    uint16_t                msearch_packet_length;
    char                    time_string[8];

    if (internal->mcast_socket_opened == WICED_FALSE)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() Socket not initialized!\r\n"));
        return WICED_ERROR;
    }

    /* build an M-Search! */
    snprintf(time_string, sizeof(time_string), "%d", in_params->msearch_scan_time);
    time_string[sizeof(time_string) - 1] = 0;

    msearch_packet_length = strlen(ssdp_msearch_template) +
                            strlen(time_string) +
                            strlen(in_params->msearch_search_target) +
                            strlen(in_params->msearch_user_agent) + 8;   /* for msearch_scan_time and padding */

    if (wiced_packet_create_udp( &internal->mcast_udp_socket, msearch_packet_length,
                                 &out_packet, &out_data, &out_data_len ) != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() wiced_packet_create_udp() error\r\n"));
        return WICED_ERROR;
    }
    /* start with a clean buffer */
    memset(out_data, 0, out_data_len);

    /* sprintf the msearch message into the packet buffer and set the end of the data */
    sprintf_len = snprintf((char*)out_data, out_data_len - 1, ssdp_msearch_template,
                           time_string, in_params->msearch_search_target, in_params->msearch_user_agent);
    out_data[sprintf_len] = 0;

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_send_msearch(len:%d)!\n[%s]\n", sprintf_len, out_data));
    if ( wiced_packet_set_data_end( out_packet, (uint8_t *)&out_data[sprintf_len] ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() wiced_packet_set_data_end() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    /* send it to the multicast address */
    if ( wiced_udp_send( &internal->mcast_udp_socket, &internal->mcast_ip_address, internal->mcast_port, out_packet ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() wiced_udp_send() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Called directly from Application
 * Suggested that m-search be sent 3 times within 100ms
 * Then wait response time
 */
wiced_result_t wiced_ssdp_send_msearch_wait_for_results(void *ssdp_info, wiced_ssdp_msearch_params_t *in_params)
{
    wiced_ssdp_internal_t*  internal;
    wiced_result_t  result;

    internal = (wiced_ssdp_internal_t*)ssdp_info;
    wiced_assert("wiced_ssdp_send_msearch() ssdp_info == NULL!", (internal != NULL) && (internal->tag == SSDP_TAG_VALID));

    if ((ssdp_info == NULL) || (in_params == NULL) || (in_params->responses == NULL) || (in_params->response_array_size == 0))
    {
        return WICED_BADARG;
    }

    /* M-Search SEND parameters */
    if ( (in_params->msearch_scan_time < 1) || (in_params->msearch_scan_time > 5))
    {
        in_params->msearch_scan_time = SSDP_DEFAULT_MSEARCH_SCAN_TIME;
    }

    if (internal->mcast_socket_opened == WICED_FALSE)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() Socket not initialized!\r\n"));
        return WICED_ERROR;
    }

    /* notify callback that we are sending an msearch */
    internal->responses_received    = 0;
    internal->responses             = in_params->responses;
    internal->response_array_size   = in_params->response_array_size;

    in_params->num_responses = 0;

    internal->msearch_active    = WICED_TRUE;
    /* send first m-search */
    result = wiced_ssdp_send_msearch(internal, in_params);
    if (result != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() 1 FAIlED! %d\r\n", result));
        internal->msearch_active    = WICED_FALSE;
        return result;
    }

    /* wait for a bit before sending next msearch */
    wiced_rtos_delay_milliseconds(WICED_SSDP_MSEARCH_SEND_TIME_1);

    /* send second m-search */
    result = wiced_ssdp_send_msearch(internal, in_params);
    if (result != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() 2 FAIlED! %d\r\n", result));
        internal->msearch_active    = WICED_FALSE;
        return result;
    }

    /* wait for a bit before sending next msearch */
    wiced_rtos_delay_milliseconds(WICED_SSDP_MSEARCH_SEND_TIME_2);

    /* send third m-search */
    result = wiced_ssdp_send_msearch(internal, in_params);
    if (result != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_send_msearch() 3 FAIlED! %d\r\n", result));
        internal->msearch_active    = WICED_FALSE;
        return result;
    }

    /* wait for specified time for responses */
    wiced_rtos_delay_milliseconds(in_params->msearch_scan_time * NUM_MSECONDS_IN_SECOND);

    internal->msearch_active       = WICED_FALSE;

    /* mark the number of responses for the caller */
    in_params->num_responses = internal->unique_responses_received;

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_send_msearch_wait_for_results() DONE!\n"));

    return WICED_SUCCESS;
}

void wiced_ssdp_notify_timer_handler( void* arg )
{
    wiced_ssdp_internal_t*      internal;

    wiced_assert("wiced_ssdp_notify_timer_handler() ARG == NULL!", (arg != 0));
    internal = (wiced_ssdp_internal_t*)arg;

    /* signal the worker thread to call wiced_ssdp_multicast_send_notify() */
    wiced_rtos_set_event_flags(&internal->ssdp_flags, SSDP_EVENT_MCAST_SEND_NOTIFY);
}

/* Send 5 notifies fairly quickly when we start, then drop back to 1/2 of the stay alive time */
wiced_result_t wiced_ssdp_multicast_send_notify(wiced_ssdp_internal_t* internal, wiced_bool_t byebye)
{
    /* out packet info */
    wiced_packet_t*         out_packet = NULL;
    uint8_t*                out_data;
    uint16_t                out_data_len;
    uint16_t                sprintf_len;
    uint16_t                notify_packet_length;
    char                    port_string[16];
    char                    time_string[16];

    wiced_assert("wiced_ssdp_multicast_send_notify() internal == NULL!", (internal != NULL) && (internal->tag == SSDP_TAG_VALID));

    /* stop the timer - we deinit it because we will change the timing each time we send it */
    wiced_rtos_stop_timer( &internal->mcast_notify_timer );
    wiced_rtos_deinit_timer( &internal->mcast_notify_timer );

    /* send out a Notify! */
    snprintf(port_string, sizeof(port_string), "%d", internal->server_port);
    port_string[sizeof(port_string) - 1] = 0;
    snprintf(time_string, sizeof(time_string), "%d", internal->mcast_notify_time);
    time_string[sizeof(time_string) - 1] = 0;

    if (byebye == WICED_TRUE)
    {

        notify_packet_length = strlen(ssdp_notify_byebye_template) +
                               strlen(internal->notify_usn_type) +
                               strlen(internal->uuid) + strlen(internal->notify_usn_type) + 8;   /* padding */

    }
    else
    {
        notify_packet_length = strlen(ssdp_notify_template) +
                               strlen(time_string) +
                               strlen(internal->local_ip_address_string) + strlen(port_string) + strlen(internal->serve_page_path) +
                               strlen(internal->notify_usn_type) +
                               strlen(internal->notify_server_type) +
                               strlen(internal->uuid) + strlen(internal->notify_usn_type) + 8;   /* padding */
    }

    if (wiced_packet_create_udp( &internal->mcast_udp_socket, notify_packet_length,
                                 &out_packet, &out_data, &out_data_len ) != WICED_SUCCESS )
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_multicast_send_notify() wiced_packet_create_udp() error\r\n"));
        return WICED_ERROR;
    }
    /* start with a clean buffer */
    memset(out_data, 0, out_data_len);

    if (byebye == WICED_TRUE)
    {
        /* sprintf the msearch message into the packet buffer and set the end of the data */
        sprintf_len = snprintf((char*)out_data, out_data_len - 1, ssdp_notify_byebye_template,
                               internal->notify_usn_type,
                               internal->uuid, internal->notify_usn_type);
        out_data[sprintf_len] = 0;
    }
    else
    {
        /* sprintf the msearch message into the packet buffer and set the end of the data */
        sprintf_len = snprintf((char*)out_data, out_data_len - 1, ssdp_notify_template, time_string,
                               internal->local_ip_address_string, port_string, internal->serve_page_path,
                               internal->notify_usn_type,
                               internal->notify_server_type,
                               internal->uuid, internal->notify_usn_type);
        out_data[sprintf_len] = 0;
    }

    WPRINT_SSDP_DEBUG(SSDP_LOG_INFO,("wiced_ssdp_multicast_send_notify(len:%d) %s!\n\n", sprintf_len, ((byebye == WICED_TRUE) ? "Bye Bye" : " ")));
    WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("[%s]\n", out_data));

    if ( wiced_packet_set_data_end( out_packet, (uint8_t *)&out_data[sprintf_len] ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_multicast_send_notify() wiced_packet_set_data_end() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    /* send it to the multicast address */
    if ( wiced_udp_send( &internal->mcast_udp_socket, &internal->mcast_ip_address, internal->mcast_port, out_packet ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_multicast_send_notify() wiced_udp_send() error\r\n"));
        wiced_packet_delete( out_packet );
        return WICED_ERROR;
    }

    /* if not byebye, re-start the timer with new random value */
    if (byebye != WICED_TRUE)
    {
        uint32_t    notify_time_ms;

        uint16_t    wiced_random_16;
        wiced_crypto_get_random( &wiced_random_16, sizeof( wiced_random_16 ) );

        /* send first 5 within 1 second */
        notify_time_ms = 100 - (wiced_random_16 % 40);      /* randomize between 60 and 100ms */

        /* the first few times we send NOTIFY we want to send it quickly */
        internal->mcast_notify_count++;
        if (internal->mcast_notify_count >= WICED_SSDP_MCAST_INITIAL_NOTIFY_FAST_COUNT)
        {
            uint32_t notify_time;
            notify_time = (internal->mcast_notify_time / 2);    /* notify_time will be 60 - 1800 seconds */
            notify_time -= (wiced_random_16 % (internal->mcast_notify_time / 10));  /* random value less than 1/2 max time */
            notify_time_ms *= NUM_MSECONDS_IN_SECOND;

            if (internal->mcast_notify_count > (uint32_t)0X7FFFFFF0 )
            {
                /* Don't go all negative on us! */
                internal->mcast_notify_count = WICED_SSDP_MCAST_INITIAL_NOTIFY_FAST_COUNT;
            }
        }
        internal->mcast_notify_timer_ms = notify_time_ms;
        wiced_rtos_init_timer(&internal->mcast_notify_timer,  internal->mcast_notify_timer_ms, wiced_ssdp_notify_timer_handler, internal);
        wiced_rtos_start_timer(&internal->mcast_notify_timer );
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_ssdp_mcast_build_default_reply(wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t * cb_info)
{
    uint16_t    sprintf_len;
    char        time_string[16];
    char        port_string[16];


    /* If we got a notify, do not respond - call application if callback registered */
    if (strncasecmp(cb_info->request_buffer, "NOTIFY", 6) == 0)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("wiced_ssdp_mcast_build_default_reply() NOTIFY - do not reply!\n"));

        if (internal->notify_callback != NULL)
        {
            int i;
            wiced_ssdp_notify_info_t notify_info;

            memset( &notify_info, 0, sizeof(notify_info));
            notify_info.ip = cb_info->peer_ip_addr;
            wiced_ssdp_v4_ip_addr_to_string(&notify_info.ip, notify_info.ip_string, sizeof(notify_info.ip_string));

            for( i = 0; i < cb_info->request_num_headers; i++)
            {
                if (strcasecmp(cb_info->request_headers[i].name, "CACHE-CONTROL") == 0)
                {
                    strncpy(notify_info.cache_control, cb_info->request_headers[i].value, sizeof(notify_info.cache_control));
                }
                else if (strcasecmp(cb_info->request_headers[i].name, "LOCATION") == 0)
                {
                    strncpy(notify_info.location, cb_info->request_headers[i].value, sizeof(notify_info.location));
                }
                else if (strcasecmp(cb_info->request_headers[i].name, "SERVER") == 0)
                {
                    strncpy(notify_info.server, cb_info->request_headers[i].value, sizeof(notify_info.server));
                }
                else if (strcasecmp(cb_info->request_headers[i].name, "NT") == 0)
                {
                    strncpy(notify_info.nt, cb_info->request_headers[i].value, sizeof(notify_info.nt));
                }
                else if (strcasecmp(cb_info->request_headers[i].name, "NTS") == 0)
                {
                    strncpy(notify_info.nts, cb_info->request_headers[i].value, sizeof(notify_info.nts));
                }
                else if (strcasecmp(cb_info->request_headers[i].name, "USN") == 0)
                {
                    strncpy(notify_info.usn, cb_info->request_headers[i].value, sizeof(notify_info.usn));
                }
            }

            WPRINT_SSDP_DEBUG(SSDP_LOG_DEBUG,("wiced_ssdp_mcast_build_default_reply() call Application!\n"));
            internal->notify_callback( &notify_info, internal->notify_callback_data);
        }

        cb_info->do_not_reply = WICED_TRUE;
        return WICED_SUCCESS;
    }

    /* if we are searching, do that instead! */
    if (internal->msearch_active == WICED_TRUE)
    {
        wiced_result_t result;
        cb_info->do_not_reply = WICED_TRUE;
        result = wiced_ssdp_mcast_collate_mcast_responses(internal, cb_info);
        if(result == WICED_SUCCESS)
        {
            return result;
        }
        /* try regular processing! */
        cb_info->do_not_reply = WICED_FALSE;
    }

    /* create the ssdp_multicast reply text */
    snprintf(time_string, sizeof(time_string), "%d", internal->mcast_notify_time);
    time_string[sizeof(time_string) - 1] = 0;
    snprintf(port_string, sizeof(port_string), "%d", internal->server_port);
    port_string[sizeof(port_string) - 1] = 0;


    sprintf_len = strlen(ssdp_msearch_reply_template) +
                  strlen(time_string) +
                  strlen(cb_info->local_ip_address_string) + strlen(port_string) + strlen(internal->serve_page_path) +
                  strlen(internal->notify_server_type) +
                  strlen(internal->notify_usn_type) +
                  strlen(internal->uuid) + strlen(internal->notify_usn_type) + 8;

    if (sprintf_len <  sizeof(internal->ssdp_reply_buffer))
    {
        memset(internal->ssdp_reply_buffer, 0x00,  sizeof(internal->ssdp_reply_buffer));
        sprintf_len = snprintf(internal->ssdp_reply_buffer,  sizeof(internal->ssdp_reply_buffer) - 1,ssdp_msearch_reply_template,
                               time_string,
                               cb_info->local_ip_address_string, port_string, internal->serve_page_path,
                               internal->notify_server_type,
                               internal->notify_usn_type,
                               internal->uuid, internal->notify_usn_type);
        internal->ssdp_reply_buffer[sprintf_len] = 0;
        cb_info->reply_length = sprintf_len;
    }

    return WICED_SUCCESS;
}

/*
 * This is the callback from the socket subsystem
 *
 * We have received a packet here
 * add it to our queue to be processed by the multicast thread
 *
 */
wiced_result_t wiced_ssdp_mcast_receive_callback( wiced_udp_socket_t* socket, void* arg )
{
    wiced_ssdp_request_info_t   request_info;
    wiced_ssdp_internal_t*      internal;
    wiced_result_t              result;

    if (arg == NULL)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_mcast_receive_callback() BAD ARG: 0x%p", arg));
        return WICED_BADARG;
    }
    internal = (wiced_ssdp_internal_t *)arg;

    memset(&request_info, 0, sizeof(wiced_ssdp_request_info_t));
    if( wiced_udp_receive(socket, &request_info.packet, SSDP_PACKET_TIMEOUT ) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* this function copies the data to the queue */
    result = wiced_rtos_push_to_queue( &internal->ssdp_queue, &request_info, SSDP_QUEUE_PUSH_TIMEOUT );
    if (result != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP: wiced_rtos_push_to_queue(%d) Failed!\n", internal->debug_queue_count));
        wiced_packet_delete( request_info.packet );
        return WICED_ERROR;
    }
    internal->debug_queue_count++;
    if (internal->debug_queue_count > internal->debug_queue_max)
    {
        internal->debug_queue_max = internal->debug_queue_count;
    }

    wiced_rtos_set_event_flags(&internal->ssdp_flags, SSDP_EVENT_WORKER_PULL_PACKET);

    return WICED_SUCCESS;
}

/*
 * Initialize a socket and register the internal callback for receiving packets
 */
wiced_bool_t wiced_ssdp_socket_create_and_register_calback( wiced_udp_socket_t* udp_socket, uint16_t port, wiced_udp_socket_callback_t receive_callback, void *data )
{
    wiced_assert("wiced_ssdp_socket_create_and_register_calback() udp_socket == NULL!", (udp_socket != NULL));

    return WICED_TRUE;
}

/** Start the Multicast support for SSDP
 *
 * @param internal  - ptr to internal data structure
 *
 *  Join the Multicast Group
 *  Open a socket to send / receive packets
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR;
 */
wiced_result_t wiced_ssdp_internal_mcast_start( wiced_ssdp_internal_t*  internal, wiced_interface_t interface )
{
    wiced_result_t  result = WICED_SUCCESS;

    /* sanity check */
    wiced_assert("wiced_ssdp_internal_mcast_start() internal == NULL!", (internal != NULL));

    result = wiced_multicast_join( internal->interface, &internal->mcast_ip_address );
    if (result != WICED_TCPIP_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: wiced_multicast_join() failed:%d\r\n", result));
        goto _mcast_start_error_exit;
    }

    if (internal->mcast_socket_opened == WICED_TRUE)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: internal->mcast_socket_opened already opened!:%d\r\n", internal->mcast_socket_opened));
        goto _mcast_start_error_exit;
    }

    /* Create a UDP socket */
    if (wiced_udp_create_socket(&internal->mcast_udp_socket, internal->mcast_port, interface) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: UDP socket creation failed\n"));
        goto _mcast_start_error_exit;
    }
    internal->mcast_socket_opened = WICED_TRUE;

    if (wiced_udp_update_socket_backlog(&internal->mcast_udp_socket, WICED_SSDP_SERVER_LISTEN_BACKLOG ) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: wiced_udp_update_socket_backlog() failed\n"));
    }

    if( wiced_udp_register_callbacks(&internal->mcast_udp_socket, wiced_ssdp_mcast_receive_callback, internal) != WICED_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: wiced_udp_register_callbacks() failed\n"));
        goto _mcast_start_error_exit;
    }

    /* start Notifications */
    internal->mcast_notify_timer_ms = 100;
    wiced_rtos_init_timer(&internal->mcast_notify_timer, internal->mcast_notify_timer_ms, wiced_ssdp_notify_timer_handler, internal);
    wiced_rtos_start_timer(&internal->mcast_notify_timer );

    return WICED_SUCCESS;

_mcast_start_error_exit:
    return WICED_ERROR;
}


/** Stop the Multicast support for SSDP
 *
 * @param internal  - ptr to internal data structure
 *
 *  Stop Notify timer
 *  Stop M-Search (if active)
 *  Open a socket to send / receive packets
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR;
 */
wiced_result_t wiced_ssdp_internal_mcast_stop( wiced_ssdp_internal_t* internal )
{
    wiced_result_t result;

    wiced_assert("wiced_ssdp_internal_mcast_stop() internal == NULL!", (internal != NULL) && (internal->tag == SSDP_TAG_VALID));

    wiced_rtos_stop_timer( &internal->mcast_notify_timer );
    wiced_rtos_deinit_timer( &internal->mcast_notify_timer );

    if (internal->mcast_socket_opened == WICED_TRUE)
    {
        if( wiced_udp_unregister_callbacks( &internal->mcast_udp_socket ) != WICED_SUCCESS)
        {
            WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_internal_mcast_stop() UDP unregister_callback() failed\n"));
        }

        /* send byebye message  - we do not expect replies from NOTIFY messages */
        wiced_ssdp_multicast_send_notify(internal, WICED_TRUE);

        /* close the socket */
        internal->mcast_socket_opened = WICED_FALSE;
        if ( wiced_udp_delete_socket( &internal->mcast_udp_socket ) != WICED_SUCCESS)
        {
            WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("wiced_ssdp_server_close_socket() UDP delete failed\n"));
        }
    }

    result = wiced_multicast_leave( internal->interface, &internal->mcast_ip_address );
    if (result != WICED_TCPIP_SUCCESS)
    {
        WPRINT_SSDP_DEBUG(SSDP_LOG_LOW,("SSDP mcast: wiced_multicast_leave() failed:%d\r\n", result));
    }
    return WICED_SUCCESS;
}
