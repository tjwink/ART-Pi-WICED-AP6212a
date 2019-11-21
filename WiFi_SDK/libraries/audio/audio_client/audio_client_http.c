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
 * Audio Client HTTP processing
 *
 */

#include <ctype.h>

#include "dns.h"

#include "wiced_result.h"
#include "wiced_defaults.h"
#include "wiced_rtos.h"
#include "wiced_tcpip.h"
#include "wiced_tls.h"
#include "wiced_network.h"
#include "wiced_log.h"

#include "audio_client_private.h"
#include "audio_client_utils.h"
#include "audio_client_hls.h"

#include "audio_client.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define PRINT_IP(addr)    (int)((addr.ip.v4 >> 24) & 0xFF), (int)((addr.ip.v4 >> 16) & 0xFF), (int)((addr.ip.v4 >> 8) & 0xFF), (int)(addr.ip.v4 & 0xFF)

#define ADVANCE_HEADER(ptr,size)        \
            do                          \
            {                           \
                ptr += size;            \
                while (*ptr == ' ')     \
                {                       \
                    ptr++;              \
                }                       \
            } while(0)

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_CLIENT_DNS_TIMEOUT_MS         (1 * 1000)
#define AUDIO_CLIENT_CONNECT_TIMEOUT_MS     (2 * 1000)
#define AUDIO_CLIENT_RECEIVE_TIMEOUT_MS     (WICED_NO_WAIT)

#define HTTP_HEARTBEAT_DURATION_MS          (20 * 1000)
#define HTTP_PACKET_TIMEOUT_MS              (1 * 60 * 1000)

#define AUDIO_CLIENT_HTTP_THREAD_STACK_SIZE (2 * 4096)
#define AUDIO_CLIENT_HTTP_THREAD_PRIORITY   (4)
#define AUDIO_CLIENT_HLS_THREAD_STACK_SIZE  (5120)
#define AUDIO_CLIENT_HLS_THREAD_PRIORITY    (5)

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

static void audio_client_http_handle_hls_playlist(audio_client_t* client, audio_client_http_params_t* params);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static char audio_client_HTTP_get_request_no_port_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "\r\n"
};

static char audio_client_HTTP_get_request_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s:%d\r\n"
    "\r\n"
};

static char audio_client_HTTP_get_range_no_port_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Range: bytes=%d-\r\n"
    "\r\n"
};

static char audio_client_HTTP_get_range_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s:%d\r\n"
    "Range: bytes=%d-\r\n"
    "\r\n"
};

static char audio_client_HTTP_response[] =
{
    "HTTP/1.?"
};

static char audio_client_ICY_response[] =
{
    "ICY"
};

static char audio_client_HTTP_body_separator[] =
{
    "\r\n\r\n"
};

static char audio_client_HTTP_chunk_separator[] =
{
    "\r\n"
};

static lookup_t container_extension_types[] =
{
    { ".ts",                        AUDIO_CLIENT_CONTAINER_TS       },
    { ".m2ts",                      AUDIO_CLIENT_CONTAINER_TS       },
    { NULL,                         AUDIO_CLIENT_CONTAINER_NULL     }       /* Must be last! */
};

static lookup_t container_mime_types[] =
{
    { "video/mp2t",                 AUDIO_CLIENT_CONTAINER_TS       },
    { NULL,                         AUDIO_CLIENT_CONTAINER_NULL     }       /* Must be last! */
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static void http_timer_callback(void *arg)
{
    audio_client_http_params_t* params = (audio_client_http_params_t*)arg;
    audio_client_t* client = params->client;

    if (client != NULL && client->tag == AUDIO_CLIENT_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&params->http_events, HTTP_EVENT_TIMER);
    }
}


static wiced_result_t received_data_callback(wiced_tcp_socket_t* socket, void* arg)
{
    audio_client_http_params_t* params = (audio_client_http_params_t*)arg;
    audio_client_t* client = params->client;

    if (client != NULL && client->tag == AUDIO_CLIENT_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&params->http_events, HTTP_EVENT_TCP_DATA);
    }

    return WICED_SUCCESS;
}


static wiced_result_t disconnect_callback(wiced_tcp_socket_t* socket, void* arg)
{
    audio_client_http_params_t* params = (audio_client_http_params_t*)arg;
    audio_client_t* client = params->client;

    if (client != NULL && client->tag == AUDIO_CLIENT_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&params->http_events, HTTP_EVENT_DISCONNECT);
    }

    return WICED_SUCCESS;
}


static void reset_initial_http_variables(audio_client_t* client, audio_client_http_params_t* params, wiced_bool_t seek_request)
{
    params->http_done                      = WICED_FALSE;
    params->http_error                     = WICED_FALSE;
    params->http_range_requests            = WICED_TRUE;
    params->http_need_header               = WICED_TRUE;
    params->http_need_extra_header_check   = WICED_FALSE;
    params->http_in_header                 = WICED_FALSE;
    params->http_icy_header                = WICED_FALSE;
    params->http_transfer_encoding_chunked = WICED_FALSE;
    params->http_redirect                  = WICED_FALSE;
    params->http_header_idx                = 0;
    params->http_buf_idx                   = 0;
    params->http_body_idx                  = 0;
    params->http_content_length            = 0;
    params->http_content_read              = 0;
    params->http_chunk_size                = 0;
    params->http_chunk_bytes               = 0;
    params->http_chunk_skip                = 0;

    if (client->hls_playlist_active == WICED_FALSE)
    {
        if (params == &client->http_params)
        {
            client->decoder_first_kick     = WICED_TRUE;
            client->threshold_high_sent    = WICED_FALSE;
            client->threshold_low_sent     = WICED_FALSE;
            params->initial_buffer_count   = 0;
        }
        params->http_content_type[0]       = '\0';
    }

    if (params == &client->http_params)
    {
        client->seek_in_progress           = seek_request;
    }

    if (seek_request == WICED_FALSE)
    {
        params->http_total_content_length  = 0;
    }
}


static wiced_result_t audio_client_uri_split(const char* uri, char* host, int host_len, char* path, int path_len, int* port, wiced_bool_t* tls)
{
    const char* uri_start;
    int copy_len;
    int len;

    *port = 0;
    *tls  = WICED_FALSE;

    /*
     * Skip over http:// or https://
     */

    uri_start = uri;
    if ((uri[0] == 'h' || uri[0] == 'H') && (uri[1] == 't' || uri[1] == 'T') && (uri[2] == 't' || uri[2] == 'T') && (uri[3] == 'p' || uri[3] == 'P'))
    {
        uri_start += 4;
        if (uri[4] == 's' || uri[4] == 'S')
        {
            *tls = WICED_TRUE;
            uri_start++;
        }
        if (uri_start[0] != ':' || uri_start[1] != '/' || uri_start[2] != '/')
        {
            return WICED_BADARG;
        }
        uri_start += 3;
    }

    /*
     * Isolate the host part of the URI.
     */

    for (len = 0; uri_start[len] != ':' && uri_start[len] != '/' && uri_start[len] != '\0'; )
    {
        len++;
    }

    if (uri_start[len] == ':')
    {
        *port = atoi(&uri_start[len + 1]);
    }
    else
    {
        if (*tls == WICED_TRUE)
        {
            *port = AUDIO_CLIENT_DEFAULT_HTTPS_PORT;
        }
        else
        {
            *port = AUDIO_CLIENT_DEFAULT_HTTP_PORT;
        }
    }

    copy_len = len;
    if (copy_len > host_len - 2)
    {
        copy_len = host_len - 2;
    }
    memcpy(host, uri_start, copy_len);
    host[copy_len] = '\0';

    /*
     * Now pull out the path part.
     */

    uri_start += len;
    if (*uri_start == ':')
    {
        while (*uri_start != '/' && *uri_start != '\0')
        {
            uri_start++;
        }
    }

    if (*uri_start != '\0')
    {
        copy_len = strlen(uri_start);
        if (copy_len > path_len - 2)
        {
            copy_len = path_len - 2;
        }
        memcpy(path, uri_start, copy_len);
        path[copy_len] = '\0';
    }
    else
    {
        *path++ = '/';
        *path   = '\0';
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_connect(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t          result;
    wiced_ip_address_t      ip_address;
    wiced_tls_context_t*    tls_context;
    int                     connect_dns_tries;
    wiced_tls_extension_t   extension;
    wiced_bool_t            have_hostname = WICED_FALSE;
    wiced_bool_t            tls = WICED_FALSE;

    result = audio_client_uri_split(params->stream_uri, params->hostname, AUDIO_CLIENT_HOSTNAME_LEN, params->path, AUDIO_CLIENT_PATH_LEN, &params->port, &tls);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to parse URI %s\n", params->stream_uri);
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Connect to host: %s, port %d, path %s\n", params->hostname, params->port, params->path);

    /*
     * Are we dealing with an IP address or a hostname?
     */

    if (str_to_ip(params->hostname, &ip_address) != 0)
    {
        /*
         * Try a hostname lookup.
         */

        connect_dns_tries = 0;
        do {
            connect_dns_tries++;
            result = wiced_hostname_lookup(params->hostname, &ip_address, AUDIO_CLIENT_DNS_TIMEOUT_MS, WICED_STA_INTERFACE);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "DNS lookup attempt #%d for %s failed with %d\n", connect_dns_tries, params->hostname, result);
            }
        } while (connect_dns_tries < 3 && result != WICED_SUCCESS);

        if (result != WICED_SUCCESS)
        {
            return result;
        }
        have_hostname = WICED_TRUE;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Using IP address %d.%d.%d.%d\n", PRINT_IP(ip_address));

    /*
     * Open a socket for the connection.
     */

    if (params->socket_ptr == NULL)
    {
        result = wiced_tcp_create_socket(&params->socket, client->params.interface);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create socket (%d)\n", result);
            return result;
        }
        params->socket_ptr = &params->socket;

        result = wiced_tcp_register_callbacks(params->socket_ptr, NULL, received_data_callback, disconnect_callback, params);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to register socket callbacks (%d)\n", result);
            return result;
        }
    }

    if (tls == WICED_TRUE)
    {
        /*
         * Allocate the TLS context and associate with our socket.
         * If we set the socket context_malloced flag to true, the TLS
         * context will be freed automatically when the socket is deleted.
         */

        tls_context = malloc(sizeof(wiced_tls_context_t));
        if (tls_context == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate tls context\n");
            return result;
        }
        wiced_tls_init_context(tls_context, NULL, NULL);
        if (have_hostname)
        {
            extension.type = TLS_EXTENSION_TYPE_SERVER_NAME;
            extension.extension_data.server_name = (uint8_t*)params->hostname;

            if (wiced_tls_set_extension(tls_context, &extension) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to set TLS hostname extension\n");
            }
        }
        wiced_tcp_enable_tls(params->socket_ptr, tls_context);
        params->socket_ptr->context_malloced = WICED_TRUE;
    }

    /*
     * Now try and connect.
     */

    connect_dns_tries = 0;
    do {
        connect_dns_tries++;
        result = wiced_tcp_connect(params->socket_ptr, &ip_address, params->port, AUDIO_CLIENT_CONNECT_TIMEOUT_MS);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Connection attempt #%d to host %s:%d failed (%d)\n",
                          connect_dns_tries, params->hostname, params->port, result);
        }
    } while (connect_dns_tries < 3 && result != WICED_SUCCESS);

    if (result != WICED_SUCCESS)
    {
        wiced_tcp_delete_socket(params->socket_ptr);
        params->socket_ptr = NULL;
        return result;
    }

    return WICED_SUCCESS;
}


static wiced_result_t audio_client_disconnect(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t result = WICED_TCPIP_SUCCESS;

    UNUSED_PARAMETER(client);

    if (params->socket_ptr != NULL)
    {
        result = wiced_tcp_disconnect_with_timeout(params->socket_ptr, WICED_NO_WAIT);
        wiced_tcp_delete_socket(params->socket_ptr);
        params->socket_ptr = NULL;
    }

    /*
     * Kill the timer.
     */

    if (wiced_rtos_is_timer_running(&params->http_timer) == WICED_SUCCESS)
    {
        wiced_rtos_stop_timer(&params->http_timer);
    }
    wiced_rtos_deinit_timer(&params->http_timer);

    return result;
}


static wiced_result_t audio_client_connect_and_send_request(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t       result = WICED_SUCCESS;
    AUDIO_CLIENT_ERROR_T err    = AUDIO_CLIENT_ERROR_SUCCESS;

    /*
     * Reset our housekeeping variables.
     */

    reset_initial_http_variables(client, params, WICED_FALSE);

    /*
     * See if we can connect to the HTTP server.
     */

    if (audio_client_connect(client, params) != WICED_SUCCESS)
    {
        err = AUDIO_CLIENT_ERROR_CONNECT_FAILED;
        params->http_error = WICED_TRUE;
        result = WICED_ERROR;
        goto _exit;
    }

    /*
     * Let the overlord know that we are connected.
     */

    if (params == &client->http_params)
    {
        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_CONNECTED, NULL);
    }

    /*
     * Send off the GET request to the HTTP server.
     * Do not include standard port in get request. Some radio stations don't like that.
     * (E.g. http://direct.franceinfo.fr/live/franceinfo-midfi.mp3)
     */

    if ((params->port == AUDIO_CLIENT_DEFAULT_HTTP_PORT) || (params->port == AUDIO_CLIENT_DEFAULT_HTTPS_PORT))
    {
        snprintf(params->http_buf, AUDIO_CLIENT_HTTP_BUF_SIZE, audio_client_HTTP_get_request_no_port_template, params->path, params->hostname);
    }
    else
    {
        snprintf(params->http_buf, AUDIO_CLIENT_HTTP_BUF_SIZE, audio_client_HTTP_get_request_template, params->path, params->hostname, params->port);
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Sending query:\n%s\n", params->http_buf);

    result = wiced_tcp_send_buffer(params->socket_ptr, params->http_buf, (uint16_t)strlen(params->http_buf));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to send HTTP query (%d)\n", result);
        err = AUDIO_CLIENT_ERROR_HTTP_QUERY_FAILED;
        params->http_error = WICED_TRUE;
        result = WICED_ERROR;
        goto _exit;
    }

    if ((params == &client->http_params) && (client->hls_playlist_active == WICED_FALSE))
    {
        client->decoder_first_kick = WICED_TRUE;
    }


    /*
     * Start a timer to monitor the socket.
     */

    if (wiced_rtos_init_timer(&params->http_timer, HTTP_HEARTBEAT_DURATION_MS, http_timer_callback, params) == WICED_SUCCESS)
    {
        wiced_rtos_start_timer(&params->http_timer);
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_WARNING, "Unable to create socket timer\n");
    }

 _exit:
    if (params == &client->http_params)
    {
        if ((client->hls_playlist_active == WICED_FALSE) || (client->hls_playlist_is_live == WICED_FALSE && client->hls_playlist_last_entry == WICED_TRUE))
        {
            if (err != AUDIO_CLIENT_ERROR_SUCCESS)
            {
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)err);
            }
        }
        else
        {
            /*
             * We're playing an HLS stream and it's not the last media segment;
             * do not report errors to the app and give us a chance
             * to try and play the next media segment
             */
            if (err != AUDIO_CLIENT_ERROR_SUCCESS)
            {
                result             = WICED_SUCCESS;
                params->http_error = WICED_FALSE;
                params->http_done  = WICED_TRUE;
                audio_client_http_handle_hls_playlist(client, params);
            }
        }
    }
    return result;
}


static void audio_client_destroy_filter(audio_client_t* client)
{
    client->filters[client->filter_type].filter_destroy(client);
    client->filter_type            = AUDIO_CLIENT_CONTAINER_NULL;
    client->filter_content_type[0] = '\0';
}


static void audio_client_look_for_container_type(audio_client_t* client, audio_client_http_params_t* params)
{
    AUDIO_CLIENT_CONTAINER_T filter_type = AUDIO_CLIENT_CONTAINER_NULL;

    if (params->http_load_file == WICED_FALSE)
    {
        int i;

        /*
         * Checking for a container mime type
         */

        for (i = 0; container_mime_types[i].name != NULL; i++)
        {
            if (strncasecmp(container_mime_types[i].name, params->http_content_type, strlen(container_mime_types[i].name)) == 0)
            {
                filter_type = container_mime_types[i].value;
                break;
            }
        }

        /*
         * Looking for extension in URI
         */

        for (i = 0; (filter_type == AUDIO_CLIENT_CONTAINER_NULL) && (container_extension_types[i].name != NULL); i++)
        {
            if (strstr(params->stream_uri, container_extension_types[i].name) != NULL)
            {
                filter_type = container_extension_types[i].value;
                break;
            }
        }
    }

    if (filter_type == client->filter_type)
    {
        if (client->hls_playlist_active == WICED_TRUE)
        {
            /*
             * Reset filter state before processing a new HLS media segment;
             * in case PAT/PMT content changes from one segment to the next
             */
            client->filters[client->filter_type].filter_ioctl(client, FILTER_IOCTL_RESET_STATE, NULL);
        }
    }
    else
    {
        /*
         * There is an active container handler / filter; clean it out
         */

        audio_client_destroy_filter(client);

        if (filter_type != AUDIO_CLIENT_CONTAINER_NULL)
        {
            /*
             * We're dealing with a container that decoders can't handle
             */
            if (client->filters[filter_type].filter_create(client) != WICED_SUCCESS)
            {
                client->filter_type = AUDIO_CLIENT_CONTAINER_NULL;
            }
            else
            {
                client->filter_type = filter_type;
            }
        }
    }
}


static wiced_result_t audio_client_parse_http_header(audio_client_t* client, audio_client_http_params_t* params)
{
    audio_client_play_request_t* play_node;
    audio_client_playlist_t* playlist;
    char*           ptr;
    char*           semi;
    char*           line_end;
    uint16_t        http_code;
    wiced_result_t  result;
    int             len;
    int             len_original_uri;

    result = audio_client_http_get_response_code(params->http_buf, params->http_buf_idx, &http_code);
    if (result == WICED_SUCCESS && (http_code == HTTP_STATUS_MOVED_PERMANENTLY || http_code == HTTP_STATUS_FOUND ||
                                    http_code == HTTP_STATUS_SEE_OTHER         || http_code == HTTP_STATUS_TEMPORARY_REDIRECT ||
                                    http_code == HTTP_STATUS_PERMANENT_REDIRECT))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP redirect: %03d\n", http_code);

        /*
         * Close the socket as we need to make a new request.
         */

        audio_client_disconnect(client, params);
        params->http_redirect = WICED_TRUE;
    }
    else if (result != WICED_SUCCESS || (http_code != HTTP_STATUS_OK && http_code != HTTP_STATUS_PARTIAL_CONTENT))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Bad HTTP status: %03d\n", http_code);
        if (params == &client->http_params)
        {
            client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)AUDIO_CLIENT_ERROR_HTTP_GET_ERROR);
        }
        return WICED_ERROR;
    }

    /*
     * Look through the HTTP headers for the Content-Length header. That tells us how
     * much data to expect.
     */

    ptr = params->http_buf;
    while (ptr != NULL && ptr < &params->http_buf[params->http_buf_idx])
    {
        line_end = strchr( (const char*)ptr, '\n');
        if (line_end == NULL)
        {
            break;
        }
        line_end[-1] = '\0';
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "%s\n", (char*)ptr);

        if (strncasecmp((const char *)ptr, HTTP_HEADER_CONTENT_LENGTH, sizeof(HTTP_HEADER_CONTENT_LENGTH) - 1) == 0)
        {
            ADVANCE_HEADER(ptr, sizeof(HTTP_HEADER_CONTENT_LENGTH));

            params->http_content_length = atol((char*)ptr);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP Content-Length: %ld\n", params->http_content_length);

            if ((params != &client->http_params) || (client->seek_in_progress == WICED_FALSE))
            {
                params->http_total_content_length = params->http_content_length;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP Total Content-Length: %ld\n", params->http_total_content_length);
            }
        }
        else if (strncasecmp((const char *)ptr, HTTP_HEADER_CONTENT_TYPE, sizeof(HTTP_HEADER_CONTENT_TYPE) - 1) == 0)
        {
            ADVANCE_HEADER(ptr, sizeof(HTTP_HEADER_CONTENT_TYPE));

            strlcpy(params->http_content_type, ptr, AUDIO_CLIENT_CONTENT_TYPE_SIZE);
            semi = strchr(params->http_content_type, ';');
            if (semi != NULL)
            {
                *semi = '\0';
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP Content-Type: %s\n", params->http_content_type);
        }
        else if (strncasecmp((const char *)ptr, HTTP_HEADER_ACCEPT_RANGES, sizeof(HTTP_HEADER_ACCEPT_RANGES) - 1) == 0)
        {
            /*
             * Do not try range request only when Server explicitly declares Accept-Ranges: none
             * General DLNA servers support range request, and we can check res protocolInfo info of meta data
             */

            ADVANCE_HEADER(ptr, sizeof(HTTP_HEADER_ACCEPT_RANGES));

            if (strncasecmp((const char *) ptr, HTTP_HEADER_NONE, sizeof(HTTP_HEADER_NONE) - 1) == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP server doesn't support range requests\n");
                params->http_range_requests = WICED_FALSE;
            }
        }
        else if (strncasecmp((const char *)ptr, HTTP_HEADER_TRANSFER_ENCODING, sizeof(HTTP_HEADER_TRANSFER_ENCODING) - 1) == 0)
        {
            ADVANCE_HEADER(ptr, sizeof(HTTP_HEADER_TRANSFER_ENCODING));

            if (strncasecmp((const char *)ptr, HTTP_HEADER_CHUNKED, sizeof(HTTP_HEADER_CHUNKED) - 1) == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP chunked transfer encoding\n");
                params->http_transfer_encoding_chunked = WICED_TRUE;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unsupported transfer encoding: %s\n", ptr);
                return WICED_ERROR;
            }
        }
        else if (strncasecmp((const char *)ptr, HTTP_HEADER_LOCATION, sizeof(HTTP_HEADER_LOCATION) - 1) == 0)
        {
            if (params->http_redirect)
            {
                ADVANCE_HEADER(ptr, sizeof(HTTP_HEADER_LOCATION));
                len              = strlen(ptr);
                len_original_uri = strlen(params->stream_uri);
                if (len <= len_original_uri)
                {
                    /*
                     * The new string is smaller in length. We can just copy over the old one.
                     */

                    strlcpy(params->stream_uri, ptr, len_original_uri + 1);
                }
                else if (params->stream_uri_storage == STREAM_URI_STORAGE_TYPE_PLAY_REQUEST)
                {
                    /*
                     * client->stream_uri is really a pointer to the URI in the current play node.
                     * Since the new URI is larger, we'll have to reallocate the node.
                     */

                    play_node = (audio_client_play_request_t*)realloc(client->current_play, sizeof(audio_client_play_request_t) + len + 1);
                    if (play_node == NULL)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate redirect URL\n");
                        return WICED_ERROR;
                    }

                    client->current_play  = play_node;
                    play_node->stream_uri = play_node->data;
                    params->stream_uri    = play_node->data;
                    strlcpy(params->stream_uri, ptr, len + 1);
                }
                else if (params->stream_uri_storage == STREAM_URI_STORAGE_TYPE_HLS_NODE)
                {
                    if (audio_client_hls_realloc_stream_uri(client, params, ptr, len) != WICED_SUCCESS)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls_realloc_stream_uri() failed!\n");
                        return WICED_ERROR;
                    }
                }
                else
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unsupported/unrecognized storage type for stream_uri: %d\n", params->stream_uri_storage);
                    return WICED_ERROR;
                }
            }
        }

        ptr = line_end + 1;
    }

    if (params->http_redirect)
    {
        if (params == &client->http_params)
        {
            client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_HTTP_REDIRECT, (void*)params->stream_uri);
        }

        /*
         * Send the main loop an event to start the connection process again.
         */

        wiced_rtos_set_event_flags(&params->http_events, HTTP_EVENT_CONNECT);
        return WICED_SUCCESS;
    }

    if (params->http_total_content_length == 0)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "HTTP Content-Length header not found in response...seek disabled\n");
    }

    /*
     * We need to check for a playlist mime type here. If we find one then we
     * need to convert the operation into a load file operation.
     */

    if ((params == &client->http_params) && client->params.load_playlist_files)
    {
        playlist = audio_client_check_for_playlist(params->http_content_type);
        if (playlist != NULL)
        {
            if (!client->params.disable_hls_streaming &&
                ((playlist->playlist_type == AUDIO_CLIENT_PLAYLIST_M3U8) || (playlist->playlist_type == AUDIO_CLIENT_PLAYLIST_M3U)))
            {
                /*
                 * We'll handle playback of HLS master and media playlist files directly
                 */
                client->hls_playlist_active = WICED_TRUE;
                params->http_load_file      = WICED_TRUE;
            }

            /*
             * Let the app know. They will let us know if we should load the file.
             * If we're already loading the file then we don't need to ask again.
             */

            if (client->hls_playlist_active == WICED_FALSE && !params->http_load_file)
            {
                result = client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYLIST_MIME_TYPE, (void*)playlist);
                if (result == WICED_SUCCESS)
                {
                    params->http_load_file = WICED_TRUE;
                }
            }
        }
    }

    if (params->http_load_file)
    {
        /*
         * We need to allocate memory for loading the file.
         */
        if (client->hls_playlist_active == WICED_FALSE)
        {
            if (params->http_total_content_length == 0)
            {
                params->http_content_length = AUDIO_CLIENT_DEFAULT_LOAD_FILE_SIZE;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "No content length specified for load file. Using default size of %d\n", params->http_content_length);
            }

            params->http_file_data = calloc(1, params->http_content_length + 1);
            if (params->http_file_data == NULL)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate memory for loading HTTP file\n");
                return WICED_ERROR;
            }
        }
        else
        {
            /*
             * Do not allocate memory in the HLS case
             */
            params->http_file_data = NULL;
        }
        params->http_file_idx = 0;
    }

    /*
     * Reset the header indices. We'll reuse the header buffer if we are dealing with chunked encoding.
     */

    params->http_header_idx = 0;
    params->http_buf_idx    = 0;

    /*
     * Look for a container type from mime or from special extension in URI
     */

    if (params == &client->http_params)
    {
        audio_client_look_for_container_type(client, params);
    }

    if ((client->filter_type == AUDIO_CLIENT_CONTAINER_NULL) && (params->http_load_file == WICED_FALSE))
    {
        /*
         * Tell the main thread that we have finished parsing the HTTP headers.
         */

        wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE);
    }

    return WICED_SUCCESS;
}


static wiced_result_t find_http_header(audio_client_t* client, audio_client_http_params_t* params, wiced_packet_t* packet)
{
    uint8_t*        data;
    uint16_t        avail_data_length;
    uint16_t        total_data_length;
    int             idx;
    wiced_result_t  result;

    /*
     * Get the pointer to the packet data.
     */

    result = wiced_packet_get_data(packet, 0, &data, &avail_data_length, &total_data_length);
    if ((result != WICED_SUCCESS) || (avail_data_length != total_data_length))
    {
        return WICED_ERROR;
    }

    idx = 0;
    if (!params->http_in_header)
    {
        /*
         * We're still searching for the beginning of the HTTP header.
         */

        while (idx < avail_data_length)
        {
            /*
             * Allow either HTTP 1.1 or 1.0. The response string has a ? where either
             * digit is allowed.
             * SHOUTcast can send it's own response header 'ICY 200 OK'
             */

            if ((data[idx] == audio_client_HTTP_response[params->http_header_idx] ||
                (audio_client_HTTP_response[params->http_header_idx] == '?' && isdigit(data[idx]))) ||
                (data[idx] == audio_client_ICY_response[params->http_header_idx]))
            {
                if (params->http_header_idx == 0 && data[idx] == audio_client_ICY_response[params->http_header_idx])
                {
                    params->http_icy_header = WICED_TRUE;
                }

                params->http_buf[params->http_buf_idx++] = data[idx];
                params->http_header_idx++;

                if ((!params->http_icy_header && audio_client_HTTP_response[params->http_header_idx] == '\0') ||
                    (params->http_icy_header  && audio_client_ICY_response[params->http_header_idx] == '\0'))
                {
                    /*
                     * Found the beginning of the header...woohoo!
                     */

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Found beginning of HTTP header\n");
                    params->http_in_header = WICED_TRUE;
                    idx++;
                    break;
                }
            }
            else
            {
                params->http_header_idx = 0;
                params->http_buf_idx    = 0;
                params->http_icy_header = WICED_FALSE;
            }
            idx++;
        }
    }

    if (!params->http_in_header)
    {
        /*
         * Haven't found the header yet.
         */

        return WICED_SUCCESS;
    }

    /*
     * Keep copying over the header data until we find the end.
     */

    while (idx < avail_data_length && params->http_buf_idx < AUDIO_CLIENT_HTTP_BUF_SIZE - 1)
    {
        params->http_buf[params->http_buf_idx++] = data[idx];
        if (data[idx] == audio_client_HTTP_body_separator[params->http_body_idx])
        {
            if (audio_client_HTTP_body_separator[++params->http_body_idx] == '\0')
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Found end of HTTP header\n");
                idx++;
                params->http_buf[params->http_buf_idx] = '\0';
                params->http_need_header = WICED_FALSE;
                params->http_in_header   = WICED_FALSE;
                break;
            }
        }
        else
        {
            params->http_body_idx = 0;
        }
        idx++;
    }

    /*
     * Update the beginning of the data in the packet to the beginning of the response body.
     */

    wiced_packet_set_data_start(packet, data + idx);
    if (params->http_buf_idx >= AUDIO_CLIENT_HTTP_BUF_SIZE - 1)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "HTTP buffer too small for entire header\n");
        return WICED_ERROR;
    }

    if (params->http_need_header)
    {
        /*
         * We haven't found all of the header yet.
         */

        return WICED_SUCCESS;
    }

    /*
     * Now that we have the header, parse it.
     */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "HTTP Response:\n%s\n", params->http_buf);
    result = audio_client_parse_http_header(client, params);

    if (result == WICED_SUCCESS)
    {
        /*
         * Reset some housekeeping variables.
         */

        params->http_content_read = 0;

        if ((params == &client->http_params) && client->seek_in_progress)
        {
            client->seek_in_progress = WICED_FALSE;
            client->decoders[client->audio_codec].decoder_ioctl(client, DECODER_IOCTL_SET_POSITION, (void*)client->seek_position);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Set position call: %lu\n", client->seek_position);
        }
    }

    return result;
}


static void send_empty_buffer_to_decoder(audio_client_t* client, audio_client_http_params_t* params)
{
    data_buf_t* dbuf;

    if (params->http_load_file)
    {
        /*
         * We're loading a file instead of sending the data to the decoder.
         */

        return;
    }

    /*
     * Tell the decoder that we're all done sending data by sending a zero length buffer.
     */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Send empty buffer to decoder\n");
    while (params->http_done == WICED_TRUE && !client->stop_in_progress)
    {
        dbuf = &client->data_bufs[client->data_buf_widx];

        if (dbuf->inuse == 0)
        {
            dbuf->buflen = 0;
            dbuf->inuse  = 1;
            client->data_buf_widx = (client->data_buf_widx + 1) % client->params.data_buffer_num;
            wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
            break;
        }

        wiced_rtos_delay_milliseconds(1);
    }
}


static wiced_result_t audio_client_http_seek(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t result;

    if (client->seek_in_progress)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Seek already in progress\n");
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Begin seek to position %lu\n", client->seek_position);

    /*
     * Disconnect the existing session to stop current http rx
     */

    result = audio_client_disconnect(client, params);
    if ( result != WICED_TCPIP_SUCCESS )
    {
        client->params.event_cb( client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*) AUDIO_CLIENT_ERROR_SEEK_ERROR );
        params->http_error = WICED_TRUE;
        return WICED_ERROR;
    }

    /*
     * Tell the decoder to flush any buffers it has waiting to be processed.
     */

    client->decoders[client->audio_codec].decoder_ioctl(client, DECODER_IOCTL_FLUSH, NULL);

    /*
     * Reconnect for range request
     */

    if ( audio_client_connect( client, params ) != WICED_SUCCESS )
    {
        client->params.event_cb( client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*) AUDIO_CLIENT_ERROR_CONNECT_FAILED );
        params->http_error = WICED_TRUE;
        return WICED_ERROR;
    }

    /*
     * Send off the GET range request to the HTTP server.
     * Do not include standard port in get request. Some radio stations don't like that.
     * (E.g. http://direct.franceinfo.fr/live/franceinfo-midfi.mp3)
     */

    if ((params->port == AUDIO_CLIENT_DEFAULT_HTTP_PORT) || (params->port == AUDIO_CLIENT_DEFAULT_HTTPS_PORT))
    {
        snprintf(params->http_buf, AUDIO_CLIENT_HTTP_BUF_SIZE, audio_client_HTTP_get_range_no_port_template, params->path, params->hostname, client->seek_position);
    }
    else
    {
        snprintf(params->http_buf, AUDIO_CLIENT_HTTP_BUF_SIZE, audio_client_HTTP_get_range_template, params->path, params->hostname, params->port, client->seek_position);
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Sending request:\n%s\n", params->http_buf);

    result = wiced_tcp_send_buffer(params->socket_ptr, params->http_buf, (uint16_t)strlen(params->http_buf));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to send HTTP request (%d)\n", result);
        client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)AUDIO_CLIENT_ERROR_HTTP_QUERY_FAILED);
        params->http_error = WICED_TRUE;
        return WICED_ERROR;
    }

    reset_initial_http_variables( client, params, WICED_TRUE );
    return WICED_SUCCESS;
}


static inline void audio_client_filter_data_buf(audio_client_t* client, audio_client_http_params_t* params, data_buf_t* dbuf, uint8_t* data, uint16_t data_length)
{
    wiced_result_t               result;
    audio_client_filter_buffer_t filter_buf;

    filter_buf.input_buffer         = data;
    filter_buf.input_buffer_length  = data_length;
    filter_buf.output_buffer        = dbuf->buf;
    filter_buf.output_buffer_length = sizeof(dbuf->buf);
    result = client->filters[client->filter_type].filter_ioctl(client, FILTER_IOCTL_PROCESS_DATA, &filter_buf);

    dbuf->buflen  = filter_buf.output_buffer_length;
    dbuf->bufused = 0;

    if (dbuf->buflen > 0)
    {
        dbuf->inuse = 1;
        client->data_buf_widx = (client->data_buf_widx + 1) % client->params.data_buffer_num;
    }
    else
    {
        /*
         * Filter hasn't written anything into the buffer; just release it back to the pool
         */

        dbuf->inuse = 0;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "%s: filter_ioctl() didn't write anything in the buffer\n", __FUNCTION__);
    }

    if ((result == WICED_UNSUPPORTED) && (client->audio_codec == AUDIO_CLIENT_CODEC_NULL))
    {
        /*
         * Filter reports that data are not supported; let the client know
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: filter_ioctl() for filter %d returned UNSUPPORTED\n", __FUNCTION__, (int)client->filter_type);
        wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE);
    }
    else if (client->filter_content_type[0] != '\0')
    {
        strlcpy(params->http_content_type, client->filter_content_type, sizeof(params->http_content_type));
        client->filter_content_type[0] = '\0';

        /*
         * Do not trigger HTTP_HEADER_COMPLETE event unless audio codec is still not set
         */

        if (client->audio_codec == AUDIO_CLIENT_CODEC_NULL)
        {
            wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE);
        }
    }
    else if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: filter_ioctl() for filter %d failed\n", __FUNCTION__, (int)client->filter_type);
    }
}


void write_data_buf(audio_client_t* client, audio_client_http_params_t* params, data_buf_t* dbuf, uint8_t* data, uint16_t data_length)
{
    if (client->filter_type != AUDIO_CLIENT_CONTAINER_NULL)
    {
        audio_client_filter_data_buf(client, params, dbuf, data, data_length);
    }
    else
    {
        memcpy(&dbuf->buf, data, data_length);
        dbuf->buflen  = data_length;
        dbuf->bufused = 0;
        dbuf->inuse   = 1;
        client->data_buf_widx = (client->data_buf_widx + 1) % client->params.data_buffer_num;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "write decoder buf: %u bytes, ridx %u, widx %u\n",
                  dbuf->buflen, client->data_buf_ridx, client->data_buf_widx);

    if (params->http_need_extra_header_check)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Ask for extra header check\n");
        params->http_need_extra_header_check = WICED_FALSE;
        wiced_rtos_set_event_flags(&client->events, AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE);
    }

    /*
     * Let the decoder know that there is data available.
     * If a preroll has been requested, queue up the first set of buffers before kicking
     * the decoder to help deal with network lag.
     * If we do have a preroll, check to make sure that it isn't greater than the total content length;
     * unless we're dealing with an HLS stream.
     */

    if (client->params.data_buffer_preroll == 0 || params->initial_buffer_count > client->params.data_buffer_preroll ||
        ((params->http_total_content_length == 0) && (client->params.no_length_disable_preroll)) ||
        ((client->hls_playlist_active == WICED_FALSE) &&
         (params->http_total_content_length > 0)  && (params->http_total_content_length < (client->params.data_buffer_preroll * AUDIO_CLIENT_DATA_BUF_SIZE))))
    {
        wiced_rtos_set_event_flags(&client->decoder_events, DECODER_EVENT_AUDIO_DATA);
        if (client->decoder_first_kick)
        {
            client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYBACK_STARTED, NULL);
            client->decoder_first_kick = WICED_FALSE;
        }

        /*
         * Do we need to send out a high buffer threshold event?
         */

        CHECK_FOR_THRESHOLD_HIGH_EVENT(client);
    }
    else
    {
        params->initial_buffer_count++;
    }
}


static wiced_result_t process_chunked_data(audio_client_t* client, audio_client_http_params_t* params, uint8_t* data, uint16_t data_length)
{
    data_buf_t* dbuf;
    char* ptr;
    int32_t chunk_size;
    wiced_bool_t end_of_chunk_size;
    uint32_t len;
    uint16_t size;
    uint16_t idx = 0;

    while (idx < data_length && !params->http_done)
    {
        while (idx < data_length && params->http_chunk_skip > 0)
        {
            idx++;
            params->http_chunk_skip--;
        }

        if (params->http_chunk_size == 0)
        {
            /*
             * We need to parse out the chunk size. Copy the chunk size data into our
             * temporary HTTP buffer. Necessary in case the chunk size spans multiple
             * network packets.
             */

            end_of_chunk_size = WICED_FALSE;
            while (idx < data_length && params->http_buf_idx < AUDIO_CLIENT_HTTP_BUF_SIZE - 1)
            {
                params->http_buf[params->http_buf_idx++] = data[idx];
                if (data[idx] == audio_client_HTTP_chunk_separator[params->http_body_idx])
                {
                    if (audio_client_HTTP_chunk_separator[++params->http_body_idx] == '\0')
                    {
                        end_of_chunk_size = WICED_TRUE;
                        idx++;
                        params->http_buf[params->http_buf_idx - (sizeof(audio_client_HTTP_chunk_separator) - 1)] = '\0';
                        break;
                    }
                }
                else
                {
                    params->http_body_idx = 0;
                }
                idx++;
            }

            /*
             * Have we finished grabbing the chunk size data?
             */

            if (end_of_chunk_size == WICED_FALSE)
            {
                if (params->http_buf_idx > AUDIO_CLIENT_HTTP_BUF_SIZE - 1)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error parsing chunk size\n");
                    params->http_body_idx = 0;
                }
                return WICED_SUCCESS;
            }

            /*
             * Check for a ';' in the string. That indicates the presence of chunk extensions which
             * we don't care about.
             */

            ptr = strchr( (const char*)params->http_buf, ';');
            if (ptr != NULL)
            {
                *ptr = 0;
            }

            if (string_to_signed((const char*)params->http_buf, strlen(params->http_buf), &chunk_size, 1) == 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error converting chunk size\n");
                return WICED_ERROR;
            }

            params->http_chunk_size = chunk_size;
            params->http_buf_idx    = 0;
            params->http_body_idx   = 0;
        }

        if (params->http_chunk_size == 0)
        {
            /*
             * Chunk size of 0 signals end of stream.
             */

            params->http_done = WICED_TRUE;
            return WICED_SUCCESS;
        }

        while (params->http_done == WICED_FALSE && idx < data_length)
        {
            size = MIN(data_length - idx, params->http_chunk_size - params->http_chunk_bytes);
            if (params->http_load_file)
            {
                len = size;
                if ((params->http_content_length > 0) && (params->http_file_idx + len > params->http_content_length))
                {
                    len = params->http_content_length - params->http_file_idx;
                }

                if (len > 0)
                {
                    if (client->hls_playlist_active == WICED_FALSE)
                    {
                        memcpy(&params->http_file_data[params->http_file_idx], &data[idx], len);
                    }
                    else
                    {
                        if (audio_client_hls_process_line_buffer(client, params, &data[idx], len) != WICED_SUCCESS)
                        {
                            return WICED_ERROR;
                        }
                    }
                    params->http_file_idx += len;
                }
            }
            else
            {
                dbuf = &client->data_bufs[client->data_buf_widx];

                if (dbuf->inuse)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Receive buffers full (%d)\n", client->data_buf_widx);
                    wiced_rtos_delay_milliseconds(4);
                    continue;
                }

                write_data_buf(client, params, dbuf, &data[idx], size);
            }
            idx += size;
            params->http_chunk_bytes += size;

            if (params->http_chunk_bytes == params->http_chunk_size)
            {
                /*
                 * Done with this chunk. We need to skip the trailing '\r\n' characters
                 * before starting on the next chunk.
                 */

                params->http_chunk_skip  = 2;
                params->http_chunk_size  = 0;
                params->http_chunk_bytes = 0;
            }
            break;
        }
    }

    return WICED_SUCCESS;
}


static wiced_result_t process_data(audio_client_t* client, audio_client_http_params_t* params, uint8_t* data, uint16_t data_length)
{
    data_buf_t* dbuf;
    uint32_t len;

    if (data_length == 0)
    {
        /*
         * If just the HTTP response was sent in the first packet, it's possible that we
         * don't have any data to process.
         */

        return WICED_SUCCESS;
    }

    if (params->http_transfer_encoding_chunked)
    {
        /*
         * Chunked data needs to be parsed rather than blindly copied.
         */

        return process_chunked_data(client, params, data, data_length);
    }

    if (params->http_load_file)
    {
        /*
         * If we are loading a file into memory, just copy the data into our buffer and return.
         */

        len = data_length;
        if ((params->http_content_length > 0) && (params->http_file_idx + len > params->http_content_length))
        {
            len = params->http_content_length - params->http_file_idx;
        }

        if (len > 0)
        {
            if (client->hls_playlist_active == WICED_FALSE)
            {
                memcpy(&params->http_file_data[params->http_file_idx], data, len);
            }
            else
            {
                if (audio_client_hls_process_line_buffer(client, params, data, len) != WICED_SUCCESS)
                {
                    return WICED_ERROR;
                }
            }
            params->http_file_idx += len;
        }

        return WICED_SUCCESS;
    }

    /*
     * Copy the packet data into a buffer to be processed.
     */

    while (params->http_done == WICED_FALSE)
    {
        dbuf = &client->data_bufs[client->data_buf_widx];

        if (dbuf->inuse)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Receive buffers full (%d)\n", client->data_buf_widx);
            wiced_rtos_delay_milliseconds(4);
            continue;
        }

        write_data_buf(client, params, dbuf, data, data_length);
        break;
    }

    return WICED_SUCCESS;
}


static void audio_client_http_process_socket(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_packet_t* packet = NULL;
    uint8_t*        data;
    uint16_t        offset;
    uint16_t        avail_data_length;
    uint16_t        total_data_length;
    uint32_t        events;
    wiced_result_t  result;

    if (params->socket_ptr == NULL)
    {
        return;
    }

    while (!params->http_done)
    {
        if ((params == &client->http_params) && client->params.high_threshold_read_inhibit && client->threshold_high_sent)
        {
            /*
             * No reading from the socket after the high threshold event is sent. This helps
             * the behavior when the app wants to put the WiFi to sleep between the high
             * and low threshold events.
             */

            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Skip read due to read inhibit setting\n");
            break;
        }

        result = wiced_rtos_wait_for_event_flags(&params->http_events, (HTTP_ALL_EVENTS & ~(HTTP_EVENT_TCP_DATA | HTTP_EVENT_TIMER)),
                                                 &events, WICED_FALSE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        if (result == WICED_SUCCESS && events != 0)
        {
            /*
             * We have a pending event notification. Bail out of the socket reading loop.
             */

            break;
        }

        /*
         * Read from the socket.
         */

        result = wiced_tcp_receive(params->socket_ptr, &packet, AUDIO_CLIENT_RECEIVE_TIMEOUT_MS);
        if (result != WICED_SUCCESS)
        {
            if (result == WICED_TIMEOUT || params->http_done)
            {
                break;
            }

            /*
             * Sad but true, some Internet streaming sites will send us a playlist with no
             * content length. So if we're doing a load file operation, have read data, and
             * have 0 total content length, then just treat this as a completed operation.
             */

            if (params->http_load_file && params->http_total_content_length == 0 && params->http_file_idx > 0)
            {
                params->http_done = WICED_TRUE;
                break;
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from socket receive (%d)\n", result);
            if (params == &client->http_params)
            {
                if ((client->hls_playlist_active == WICED_FALSE) ||
                    (client->hls_playlist_is_live == WICED_FALSE && client->hls_playlist_last_entry == WICED_TRUE) ||
                    (result == WICED_TLS_ERROR_OUT_OF_MEMORY))
                {
                    params->http_error = WICED_TRUE;
                    client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_ERROR, (void*)AUDIO_CLIENT_ERROR_HTTP_READ_ERROR);
                }
                else
                {
                    /*
                     * Move on to the next media segment if we're playing HLS
                     */
                    params->http_done = WICED_TRUE;
                }
            }
            else
            {
                params->http_error = WICED_TRUE;
            }
            break;
        }

        /*
         * Note the current time.
         */

        wiced_time_get_time(&params->http_last_packet_time);

        /*
         * Are we looking for the HTTP header?
         */

        if (params->http_need_header)
        {
            result = find_http_header(client, params, packet);
            if (result != WICED_SUCCESS)
            {
                params->http_error = WICED_TRUE;
                break;
            }

            if (params->http_redirect)
            {
                break;
            }

            if (params->http_need_header)
            {
                /*
                 * Still searching for the HTTP response header. Toss this packet and get the next one.
                 */

                wiced_packet_delete(packet);
                packet = NULL;
                continue;
            }
        }

        offset = 0;
        do
        {
            result = wiced_packet_get_data(packet, offset, &data, &avail_data_length, &total_data_length);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error getting packet data (%d)\n", result);
                params->http_error = WICED_TRUE;
                break;
            }

            if (avail_data_length > AUDIO_CLIENT_DATA_BUF_SIZE)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Receive buffers too small!!! %d (max %d)\n", avail_data_length, AUDIO_CLIENT_DATA_BUF_SIZE);
                params->http_error = WICED_TRUE;
                break;
            }
            params->http_content_read += avail_data_length;

            /*
             * Send the data onwards.
             */

            process_data(client, params, data, avail_data_length);

            offset += avail_data_length;
        } while (total_data_length > avail_data_length);

        /*
         * All done with the packet.
         */

        wiced_packet_delete(packet);
        packet = NULL;

        /*
         * Have we read all the data?
         */

        if (params->http_content_length > 0 && params->http_content_read >= params->http_content_length)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "All done reading HTTP data (%lu, %lu)\n", params->http_content_read, params->http_content_length);
            params->http_done = WICED_TRUE;
        }
    }

    if (packet != NULL)
    {
        wiced_packet_delete(packet);
        packet = NULL;
    }

    /*
     * Walk through HLS playlist (if HLS streaming is active)
     */
    audio_client_http_handle_hls_playlist(client, params);
}


static void audio_client_http_process_timer(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_time_t cur_time;

    if (client->state != AUDIO_CLIENT_STATE_PLAY)
    {
        return;
    }

    /*
     * Get the current time.
     */

    wiced_time_get_time(&cur_time);

    /*
     * Has it been too long without a packet?
     */

    if (cur_time > params->http_last_packet_time + HTTP_PACKET_TIMEOUT_MS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "%lu ms since last packet - disconnect initiated\n", cur_time - params->http_last_packet_time);
        params->http_error = WICED_TRUE;
    }
}


static void audio_client_http_handle_hls_playlist(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_bool_t make_request = WICED_FALSE;

    if ((client->hls_playlist_active == WICED_FALSE) ||
        (client->hls_playlist_done == WICED_TRUE) ||
        (client->stop_in_progress == WICED_TRUE))
    {
        return;
    }

    if (params == &client->hls_params)
    {
        if (params->http_error == WICED_TRUE)
        {
            audio_client_disconnect(client, params);
        }
        else if (params->http_done == WICED_TRUE)
        {
            audio_client_hls_process_reload(client, params);
            params->http_done = WICED_FALSE;
            audio_client_disconnect(client, params);
        }
        return;
    }
    else
    {
        if ((params->http_error == WICED_TRUE) || (params->http_done == WICED_FALSE))
        {
            return;
        }

        if (client->hls_playlist_parsing_complete == WICED_FALSE)
        {
            if (audio_client_hls_process_playlist(client, params) == WICED_SUCCESS)
            {
                make_request = WICED_TRUE;
            }
        }
        else if (audio_client_hls_get_next_playlist_entry(client, params) == WICED_SUCCESS)
        {
            make_request = WICED_TRUE;
        }
        else
        {
            if ((client->hls_playlist_is_live == WICED_FALSE) && (client->hls_playlist_last_entry == WICED_TRUE))
            {
                /*
                 * We've played the last segment in the HLS playlist
                 */
                audio_client_hls_flush_list(client);
            }
        }
    }

    if (make_request && !client->hls_playlist_done)
    {
        params->http_done = WICED_FALSE;
        audio_client_disconnect(client, params);
        wiced_rtos_set_event_flags(&params->http_events, HTTP_EVENT_CONNECT);
    }
}


static void audio_client_hls_thread(uint32_t arg)
{
    audio_client_http_params_t* hls_params = (audio_client_http_params_t*)arg;
    audio_client_t*             client     = hls_params->client;
    uint32_t                    events;
    wiced_result_t              result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client HLS thread begin\n");

    while (!hls_params->http_done && !AUDIO_CLIENT_HLS_IS_DONE(client))
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&hls_params->http_events, HTTP_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if ((result != WICED_SUCCESS) || hls_params->http_done || AUDIO_CLIENT_HLS_IS_DONE(client))
        {
            break;
        }

        if (events & HTTP_EVENT_CONNECT)
        {
            audio_client_connect_and_send_request(client, hls_params);
        }

        if (events & HTTP_EVENT_TIMER)
        {
            audio_client_http_process_timer(client, hls_params);
        }

        if (events & (HTTP_EVENT_TCP_DATA | HTTP_EVENT_DISCONNECT))
        {
            if (events & HTTP_EVENT_DISCONNECT)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client: Disconnect event\n");
            }
            audio_client_http_process_socket(client, hls_params);
        }
    }

    audio_client_disconnect(client, hls_params);

    /*
     * Make sure that any pending events are cleared out
     */

    wiced_rtos_wait_for_event_flags(&hls_params->http_events, HTTP_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client HLS thread ended.\n");

    WICED_END_OF_CURRENT_THREAD();
}


static void audio_client_http_thread(uint32_t arg)
{
    audio_client_http_params_t* http_params = (audio_client_http_params_t*)arg;
    audio_client_t*             client      = http_params->client;
    uint32_t                    events;
    uint32_t                    event;
    wiced_result_t              result;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client HTTP thread begin\n");

    /*
     * Make sure that any old events are cleared out and send ourselves an event
     * to connect to the HTTP server.
     */

    wiced_rtos_wait_for_event_flags(&http_params->http_events, HTTP_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
    wiced_rtos_set_event_flags(&http_params->http_events, HTTP_EVENT_CONNECT);

    while (!http_params->http_done && !http_params->http_error && !AUDIO_CLIENT_HLS_IS_DONE(client))
    {
        events = 0;

        result = wiced_rtos_wait_for_event_flags(&http_params->http_events, HTTP_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if ((result != WICED_SUCCESS) || http_params->http_done || AUDIO_CLIENT_HLS_IS_DONE(client))
        {
            break;
        }

        if (events & HTTP_EVENT_CONNECT)
        {
            if (audio_client_connect_and_send_request(client, http_params) != WICED_SUCCESS)
            {
                break;
            }
        }

        if (events & HTTP_EVENT_SEEK)
        {
            audio_client_http_seek(client, http_params);
        }

        if (events & HTTP_EVENT_TIMER)
        {
            audio_client_http_process_timer(client, http_params);
        }

        if (events & (HTTP_EVENT_TCP_DATA | HTTP_EVENT_DISCONNECT))
        {
            if (events & HTTP_EVENT_DISCONNECT)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client: Disconnect event\n");
            }
            audio_client_http_process_socket(client, http_params);
        }
    }

    /*
     * Destroy filter
     */

    audio_client_destroy_filter(client);

    /*
     * Tell the decoder that we're all done sending data.
     */

    send_empty_buffer_to_decoder(client, http_params);

    audio_client_disconnect(client, http_params);

    if (http_params->http_error)
    {
        event = AUDIO_CLIENT_EVENT_HTTP_ERROR;
    }
    else
    {
        event = AUDIO_CLIENT_EVENT_HTTP_THREAD_DONE;
    }
    wiced_rtos_set_event_flags(&client->events, event);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio client HTTP thread ended.\n");

    WICED_END_OF_CURRENT_THREAD();
}


wiced_result_t audio_client_http_reader_stop(audio_client_t* client, audio_client_http_params_t* params)
{
    if (client == NULL || params == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_http_reader_stop: Bad ARG\n");
        return WICED_BADARG;
    }

    if (params->http_thread_ptr != NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_http_reader_stop\n");

        if (params == &client->http_params)
        {
            audio_client_hls_stop(client);
        }
        params->http_done = WICED_TRUE;
        wiced_rtos_thread_force_awake(&params->http_thread);
        wiced_rtos_thread_join(&params->http_thread);
        wiced_rtos_delete_thread(&params->http_thread);
        params->http_thread_ptr = NULL;
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_client_http_reader_start(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t result;

    if (params->http_thread_ptr != NULL)
    {
        /*
         * We already have a reader thread running...that's bad.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Audio client HTTP thread already running (%lu)\n", params->http_content_read);
        return WICED_ERROR;
    }

    /*
     * Spawn off the HTTP reader thread.
     */

    reset_initial_http_variables(client, params, WICED_FALSE);

    if (params == &client->http_params)
    {
        client->data_buf_ridx = 0;
        client->data_buf_widx = 0;
        memset(client->data_bufs, 0, client->params.data_buffer_num * sizeof(data_buf_t));

        result = wiced_rtos_create_thread(&params->http_thread, AUDIO_CLIENT_HTTP_THREAD_PRIORITY, "Audio client HTTP thread",
                                          audio_client_http_thread, AUDIO_CLIENT_HTTP_THREAD_STACK_SIZE, params);
    }
    else if (params == &client->hls_params)
    {
        result = wiced_rtos_create_thread(&params->http_thread, AUDIO_CLIENT_HLS_THREAD_PRIORITY, "Audio client HLS thread",
                                          audio_client_hls_thread, AUDIO_CLIENT_HLS_THREAD_STACK_SIZE, params);
    }
    else
    {
        result = WICED_ERROR;
    }

    if (result == WICED_SUCCESS)
    {
        params->http_thread_ptr = &params->http_thread;
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create audio client HTTP thread\n");
    }

    return result;
}
