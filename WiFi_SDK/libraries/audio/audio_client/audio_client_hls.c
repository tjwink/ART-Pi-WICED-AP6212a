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
 * Audio client library - HLS processing -
 *
 */

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "wiced_result.h"
#include "wiced_log.h"

#include "audio_client.h"
#include "audio_client_private.h"
#include "audio_client_utils.h"
#include "audio_client_hls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HLS_PLAYLIST_EXT              ".m3u8"
#define HLS_LEGACY_PLAYLIST_EXT       ".m3u"

#define HLS_PLAYLIST_TAG_STARTER      "#EXT"
#define HLS_PLAYLIST_TAG_HEADER       "#EXTM3U"

#define HLS_PLAYLIST_TOKEN_DELIM      ":,"
#define HLS_PLAYLIST_SUBTOKEN_DELIM   "="

#define HLS_PLAYLIST_TAG_BANDWIDTH    "BANDWIDTH"   /* #EXT-X-STREAM-INF    */
#define HLS_PLAYLIST_TAG_YES          "YES"         /* #EXT-X-ALLOW-CACHE   */
#define HLS_PLAYLIST_TAG_NO           "NO"          /* #EXT-X-ALLOW-CACHE   */
#define HLS_PLAYLIST_TAG_METHOD       "METHOD"      /* #EXT-X-KEY           */
#define HLS_PLAYLIST_TAG_NONE         "NONE"        /* #EXT-X-KEY           */
#define HLS_PLAYLIST_TAG_AES_128      "AES-128"     /* #EXT-X-KEY           */
#define HLS_PLAYLIST_TAG_SAMPLE_AES   "SAMPLE-AES"  /* #EXT-X-KEY           */
#define HLS_PLAYLIST_TAG_VOD          "VOD"         /* #EXT-X-PLAYLIST-TYPE */
#define HLS_PLAYLIST_TAG_EVENT        "EVENT"       /* #EXT-X-PLAYLIST-TYPE */

#define HLS_PLAYLIST_RELOAD_THRESHOLD  (4)
#define HLS_PLAYLIST_SHORT_THRESHOLD   (6)

#define HLS_PLAYLIST_WAIT_FLAG         (0x1)

#define HLS_PLAYLIST_MAX_ENTRY_COUNT   (30)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    /*
     * linked_list_node_t struct must be at the beginning/top of the struct;
     * data field of linked_list_node_t struct points back to dynamically allocated
     * audio_client_hls_playlist_node_t struct;
     */
    linked_list_node_t                       node;
    AUDIO_CLIENT_HLS_NODE_TYPE_T             type;          /* node type: playlist URI, media URI or unknown            */
    audio_client_hls_playlist_info_t         info;          /* details on the master/media playlists and media segments */
    char                                     uri[0];        /* NULL-terminated URI; must be last field                  */
} audio_client_hls_playlist_node_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static lookup_t hls_playlist_tags[] =
{
    { "#EXT-X-VERSION",        AUDIO_CLIENT_HLS_PLAYLIST_TAG_VERSION         },
    { "#EXT-X-ALLOW-CACHE",    AUDIO_CLIENT_HLS_PLAYLIST_TAG_ALLOW_CACHE     },     /* was removed in version 7 of the HLS protocol */
    { "#EXT-X-STREAM-INF",     AUDIO_CLIENT_HLS_PLAYLIST_TAG_STREAM_INF      },
    { "#EXT-X-TARGETDURATION", AUDIO_CLIENT_HLS_PLAYLIST_TAG_TARGET_DURATION },
    { "#EXT-X-MEDIA-SEQUENCE", AUDIO_CLIENT_HLS_PLAYLIST_TAG_MEDIA_SEQUENCE  },
    { "#EXT-X-KEY",            AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY             },
    { "#EXTINF",               AUDIO_CLIENT_HLS_PLAYLIST_TAG_INF             },
    { "#EXT-X-PLAYLIST-TYPE",  AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE            },
    { "#EXT-X-ENDLIST",        AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST         },
    { "#EXT-X-DISCONTINUITY",  AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY   },
    { NULL,                    AUDIO_CLIENT_HLS_PLAYLIST_TAG_UNKNOWN         },      /* Must be last! */
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static void audio_client_hls_reset_state(audio_client_t* client)
{
    client->hls_playlist_active           = WICED_FALSE;
    client->hls_playlist_parsing_complete = WICED_FALSE;
    client->hls_playlist_is_live          = WICED_FALSE;
    client->hls_playlist_is_reloading     = WICED_FALSE;
    client->hls_playlist_last_entry       = WICED_FALSE;
    client->hls_playlist_done             = WICED_FALSE;
    client->hls_playlist_needs_reload     = WICED_FALSE;
    client->hls_do_not_store_node         = WICED_FALSE;
    client->hls_base_uri_length           = 0;
    client->hls_start_offset_ms           = 0;
    client->hls_target_duration_ms        = 0;
    client->hls_entry_number              = 0;
    client->hls_playlist_count            = 0;
    client->hls_segment_count             = 0;
    client->hls_entry_number_reload       = 0;
    client->hls_node_current              = NULL;
    client->hls_node_previous             = NULL;
    client->hls_playlist_load_ts          = 0;
    client->hls_line_buffer_index         = 0;
    client->hls_node_type                 = AUDIO_CLIENT_HLS_NODE_TYPE_UNKNOWN;

    memset(&(client->hls_playlist_node_info), 0, sizeof(client->hls_playlist_node_info));
}


wiced_result_t audio_client_hls_init(audio_client_t* client)
{
    wiced_result_t result = WICED_SUCCESS;

    if (client->params.disable_hls_streaming == WICED_TRUE)
    {
        client->hls_playlist_active = WICED_FALSE;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio_client: disabling HTTP Live Streaming support\n");
    }
    else
    {
        /* keep reference to client */
        client->hls_params.client = client;

        if (client->params.hls_max_entry_count < HLS_PLAYLIST_MAX_ENTRY_COUNT)
        {
            client->params.hls_max_entry_count = HLS_PLAYLIST_MAX_ENTRY_COUNT;
        }

        audio_client_hls_reset_state(client);
        result  = linked_list_init(&client->hls_list_playlist);
        result |= linked_list_init(&client->hls_list_media);
        result |= linked_list_init(&client->hls_list_media_reload);
        result |= wiced_rtos_init_event_flags(&client->hls_wait_flag);
        result |= wiced_rtos_init_event_flags(&client->hls_params.http_events);
    }

    return result;
}


wiced_result_t audio_client_hls_deinit(audio_client_t* client)
{
    wiced_result_t result = WICED_SUCCESS;

    if (client->params.disable_hls_streaming == WICED_FALSE)
    {
        audio_client_hls_flush_list(client);
        result |= linked_list_deinit(&client->hls_list_playlist);
        result |= linked_list_deinit(&client->hls_list_media);
        result |= linked_list_deinit(&client->hls_list_media_reload);
        result |= wiced_rtos_deinit_event_flags(&client->hls_wait_flag);
        result |= wiced_rtos_deinit_event_flags(&client->hls_params.http_events);

        if (client->hls_params.socket_ptr != NULL)
        {
            wiced_tcp_delete_socket(client->hls_params.socket_ptr);
            client->hls_params.socket_ptr = NULL;
        }
    }

    return result;
}


wiced_result_t audio_client_hls_stop(audio_client_t* client)
{
    wiced_result_t result = WICED_SUCCESS;

    if (client->hls_playlist_active == WICED_TRUE)
    {
        client->hls_playlist_done = WICED_TRUE;
        result = wiced_rtos_set_event_flags(&client->hls_wait_flag, HLS_PLAYLIST_WAIT_FLAG);
    }

    return result;
}


wiced_bool_t audio_client_hls_is_playlist_active(audio_client_t* client)
{
    return client->hls_playlist_active;
}


static void audio_client_hls_empty_list(audio_client_t* client, linked_list_t* list_ptr)
{
    uint32_t count = 0;

    while ((linked_list_get_count(list_ptr, &count) == WICED_SUCCESS) && (count > 0))
    {
        linked_list_node_t* node = NULL;

        if (linked_list_remove_node_from_rear(list_ptr, &node) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't remove HLS list node\n");
            continue;
        }
        free(node->data);
    }
}


void audio_client_hls_flush_list(audio_client_t* client)
{
    uint32_t events;

    UNUSED_PARAMETER(events);

    audio_client_http_reader_stop(client, &client->hls_params);
    free(client->hls_playlist_stream_uri);
    client->hls_playlist_stream_uri = NULL;
    audio_client_hls_reset_state(client);
    audio_client_hls_empty_list(client, &client->hls_list_media_reload);
    audio_client_hls_empty_list(client, &client->hls_list_media);
    audio_client_hls_empty_list(client, &client->hls_list_playlist);
    wiced_rtos_wait_for_event_flags(&client->hls_wait_flag, HLS_PLAYLIST_WAIT_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
}


wiced_result_t audio_client_hls_realloc_stream_uri(audio_client_t* client, audio_client_http_params_t* params, char* new_uri, uint32_t new_uri_length)
{
    wiced_result_t                    result   = WICED_ERROR;
    audio_client_hls_playlist_node_t* hls_node = NULL;
    audio_client_hls_playlist_node_t* hls_node_new = NULL;

    if (client->hls_node_current == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: current node is NULL\n");
        goto _exit;
    }

    hls_node = (audio_client_hls_playlist_node_t*)(client->hls_node_current->data);

    if (hls_node->uri != params->stream_uri)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: current node uri doesn't match stream_uri\n");
        goto _exit;
    }

    hls_node_new = realloc(hls_node, sizeof(audio_client_hls_playlist_node_t) + new_uri_length + 1);
    if (hls_node_new == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: realloc failed\n");
        result = WICED_OUT_OF_HEAP_SPACE;
        goto _exit;
    }

    if (&hls_node_new->node != &hls_node->node)
    {
        if (hls_node_new->node.prev != NULL)
        {
            hls_node_new->node.prev->next = &hls_node_new->node;
        }

        if (hls_node_new->node.next != NULL)
        {
            hls_node_new->node.next->prev = &hls_node_new->node;
        }
    }

    strlcpy(hls_node_new->uri, new_uri, new_uri_length + 1);
    params->stream_uri             = hls_node_new->uri;
    client->hls_node_current       = &hls_node_new->node;
    client->hls_node_current->data = hls_node_new;
    result                         = WICED_SUCCESS;

 _exit:
    return result;
}


static wiced_bool_t linked_list_compare_node_uri_cbf(linked_list_node_t* node_to_compare, void* user_data)
{
    wiced_bool_t    rc     = WICED_FALSE;
    audio_client_t* client = (audio_client_t*)user_data;

    if ((node_to_compare != NULL) && (node_to_compare->data != NULL))
    {
        audio_client_hls_playlist_node_t* hls_node;
        audio_client_hls_playlist_node_t* hls_node_to_play;
        uint32_t uri_compare_length;
        uint32_t uri_to_play_length;

        hls_node           = (audio_client_hls_playlist_node_t*)node_to_compare->data;
        hls_node_to_play   = (audio_client_hls_playlist_node_t*)client->hls_node_current->data;
        uri_compare_length = strlen(hls_node->uri);
        uri_to_play_length = strlen(hls_node_to_play->uri);

        if ((uri_compare_length == uri_to_play_length) && (strncmp(hls_node->uri, hls_node_to_play->uri, uri_to_play_length) == 0))
        {
            rc = WICED_TRUE;
        }
    }

    return rc;
}


static wiced_result_t audio_client_hls_parse_tag(char *line, uint32_t line_length, audio_client_hls_playlist_info_t *info)
{
    wiced_result_t                  result           = WICED_SUCCESS;
    uint32_t                        i;
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_T tag_id;
    uint32_t                        tag_length;
    char*                           token;
    char*                           subtoken;
    char*                           saveptr_token;
    char*                           saveptr_subtoken;
    char*                           endptr;

    token = strtok_r(line, HLS_PLAYLIST_TOKEN_DELIM, &saveptr_token);
    if (token == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get HLS playlist tag\n");
        result = WICED_ERROR;
        goto _exit;
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: TAG=[%s]\n", token);

    tag_id = AUDIO_CLIENT_HLS_PLAYLIST_TAG_UNKNOWN;

    for (i = 0; hls_playlist_tags[i].name != NULL; i++)
    {
        tag_length = strlen(hls_playlist_tags[i].name);

        if ((strlen(token) == tag_length) && (strncmp(hls_playlist_tags[i].name, token, tag_length) == 0))
        {
            tag_id = hls_playlist_tags[i].value;
            break;
        }
    }

    if (tag_id == AUDIO_CLIENT_HLS_PLAYLIST_TAG_UNKNOWN)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: unrecognized TAG=[%s]\n", token);
        goto _exit;
    }

    if (tag_id == AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST)
    {
        info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST;
        goto _exit;
    }

    if (tag_id == AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY)
    {
        info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY;
        goto _exit;
    }

    token = strtok_r(NULL, HLS_PLAYLIST_TOKEN_DELIM, &saveptr_token);
    if (token == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get HLS playlist tag value\n");
        result = WICED_ERROR;
        goto _exit;
    }
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: VALUE=[%s]\n", token);

    switch (tag_id)
    {
        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_UNKNOWN:
        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST:
        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY:
            /* we should never land there ! */
            result = WICED_ERROR;
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_VERSION:
            info->version = strtoul(token, &endptr, 0);
            if (*endptr != '\0')
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get HLS protocol version\n");
            }
            else
            {
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_VERSION;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: version=%lu\n", info->version);
            }
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_ALLOW_CACHE:
            if (strncmp(token, HLS_PLAYLIST_TAG_YES, strlen(HLS_PLAYLIST_TAG_YES)) == 0)
            {
                info->allow_cache = WICED_TRUE;
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_ALLOW_CACHE;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: allow-cache=%d\n", (int)info->allow_cache);
            }
            else if (strncmp(token, HLS_PLAYLIST_TAG_NO, strlen(HLS_PLAYLIST_TAG_NO)) == 0)
            {
                info->allow_cache = WICED_FALSE;
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_ALLOW_CACHE;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: allow-cache=%d\n", (int)info->allow_cache);
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: unknown allow-cache directive %s\n", token);
            }
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_STREAM_INF:
            do
            {
                saveptr_subtoken = NULL;
                subtoken = strtok_r(token, HLS_PLAYLIST_SUBTOKEN_DELIM, &saveptr_subtoken);
                if ((subtoken != NULL) && (strncmp(subtoken, HLS_PLAYLIST_TAG_BANDWIDTH, strlen(HLS_PLAYLIST_TAG_BANDWIDTH)) == 0))
                {
                    subtoken = strtok_r(NULL, HLS_PLAYLIST_SUBTOKEN_DELIM, &saveptr_subtoken);
                    if (subtoken != NULL)
                    {
                        info->bandwidth = strtoul(subtoken, &endptr, 0);
                        if (*endptr != '\0')
                        {
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get stream info (bandwidth)\n");
                        }
                        else
                        {
                            info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_STREAM_INF;
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: bandwidth=%lu\n", info->bandwidth);
                        }
                        break;
                    }
                }

                token = strtok_r(NULL, HLS_PLAYLIST_TOKEN_DELIM, &saveptr_token);
            } while (token != NULL);
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_TARGET_DURATION:
            info->target_duration = strtoul(token, &endptr, 0);
            if (*endptr != '\0')
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get target duration\n");
            }
            else
            {
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_TARGET_DURATION;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_hls: target-duration=%lu\n", info->target_duration);
            }
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_MEDIA_SEQUENCE:
            info->media_sequence = strtoul(token, &endptr, 0);
            if (*endptr != '\0')
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get media sequence\n");
            }
            else
            {
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_MEDIA_SEQUENCE;
                if (info->media_sequence > 0)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio_client_hls: media_sequence=%lu\n", info->media_sequence);
                }
            }
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY:
            do
            {
                saveptr_subtoken = NULL;
                subtoken = strtok_r(token, HLS_PLAYLIST_SUBTOKEN_DELIM, &saveptr_subtoken);
                if ((subtoken != NULL) && (strncmp(subtoken, HLS_PLAYLIST_TAG_METHOD, strlen(HLS_PLAYLIST_TAG_METHOD)) == 0))
                {
                    subtoken = strtok_r(NULL, HLS_PLAYLIST_SUBTOKEN_DELIM, &saveptr_subtoken);
                    if (subtoken != NULL)
                    {
                        if (strncmp(subtoken, HLS_PLAYLIST_TAG_NONE, strlen(HLS_PLAYLIST_TAG_NONE)) == 0)
                        {
                            info->encrypted = WICED_FALSE;
                            info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY;
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: encrypted=%d\n", (int)info->encrypted);
                        }
                        else if ((strncmp(subtoken, HLS_PLAYLIST_TAG_AES_128, strlen(HLS_PLAYLIST_TAG_AES_128)) == 0) ||
                                 (strncmp(subtoken, HLS_PLAYLIST_TAG_SAMPLE_AES, strlen(HLS_PLAYLIST_TAG_SAMPLE_AES)) == 0))
                        {
                            info->encrypted = WICED_TRUE;
                            info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY;
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: encrypted=%d\n", (int)info->encrypted);
                        }
                        else
                        {
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: unknown encryption method %s\n", subtoken);
                        }
                        break;
                    }
                }

                token = strtok_r(NULL, HLS_PLAYLIST_TOKEN_DELIM, &saveptr_token);
            } while (token != NULL);
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_INF:
            info->segment_duration = strtod(token, &endptr);
            if (*endptr != '\0')
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: can't get media segment duration\n");
            }
            else
            {
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_INF;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG3, "audio_client_hls: segment_duration=%f/%d\n", info->segment_duration, (int)info->segment_duration);
            }
            break;

        case AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE:
            if (strncmp(token, HLS_PLAYLIST_TAG_VOD, strlen(HLS_PLAYLIST_TAG_VOD)) == 0)
            {
                info->type = AUDIO_CLIENT_HLS_PLAYLIST_TYPE_VOD;
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE;
            }
            else if (strncmp(token, HLS_PLAYLIST_TAG_EVENT, strlen(HLS_PLAYLIST_TAG_EVENT)) == 0)
            {
                info->type = AUDIO_CLIENT_HLS_PLAYLIST_TYPE_EVENT;
                info->tags |= AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE;
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: unknown playlist type %s\n", token);
            }
            break;
    }

 _exit:
    return result;
}


wiced_result_t audio_client_hls_process_line_buffer(audio_client_t* client, audio_client_http_params_t* params, uint8_t* data, uint32_t data_length)
{
    const uint32_t                    tag_starter_length = strlen(HLS_PLAYLIST_TAG_STARTER);
    const uint32_t                    tag_header_length  = strlen(HLS_PLAYLIST_TAG_HEADER);
    wiced_result_t                    result             = WICED_SUCCESS;
    uint8_t*                          ptr;
    audio_client_hls_playlist_node_t* hls_node;
    char*                             uri_ptr            = NULL;
    uint32_t                          extra;
    wiced_bool_t                      use_base;
    audio_client_playlist_t*          playlist;

    if ((client->hls_playlist_active == WICED_FALSE) ||
        ((client->hls_playlist_parsing_complete == WICED_TRUE) && (client->hls_playlist_needs_reload == WICED_FALSE)))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: not an HLS playlist or HLS playlist processing has already completed\n");
        result = WICED_ERROR;
        goto _exit;
    }

    ptr = data;

    if (params->http_file_idx == 0)
    {
        if ((data_length <= tag_header_length) || (strncmp((char*)ptr, HLS_PLAYLIST_TAG_HEADER, tag_header_length) != 0))
        {
            /*
             * If the playlist does not start with #EXTM3U then it's not a valid HLS playlist
             */

            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: HLS playlist is not valid (%s)\n", params->http_content_type);
            playlist = audio_client_check_for_playlist(params->http_content_type);
            if (playlist != NULL)
            {
                /*
                 * Is the app interested in this file?
                 */

                result = client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_PLAYLIST_MIME_TYPE, (void*)playlist);
                if (result == WICED_SUCCESS)
                {
                    /*
                     * Yes, give it to them. If we bail out here, the normal HTTP shutdown processing will
                     * send the playlist to the app.
                     */

                    client->hls_playlist_active = WICED_FALSE;

                    if (params->http_total_content_length == 0)
                    {
                        params->http_content_length = AUDIO_CLIENT_DEFAULT_LOAD_FILE_SIZE;
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "audio_client_hls: no content length specified for load file. Using default size of %d\n",
                                      params->http_content_length);
                    }

                    params->http_file_data = calloc(1, params->http_content_length + 1);
                    if (params->http_file_data == NULL)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: unable to allocate memory for loading HTTP file\n");
                        result = WICED_ERROR;
                    }
                    else
                    {
                        memcpy(&params->http_file_data[params->http_file_idx], data, data_length);
                        result = WICED_SUCCESS;

                    }
                    goto _exit;
                }
                result = WICED_SUCCESS;
            }
        }

        if ((client->hls_playlist_count == 0) && (client->hls_segment_count == 0))
        {
            /*
             * Keep track of original URI
             */
            if (client->hls_playlist_stream_uri == NULL)
            {
                client->hls_playlist_stream_uri = strdup(params->stream_uri);
            }

            /*
             * Keep track of client->start_offset_ms
             */
            client->hls_start_offset_ms = client->start_offset_ms;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: start_offset_ms=%lu\n", client->hls_start_offset_ms);
        }
    }

    do
    {

        while ((client->hls_line_buffer_index < sizeof(client->hls_line_buffer)) && (ptr < (data + data_length)) && (*ptr != '\r') && (*ptr != '\n'))
        {
            client->hls_line_buffer[client->hls_line_buffer_index] = *ptr;
            client->hls_line_buffer_index++;
            ptr++;
        }

        if (client->hls_line_buffer_index == sizeof(client->hls_line_buffer))
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: playlist line buffer is too small (%lu)\n", sizeof(client->hls_line_buffer));
            result = WICED_ERROR;
            break;
        }

        if (ptr == (data + data_length))
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing needs more data\n");
            break;
        }

        client->hls_line_buffer[client->hls_line_buffer_index] = '\0';
        client->hls_line_buffer_index++;

#ifdef WITH_AUDIO_CLIENT_HLS_TRACE
        wiced_log_printf("%s\n", (char*)client->hls_line_buffer);
#endif

        if (client->hls_line_buffer[0] == '#')
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing found comment line\n");

            if ((client->hls_line_buffer_index > tag_starter_length) && (strncmp((char*)client->hls_line_buffer, HLS_PLAYLIST_TAG_STARTER, tag_starter_length) == 0))
            {
                /*
                 * Valid HLS playlist tag
                 */

                audio_client_hls_parse_tag((char*)client->hls_line_buffer, client->hls_line_buffer_index - 1, &(client->hls_playlist_node_info));

                if (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing got TAG_ENDLIST\n");
                }
            }
        }
        else if (!isspace((int)client->hls_line_buffer[0]))
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing found URL\n");

            if (client->hls_line_buffer_index > 1)
            {
                linked_list_t*               list_ptr   = NULL;
                AUDIO_CLIENT_HLS_NODE_TYPE_T node_type  = AUDIO_CLIENT_HLS_NODE_TYPE_UNKNOWN;

                /*
                 * Determine node type (either a playlist or a media segment)
                 */

                if ((strstr((char*)client->hls_line_buffer, HLS_PLAYLIST_EXT) != NULL) ||
                    (strstr((char*)client->hls_line_buffer, HLS_LEGACY_PLAYLIST_EXT) != NULL) ||
                    (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_STREAM_INF))
                {
                    node_type = AUDIO_CLIENT_HLS_NODE_TYPE_PLAYLIST;
                }
                else
                {
                    /*
                     * If it's not a playlist then we assume it's a media file.
                     */

                    node_type = AUDIO_CLIENT_HLS_NODE_TYPE_MEDIA;

                    if (client->hls_node_type == AUDIO_CLIENT_HLS_NODE_TYPE_PLAYLIST)
                    {
                        /* if last node type was PLAYLIST, then reset entry count */
                        client->hls_entry_number = 0;

                        client->hls_target_duration_ms = 0;
                        if (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_TARGET_DURATION)
                        {
                            client->hls_target_duration_ms = client->hls_playlist_node_info.target_duration * 1000UL;
                        }
                        /*
                         * For the purpose of starting playback at a specific time offset,
                         * we are going to assume that all media segments share the same target duration
                         * (although, HLS specs does say this DOESN'T have to be the case)
                         */
                        if ((client->hls_target_duration_ms > 0) && (client->hls_start_offset_ms > client->hls_target_duration_ms) && (client->hls_entry_number_reload == 0))
                        {
                            client->hls_entry_number_reload  = ((client->hls_start_offset_ms / client->hls_target_duration_ms) * client->hls_target_duration_ms);
                            client->hls_entry_number_reload /= client->hls_target_duration_ms;
                        }
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: starting from segment #%lu (start_offset_ms=%lu)\n",
                                      (client->hls_entry_number_reload + 1), client->hls_start_offset_ms);
                    }
                }

                /*
                 * Determine in which list we're going to store/retrieve the node to/from
                 */

                if (params == &client->hls_params)
                {
                    list_ptr = &client->hls_list_media_reload;
                }
                else
                {
                    if (node_type == AUDIO_CLIENT_HLS_NODE_TYPE_PLAYLIST)
                    {
                        list_ptr = &client->hls_list_playlist;
                    }
                    else
                    {
                        list_ptr = &client->hls_list_media;
                    }
                }

                /*
                 * Update node entry number
                 */

                client->hls_entry_number++;
                client->hls_playlist_node_info.entry_number = client->hls_entry_number;

                /*
                 * Determine whether or not we're actually going to store that node
                 */

                if (node_type == AUDIO_CLIENT_HLS_NODE_TYPE_MEDIA)
                {
                    uint32_t media_count = 0;
                    linked_list_get_count(list_ptr, &media_count);

                    if (client->hls_do_not_store_node == WICED_FALSE)
                    {
                        if (client->hls_entry_number <= client->hls_entry_number_reload)
                        {
                            client->hls_do_not_store_node = WICED_TRUE;
                        }

                        if (media_count >= client->params.hls_max_entry_count)
                        {
                            client->hls_do_not_store_node = WICED_TRUE;
                            client->hls_entry_number_reload = client->hls_entry_number - 1;
                        }

                        if (client->hls_do_not_store_node)
                        {
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: do_not_store=%d,count=%03lu,current=%03lu,reload=%03lu\n",
                                    client->hls_do_not_store_node, media_count, client->hls_entry_number, client->hls_entry_number_reload);
                        }
                    }
                    else
                    {
                        if ((media_count < client->params.hls_max_entry_count) && (client->hls_entry_number > client->hls_entry_number_reload))
                        {
                            client->hls_do_not_store_node = WICED_FALSE;
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: do_not_store=%d,count=%03lu,current=%03lu,reload=%03lu\n",
                                    client->hls_do_not_store_node, media_count, client->hls_entry_number, client->hls_entry_number_reload);
                        }
                    }
                }

                /*
                 * Do we have a full path or a relative path?
                 */

                if (client->hls_do_not_store_node == WICED_FALSE)
                {
                    if ((strncasecmp("http:", (char*)client->hls_line_buffer, 5) == 0) || (strncasecmp("https:", (char*)client->hls_line_buffer, 6) == 0))
                    {
                        extra    = client->hls_line_buffer_index;
                        use_base = WICED_FALSE;
                    }
                    else
                    {
                        if (client->hls_base_uri_length == 0)
                        {
                            /*
                             * We need to figure out the base length.
                             */

                            uri_ptr = &params->stream_uri[strlen(params->stream_uri)];
                            while (uri_ptr > params->stream_uri && *uri_ptr != '/')
                            {
                                uri_ptr--;
                            }
                            if (uri_ptr > params->stream_uri)
                            {
                                client->hls_base_uri_length = (uint32_t)(uri_ptr - params->stream_uri) + 1;
                            }
                        }
                        extra    = client->hls_line_buffer_index + client->hls_base_uri_length;
                        use_base = WICED_TRUE;
                    }

                    hls_node = calloc(1, sizeof(audio_client_hls_playlist_node_t) + extra);
                    if (hls_node == NULL)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: out of memory\n");
                        result = WICED_OUT_OF_HEAP_SPACE;
                        break;
                    }

                    if (use_base && client->hls_base_uri_length > 0)
                    {
                        memcpy(hls_node->uri, params->stream_uri, client->hls_base_uri_length);
                        strlcpy(&hls_node->uri[client->hls_base_uri_length], (char*)client->hls_line_buffer, client->hls_line_buffer_index);
                    }
                    else
                    {
                        strlcpy(hls_node->uri, (char*)client->hls_line_buffer, client->hls_line_buffer_index);
                    }

                    /*
                     * Copy HLS tag info and add new node to list
                     */

                    hls_node->type = node_type;
                    memcpy(&hls_node->info, &client->hls_playlist_node_info, sizeof(client->hls_playlist_node_info));
                    hls_node->node.data = hls_node;
                    linked_list_insert_node_at_rear(list_ptr, &hls_node->node);
                }

                /*
                 * Reset HLS tag info flags
                 */

                if (node_type == AUDIO_CLIENT_HLS_NODE_TYPE_MEDIA)
                {
                    /*
                     * Only reset flags for tags that are specific to media segments:
                     * EXTINF
                     * EXT-X-BYTERANGE
                     * EXT-X-DISCONTINUITY
                     * EXT-X-KEY
                     * EXT-X-MAP
                     * EXT-X-PROGRAM-DATE-TIME
                     * EXT-X-DATERANGE
                     */
                    client->hls_playlist_node_info.tags &= ~AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY;
                    client->hls_playlist_node_info.tags &= ~AUDIO_CLIENT_HLS_PLAYLIST_TAG_INF;
                    client->hls_playlist_node_info.tags &= ~AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY;
                }
                else
                {
                    /*
                     * Reset all the flags as we're going from a playlist to a media segment
                     */
                    client->hls_playlist_node_info.tags = 0;
                }

                client->hls_node_type = node_type;
            }
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing empty line\n");
        }

        client->hls_line_buffer_index = 0;

        while ((ptr < (data + data_length)) && ((*ptr == '\r') || (*ptr == '\n')))
        {
            ptr++;
        }

        if (ptr == (data + data_length))
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "audio_client_hls: playlist processing needs more data(2)\n");
            break;
        }

    } while (WICED_TRUE);

 _exit:
    return result;
}


wiced_result_t audio_client_hls_process_reload(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t result             = WICED_SUCCESS;
    uint32_t       media_reload_count = 0;
    wiced_time_t   tick;

    if ((client->hls_playlist_active == WICED_FALSE) ||
        (client->hls_playlist_needs_reload == WICED_FALSE))
    {
        result = WICED_ERROR;
        goto _exit;
    }

    wiced_time_get_time(&tick);
    linked_list_get_count(&client->hls_list_media_reload, &media_reload_count);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: reloaded %lu segments in %lu msecs\n",
                  media_reload_count, (uint32_t)(tick - client->hls_playlist_load_ts));
    result = wiced_rtos_set_event_flags(&client->hls_wait_flag, HLS_PLAYLIST_WAIT_FLAG);

 _exit:
    return result;
}


wiced_result_t audio_client_hls_process_playlist(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t                    result             = WICED_SUCCESS;
    linked_list_node_t*               node;
    linked_list_node_t*               node_to_play;
    audio_client_hls_playlist_node_t* hls_node;
    uint32_t                          playlist_count     = 0;
    uint32_t                          media_count        = 0;
    uint32_t                          media_reload_count = 0;

    if ((client->hls_playlist_active == WICED_FALSE) ||
        (client->hls_playlist_parsing_complete == WICED_TRUE))
    {
        result = WICED_ERROR;
        goto _exit;
    }

    linked_list_get_count(&client->hls_list_playlist, &playlist_count);
    linked_list_get_count(&client->hls_list_media, &media_count);
    linked_list_get_count(&client->hls_list_media_reload, &media_reload_count);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: playlist=%lu, media=%lu, reload=%lu, so far\n",
                  playlist_count, media_count, media_reload_count);

    if ((playlist_count > 0) && (media_count == 0) && (media_reload_count == 0))
    {
        client->hls_playlist_count = client->hls_entry_number;

        if ((linked_list_get_front_node(&client->hls_list_playlist, &node) != WICED_SUCCESS) ||
            (((audio_client_hls_playlist_node_t*)node->data)->type == AUDIO_CLIENT_HLS_NODE_TYPE_UNKNOWN))
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not retrieve first valid playlist entry\n");
            result = WICED_ERROR;
            goto _exit;
        }

        hls_node = (audio_client_hls_playlist_node_t*)(node->data);
        params->stream_uri            = hls_node->uri;
        client->hls_base_uri_length   = 0;
        client->start_offset_ms       = 0;
        params->http_load_file        = WICED_TRUE;
        params->stream_uri_storage    = STREAM_URI_STORAGE_TYPE_HLS_NODE;
        client->hls_do_not_store_node = WICED_FALSE;
    }
    else if (media_count > 0)
    {
        client->hls_segment_count = client->hls_entry_number;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: %lu playlist, %lu/%lu segments\n",
                      client->hls_playlist_count, media_count, client->hls_segment_count);

        if (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE)
        {
            if (client->hls_playlist_node_info.type == AUDIO_CLIENT_HLS_PLAYLIST_TYPE_VOD)
            {
                client->hls_playlist_is_live = WICED_FALSE;
            }
            else if (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST)
            {
                client->hls_playlist_is_live = WICED_FALSE;
            }
            else
            {
                client->hls_playlist_is_live = WICED_TRUE;
            }
        }
        else
        {
            if (client->hls_playlist_node_info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST)
            {
                client->hls_playlist_is_live = WICED_FALSE;
            }
            else
            {
                client->hls_playlist_is_live = WICED_TRUE;
            }
        }

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: playlist is %s\n", client->hls_playlist_is_live ? "LIVE" : "*NOT* LIVE");

        /*
         * If HLS stream is not live and we don't have the last media segment in our list
         * then we'll need to reload later on
         */

        if (client->hls_playlist_is_live == WICED_FALSE)
        {
            if (linked_list_get_rear_node(&client->hls_list_media, &node) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not get last entry of media list\n");
                result = WICED_ERROR;
                goto _exit;
            }

            hls_node = (audio_client_hls_playlist_node_t*)(node->data);
            if (hls_node->info.entry_number < client->hls_segment_count)
            {
                if (audio_client_http_reader_start(client, &client->hls_params) != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not start HLS playlist reload thread\n");
                    result = WICED_ERROR;
                    goto _exit;
                }
            }
        }

        if (linked_list_get_front_node(&client->hls_list_media, &node) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not get first entry of media list\n");
            result = WICED_ERROR;
            goto _exit;
        }

        hls_node = (audio_client_hls_playlist_node_t*)(node->data);

        if (client->hls_playlist_is_reloading == WICED_TRUE)
        {
            if (client->hls_playlist_is_live == WICED_TRUE)
            {
                if (client->hls_node_current == NULL)
                {
                    /*
                     * If current node is NULL, use previous node
                     */
                    client->hls_node_current = client->hls_node_previous;
                }

                if (client->hls_node_current != NULL)
                {
                    /*
                     * Need to find the new node that matches the node we were about to play
                     */

                    if (linked_list_find_node(&client->hls_list_media, linked_list_compare_node_uri_cbf, client, &node_to_play) != WICED_SUCCESS)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not find a matching entry in the new playlist\n");
                        client->hls_node_previous = NULL;
                    }
                    else
                    {
                        if (client->hls_node_previous != NULL)
                        {
                            /*
                             * If we've been using the previous node,
                             * we have to start playback from the NEXT node
                             */
                            node = node_to_play->next;
                            client->hls_node_previous = NULL;
                        }
                        else
                        {
                            node = node_to_play;
                        }

                        if (node == NULL)
                        {
                            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: no node to play\n");
                            free(client->hls_node_current->data);
                            client->hls_node_current = NULL;
                            result = audio_client_hls_get_next_playlist_entry(client, params);
                            goto _exit;
                        }

                        hls_node = (audio_client_hls_playlist_node_t*)(node->data);
                    }

                    /*
                     * We need to free the current node here
                     */
                    free(client->hls_node_current->data);
                }
            }
        }
        else
        {
            /* Make sure EVENT_PLAYBACK_STARTED is only kicked once */
            client->decoder_first_kick   = WICED_TRUE;
            params->initial_buffer_count = 0;

            /* Reset HTTP content type string */
            params->http_content_type[0] = '\0';
            /* Reset high/low threshold flags */
            client->threshold_high_sent  = WICED_FALSE;
            client->threshold_low_sent   = WICED_FALSE;
        }

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: about to play segment #%lu\n", hls_node->info.entry_number);

        params->stream_uri                    = hls_node->uri;
        client->start_offset_ms               = 0;
        params->http_load_file                = WICED_FALSE;
        params->stream_uri_storage            = STREAM_URI_STORAGE_TYPE_HLS_NODE;

        client->hls_node_current              = node;
        client->hls_playlist_parsing_complete = WICED_TRUE;

        /*
         * Playlist load timestamp
         */
        wiced_time_get_time(&client->hls_playlist_load_ts);
    }

 _exit:
    return result;
}


wiced_result_t audio_client_hls_get_next_playlist_entry(audio_client_t* client, audio_client_http_params_t* params)
{
    wiced_result_t                    result        = WICED_SUCCESS;
    audio_client_hls_playlist_node_t* hls_node      = NULL;
    linked_list_node_t*               node_playlist = NULL;
    uint32_t                          media_count   = 0;

    if ((client->hls_playlist_active == WICED_FALSE) ||
        ((client->hls_playlist_is_live == WICED_FALSE) && (client->hls_node_current == NULL)))
    {
        result = WICED_ERROR;
        goto _exit;
    }

    if (client->hls_node_current != NULL)
    {
        client->hls_node_current = client->hls_node_current->next;

        if (client->hls_node_current != NULL)
        {
            hls_node = (audio_client_hls_playlist_node_t*)(client->hls_node_current->data);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: about to play segment #%lu\n", hls_node->info.entry_number);
        }
    }

    if (client->hls_node_current == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "audio_client_hls: no entry left to play\n");
        client->hls_playlist_last_entry = WICED_TRUE;
        if (client->hls_playlist_is_live == WICED_FALSE)
        {
            result = WICED_ERROR;
            goto _exit;
        }
    }
    else if (client->hls_node_current->next == NULL)
    {
        if (client->hls_playlist_needs_reload == WICED_FALSE)
        {
            client->hls_playlist_last_entry = WICED_TRUE;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "audio_client_hls: playing last entry\n");
        }
        else
        {
            uint32_t            events = 0;
            linked_list_node_t* node   = NULL;
            wiced_time_t        tick;
            wiced_time_t        tack;

            if (client->hls_params.http_error == WICED_FALSE)
            {
                wiced_time_get_time(&tick);
                wiced_rtos_wait_for_event_flags(&client->hls_wait_flag, HLS_PLAYLIST_WAIT_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, client->hls_target_duration_ms);
                wiced_time_get_time(&tack);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: waited %lu msecs for reloaded segments\n", (uint32_t)(tack - tick));
            }

            if (client->hls_playlist_done == WICED_TRUE)
            {
                result = WICED_ERROR;
                goto _exit;
            }

            /* remove all segments ahead of current node */
            while (linked_list_get_front_node(&client->hls_list_media, &node) == WICED_SUCCESS)
            {
                if (client->hls_node_current == node)
                {
                    break;
                }
                if (linked_list_remove_node(&client->hls_list_media, node) != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not trim node from  hls_list_media\n");
                    continue;
                }
                free(node->data);
            }

            /* add new reloaded segments, IF ANY, to media list */
            while (linked_list_remove_node_from_front(&client->hls_list_media_reload, &node) == WICED_SUCCESS)
            {
                if (linked_list_insert_node_at_rear(&client->hls_list_media, node) != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not add new reload segments to media list\n");
                }
            }

            /*
             * Reset reload flag
             */

            client->hls_playlist_needs_reload = WICED_FALSE;
        }
    }

    linked_list_get_count(&client->hls_list_media, &media_count);

    if (client->hls_playlist_is_live == WICED_TRUE)
    {
        client->hls_playlist_is_reloading = WICED_FALSE;

        if (media_count >= HLS_PLAYLIST_SHORT_THRESHOLD)
        {
            if ((hls_node == NULL) ||
                (hls_node->info.entry_number > (media_count - HLS_PLAYLIST_RELOAD_THRESHOLD)))
            {
                client->hls_playlist_is_reloading = WICED_TRUE;
            }

        }
        else if ((client->hls_playlist_last_entry == WICED_TRUE) || (hls_node == NULL))
        {
            client->hls_playlist_is_reloading = WICED_TRUE;
        }

        /*
         * If we're playing live and we're left with hls_reload_thresh chunks to play, or less,
         * we have to reload the playlist to get fresh media segments
         */

        if (client->hls_playlist_is_reloading == WICED_TRUE)
        {
            wiced_time_t        tick;
            uint32_t            wait_time;
            uint32_t            events;

            UNUSED_PARAMETER(events);

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: reloading playlist...\n");

            /*
             * Grab the playlist node
             */

            if (linked_list_get_front_node(&client->hls_list_playlist, &node_playlist) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not get first playlist node\n");
                goto _play_next_entry;
            }

            /*
             * Remove node we were about to play, if any; we'll refer to it later on
             */

            if (client->hls_node_current != NULL)
            {
                if (linked_list_remove_node(&client->hls_list_media, client->hls_node_current) != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not remove the node we were about to play\n");
                    goto _play_next_entry;
                }
            }
            else if (client->hls_node_previous == NULL)
            {
                if (linked_list_remove_node_from_rear(&client->hls_list_media, &client->hls_node_previous) != WICED_SUCCESS)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not remove the last node\n");
                }
            }

            /*
             * Remove all nodes in the list
             */

            audio_client_hls_empty_list(client, &client->hls_list_media);

            /*
             * Make sure we don't reload the playlist too often
             */

            wiced_time_get_time(&tick);
            wait_time = tick - client->hls_playlist_load_ts;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: waited %lu/%lu msecs so far\n",
                          wait_time, client->hls_target_duration_ms);

            if ((client->hls_target_duration_ms + 1000UL) > wait_time)
            {
                wait_time = (client->hls_target_duration_ms + 1000UL) - wait_time;
            }
            else
            {
                wait_time = 0;
            }
            wiced_rtos_wait_for_event_flags(&client->hls_wait_flag, HLS_PLAYLIST_WAIT_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, wait_time);

            wiced_time_get_time(&tick);
            wait_time = tick - client->hls_playlist_load_ts;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_hls: waited %lu/%lu total\n",
                          wait_time, (client->hls_target_duration_ms + 1000UL));

            if (client->hls_playlist_done == WICED_TRUE)
            {
                result = WICED_ERROR;
                goto _exit;
            }

            hls_node = (audio_client_hls_playlist_node_t*)(node_playlist->data);
            params->stream_uri                    = hls_node->uri;
            client->hls_base_uri_length           = 0;
            client->start_offset_ms               = 0;
            params->http_load_file                = WICED_TRUE;
            params->stream_uri_storage            = STREAM_URI_STORAGE_TYPE_HLS_NODE;
            client->hls_playlist_parsing_complete = WICED_FALSE;
            client->hls_playlist_last_entry       = WICED_FALSE;
            /*
             * Reset entry number here
             */
            client->hls_entry_number              = 0;
            goto _exit;
        }
    }
    else if (client->hls_playlist_needs_reload == WICED_FALSE)
    {
        linked_list_node_t*               last_node     = NULL;
        audio_client_hls_playlist_node_t* hls_last_node = NULL;

        if (media_count == 0)
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: media list is empty\n");
            goto _exit;
        }

        if (media_count >= client->hls_segment_count)
        {
            /* media list contains all the segments in the playlist; no need to fetch new segments */
            goto _play_next_entry;
        }

        if (linked_list_get_rear_node(&client->hls_list_media, &last_node) != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not get last media node\n");
            goto _play_next_entry;
        }

        hls_last_node = (audio_client_hls_playlist_node_t*)(last_node->data);

        if (hls_last_node->info.entry_number == client->hls_segment_count)
        {
            /* media list contains the last segment of the playlist; no need to reload */
            goto _play_next_entry;
        }

        if (hls_node->info.entry_number > (hls_last_node->info.entry_number - HLS_PLAYLIST_RELOAD_THRESHOLD))
        {
            /*
             * We're reloading; grab playlist node
             */

            if (linked_list_get_front_node(&client->hls_list_playlist, &node_playlist) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: could not get first playlist node\n");
                goto _play_next_entry;
            }

            hls_node = (audio_client_hls_playlist_node_t*)(node_playlist->data);
            client->hls_params.stream_uri         = hls_node->uri;
            client->hls_base_uri_length           = 0;
            client->hls_params.http_load_file     = WICED_TRUE;
            client->hls_params.stream_uri_storage = STREAM_URI_STORAGE_TYPE_HLS_NODE;
            client->hls_playlist_last_entry       = WICED_FALSE;
            client->hls_playlist_needs_reload     = WICED_TRUE;
            client->hls_do_not_store_node         = WICED_FALSE;
            /*
             * Reset entry number here
             */
            client->hls_entry_number              = 0;

            /* signal HLS-specific HTTP reader thread */
            wiced_rtos_set_event_flags(&client->hls_params.http_events, HTTP_EVENT_CONNECT);
            /* get a timestamp */
            wiced_time_get_time(&client->hls_playlist_load_ts);
        }
    }

 _play_next_entry:
    if (client->hls_node_current == NULL)
    {
        result = WICED_ERROR;
    }
    else
    {
        hls_node = (audio_client_hls_playlist_node_t*)(client->hls_node_current->data);
        if (hls_node->type != AUDIO_CLIENT_HLS_NODE_TYPE_MEDIA)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: that's NOT a media URI\n");
            result = WICED_ERROR;
            goto _exit;
        }

        if (hls_node->info.tags & AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY)
        {
            /*
             * Media segment bears a DISCONTINUITY flag;
             * something about audio encoding may have changed;
             * simply issue a warning and do not enforce for now
             */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_hls: segment bears a DISCONTINUITY tag\n");
        }

        params->stream_uri         = hls_node->uri;
        client->start_offset_ms    = 0;
        params->http_load_file     = WICED_FALSE;
        params->stream_uri_storage = STREAM_URI_STORAGE_TYPE_HLS_NODE;
    }

 _exit:
    return result;
}


wiced_result_t audio_client_hls_suspend(audio_client_t* client, audio_client_suspend_t** suspend)
{
    wiced_result_t                    result     = WICED_SUCCESS;
    uint32_t                          uri_length = 0;
    audio_client_hls_playlist_node_t* hls_node   = NULL;

    if ((client->state == AUDIO_CLIENT_STATE_IDLE) ||
        (client->hls_playlist_active == WICED_FALSE) ||
        (client->hls_playlist_stream_uri == NULL))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid state for suspend: %d, hls active: %d\n",
                      client->state, (int)client->hls_playlist_active, client->hls_playlist_stream_uri);
        result = WICED_ERROR;
        goto _exit;
    }

    uri_length = strlen(client->hls_playlist_stream_uri);

    if ((*suspend = calloc(1, sizeof(audio_client_suspend_t) + uri_length + 1)) == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate suspend structure\n");
        result = WICED_OUT_OF_HEAP_SPACE;
        goto _exit;
    }

    /*
     * Cache the information we need to resume playback.
     */

    (*suspend)->uri          = (char*)((*suspend)->data);
    strlcpy((*suspend)->uri, client->hls_playlist_stream_uri, uri_length + 1);
    (*suspend)->live_stream  = client->hls_playlist_is_live;
    (*suspend)->codec        = client->audio_codec;
    (*suspend)->time_playing = 0;

    if (client->hls_playlist_is_live == WICED_FALSE)
    {
        if (client->hls_node_current != NULL)
        {
            hls_node = (audio_client_hls_playlist_node_t*)(client->hls_node_current->data);
        }
        else if (client->hls_node_previous != NULL)
        {
            hls_node = (audio_client_hls_playlist_node_t*)(client->hls_node_previous->data);
        }

        if (hls_node != NULL)
        {
            (*suspend)->time_playing = (hls_node->info.entry_number - 1) * client->hls_target_duration_ms;
        }
        else
        {
            /*
             * Current and previous nodes are not set;
             * we may not have been fully ready for playback when the suspend directive was issued.
             * Use the start_offset_ms from the original playback request.
             */
            (*suspend)->time_playing = client->hls_start_offset_ms;
        }
    }

  _exit:
    return result;
}
