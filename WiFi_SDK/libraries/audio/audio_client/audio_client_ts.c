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

/** @file Audio Client TS Filtering Routines
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_log.h"
#include "wwd_assert.h"

#include "tsfilter.h"
#include "audio_client_private.h"
#include "audio_client_ts.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TS_TAG_VALID                          0x3852AF0D
#define TS_TAG_INVALID                        0xBEADFEEF

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t                      tag;
    tsfilter_ref                  handle;
    tsfilter_init_params_t        init_params;
    audio_client_filter_buffer_t* filter_buf;
    uint32_t                      output_buffer_index;
    wiced_bool_t                  found_unsupported_codec;
} ts_filter_t;

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

static wiced_result_t stream_type_cbf(tsfilter_stream_type_params_t* type_params)
{
    audio_client_t*      client = (audio_client_t*)type_params->user_context;
    ts_filter_t*         filter = (ts_filter_t *)client->filter_handle;
    AUDIO_CLIENT_CODEC_T codec  = AUDIO_CLIENT_CODEC_NULL;
    int                  i      = 0;

    switch (type_params->type)
    {
        case TSFILTER_AUDIO_TYPE_MPEG1:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_ts: we've got MPEG-1 audio payload\n");
            codec = AUDIO_CLIENT_CODEC_MP3;
            break;

        case TSFILTER_AUDIO_TYPE_MPEG2_BC:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_ts: we've got MPEG-2 BC audio payload\n");
            codec = AUDIO_CLIENT_CODEC_MP3;
            break;

        case TSFILTER_AUDIO_TYPE_AAC_ADTS:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client_ts: we've got ADTS AAC payload\n");
            codec = AUDIO_CLIENT_CODEC_AAC_ADTS;
            break;

        case TSFILTER_AUDIO_TYPE_MPEG4_LOAS:
            break;

        case TSFILTER_AUDIO_TYPE_PCM:
            break;

        case TSFILTER_AUDIO_TYPE_DD:
            break;

        case TSFILTER_AUDIO_TYPE_DTS:
            break;

        case TSFILTER_AUDIO_TYPE_DD_TRUEHD:
            break;

        case TSFILTER_AUDIO_TYPE_DDPLUS_BD:
            break;

        case TSFILTER_AUDIO_TYPE_DTSHD_HR:
            break;

        case TSFILTER_AUDIO_TYPE_DTSHD_MA:
            break;

        case TSFILTER_AUDIO_TYPE_DDPLUS_ATSC:
            break;

        default:
            break;
    }

    for (i = 0; (codec != AUDIO_CLIENT_CODEC_NULL) && (mime_types[i].name != NULL); i++)
    {
        if (codec == mime_types[i].value)
        {
            break;
        }
    }

    if ((codec == AUDIO_CLIENT_CODEC_NULL) || (mime_types[i].name == NULL))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_ts: codec is not supported !\n");
        filter->found_unsupported_codec = WICED_TRUE;
    }
    else
    {
        strlcpy(client->filter_content_type, mime_types[i].name, sizeof(client->filter_content_type));
    }

    return WICED_SUCCESS;
}


static wiced_result_t stream_data_cbf(tsfilter_stream_data_params_t* data_params)
{
    audio_client_t* client      = (audio_client_t*)data_params->user_context;
    ts_filter_t*    filter      = (ts_filter_t *)client->filter_handle;
    uint32_t        copy_length = MIN((filter->filter_buf->output_buffer_length - filter->output_buffer_index), data_params->data_length);

    if (copy_length < data_params->data_length)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "audio_client_ts: unable to copy entire payload: %lu/%lu\n", copy_length, data_params->data_length);
    }

    memcpy(&(filter->filter_buf->output_buffer[filter->output_buffer_index]), data_params->data, copy_length);
    filter->output_buffer_index += copy_length;

    return WICED_SUCCESS;
}


wiced_result_t audio_client_ts_filter_create(audio_client_t* client)
{
    wiced_result_t result = WICED_ERROR;
    ts_filter_t* filter;

    wiced_action_jump_when_not_true( client != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG\n", __FUNCTION__));

    /*
     * Allocate the internal filter structure.
     */

    filter = (ts_filter_t*)calloc(1, sizeof(ts_filter_t));
    wiced_action_jump_when_not_true(filter != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: unable to allocate TS filter structure\n", __FUNCTION__));

    filter->init_params.user_context    = client;
    filter->init_params.stream_data_cbf = stream_data_cbf;
    filter->init_params.stream_type_cbf = stream_type_cbf;

    result = tsfilter_init(&filter->init_params, &filter->handle);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: unable to initialize TS filter library\n", __FUNCTION__);
        free(filter);
        goto _exit;
    }

    client->filter_handle = filter;
    filter->tag = TS_TAG_VALID;

 _exit:
    return result;
}


wiced_result_t audio_client_ts_filter_destroy(audio_client_t* client)
{
    wiced_result_t  result = WICED_BADARG;
    ts_filter_t*    filter = NULL;

    wiced_action_jump_when_not_true(client != NULL, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG (NULL pointer)\n", __FUNCTION__));
    wiced_action_jump_when_not_true(client->filter_handle != NULL, _exit, result = WICED_SUCCESS);
    filter = (ts_filter_t*)client->filter_handle;
    wiced_action_jump_when_not_true(filter->tag == TS_TAG_VALID, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG (invalid client structure)\n", __FUNCTION__));

    filter->tag = TS_TAG_INVALID;
    result = tsfilter_deinit(filter->handle);
    free(client->filter_handle);
    client->filter_handle = NULL;

 _exit:
    return result;
}


wiced_result_t audio_client_ts_filter_ioctl(audio_client_t* client, FILTER_IOCTL_T ioctl, void* arg)
{
    wiced_result_t                result     = WICED_BADARG;
    ts_filter_t*                  filter     = NULL;
    audio_client_filter_buffer_t* filter_buf = (audio_client_filter_buffer_t*)arg;

    wiced_action_jump_when_not_true((client != NULL) && (client->filter_handle != NULL), _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG (NULL pointer)\n", __FUNCTION__));
    filter = (ts_filter_t*)client->filter_handle;
    wiced_action_jump_when_not_true(filter->tag == TS_TAG_VALID, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG (invalid client structure)\n", __FUNCTION__));

    switch (ioctl)
    {
        case FILTER_IOCTL_PROCESS_DATA:
            if ((filter_buf == NULL) || (filter_buf->input_buffer == NULL) || (filter_buf->output_buffer == NULL) ||
                (filter_buf->input_buffer_length == 0) || (filter_buf->output_buffer_length == 0))
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad ARG (invalid buffer params)\n", __FUNCTION__);
                goto _exit;
            }
            filter->filter_buf              = filter_buf;
            filter->output_buffer_index     = 0;
            filter->found_unsupported_codec = WICED_FALSE;
            result = tsfilter_parse(filter->handle, filter_buf->input_buffer, filter_buf->input_buffer_length);
            if (filter->found_unsupported_codec == WICED_TRUE)
            {
                result = WICED_UNSUPPORTED;
            }
            filter_buf->output_buffer_length = filter->output_buffer_index;
            filter->filter_buf               = NULL;
            filter->output_buffer_index      = 0;
            break;

        case FILTER_IOCTL_RESET_STATE:
            result = tsfilter_reset(filter->handle);
            break;
    }

 _exit:
    return result;
}
