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
 * Audio Client Utility Routines
 *
 */

#include "wiced_result.h"
#include "wiced_utilities.h"

#include "audio_client_utils.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP_RESPONSE_LEN       13  /* 'HTTP/x.x xxx ' */
#define ICY_RESPONSE_LEN         8  /* 'ICY xxx '      */

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

static lookup_t codec_types[] =
{
    { ".flac",          AUDIO_CLIENT_CODEC_FLAC     },
    { ".wav",           AUDIO_CLIENT_CODEC_WAV      },
    { ".m4a",           AUDIO_CLIENT_CODEC_AAC_M4A  },
    { ".aac",           AUDIO_CLIENT_CODEC_AAC_ADTS },
    { ".mp3",           AUDIO_CLIENT_CODEC_MP3      },
    { NULL,             0                           }       /* Must be last! */
};

static audio_client_playlist_t audio_client_playlist_mime_types[] =
{
    { AUDIO_CLIENT_PLAYLIST_M3U,    "audio/x-mpegurl"               },
    { AUDIO_CLIENT_PLAYLIST_M3U8,   "application/vnd.apple.mpegurl" },
    { AUDIO_CLIENT_PLAYLIST_M3U8,   "application/x-mpegurl"         },
    { AUDIO_CLIENT_PLAYLIST_PLS,    "application/pls+xml"           },
    { AUDIO_CLIENT_PLAYLIST_PLS,    "audio/x-scpls"                 },
    { AUDIO_CLIENT_PLAYLIST_PLS,    "audio/scpls"                   },
    { -1,                           NULL                            }
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t audio_client_http_get_response_code(char* data, uint16_t data_length, uint16_t *code)
{
    wiced_bool_t icy_header;
    char* ptr;

    /* Check we have enough data to identify the response number */
    if (data_length < MAX(HTTP_RESPONSE_LEN, ICY_RESPONSE_LEN))
    {
        return WICED_ERROR;
    }

    icy_header = WICED_FALSE;

    /* Find the HTTP/x.x part*/
    ptr = strnstrn((const char*)data, data_length, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1);
    if (ptr == NULL)
    {
        /*
         * Winamp uses it's own header (ICY) instead of HTTP in the response.
         */

        ptr = strnstrn((const char*)data, data_length, ICY_HEADER_STR, sizeof(ICY_HEADER_STR) - 1);
        if (ptr == NULL)
        {
            return WICED_ERROR;
        }
        icy_header = WICED_TRUE;
    }

    if (icy_header)
    {
        /* Check that we have enough data before dereferencing it */
        if (((data + data_length) - ptr) < ICY_RESPONSE_LEN)
        {
           return WICED_ERROR;
        }

        /* Skip the "ICY" */
        ptr += 3;
    }
    else
    {
        /* Check that we have enough data before dereferencing it */
        if (((data + data_length) - ptr) < HTTP_RESPONSE_LEN)
        {
           return WICED_ERROR;
        }

        /* Skip the "HTTP/" and the version "x.x" */
        ptr += 5 + 3;
    }

    /* Verify next character is a space */
    if (*ptr++ != ' ')
    {
        return WICED_ERROR;
    }

    /* Verify response is 3 characters followed by a space */
    if (ptr[3] != ' ')
    {
        return WICED_ERROR;
    }

    *code = (uint16_t)atoi(ptr);

    return WICED_SUCCESS;
}


audio_client_playlist_t* audio_client_check_for_playlist(char* mime_type)
{
    int i;

    for (i = 0; audio_client_playlist_mime_types[i].mime_type != NULL; i++)
    {
        if (strncasecmp(audio_client_playlist_mime_types[i].mime_type, mime_type, strlen(audio_client_playlist_mime_types[i].mime_type)) == 0)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Playlist mime type found: %s\n", mime_type);

            return &audio_client_playlist_mime_types[i];
        }
    }

    return NULL;
}


AUDIO_CLIENT_CODEC_T audio_client_check_uri_for_audio_codec(char* uri)
{
    int extlen;
    int len;
    int i;

    len = strlen(uri);
    for (i = 0; codec_types[i].name != NULL; i++)
    {
        extlen = strlen(codec_types[i].name);
        if (strncasecmp(&uri[len - extlen], codec_types[i].name, extlen) == 0)
        {
            return codec_types[i].value;
        }
    }

    return AUDIO_CLIENT_CODEC_NULL;
}


AUDIO_CLIENT_CODEC_T audio_client_probe_for_audio_codec(audio_client_t* client, char* uri)
{
    AUDIO_CLIENT_CODEC_T codec;
    data_buf_t* dbuf;
    uint8_t* ptr;

    /*
     * It's possible to end up here when processing a HLS playlist and the URI extension
     * for the current entry will designate the codec. Check for that situation first.
     */

    codec = audio_client_check_uri_for_audio_codec(uri);

    if (codec != AUDIO_CLIENT_CODEC_NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Matched extension - using codec %d\n", codec);
        return codec;
    }

    /*
     * Do we have data to probe?
     */

    dbuf = &client->data_bufs[client->data_buf_ridx];
    if (!dbuf->inuse || dbuf->buflen < 12)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Not enough data to check (%d,%d)\n", dbuf->inuse, dbuf->buflen);
        return codec;
    }

    /*
     * We have data in the buffer. Can we identify it?
     */

    ptr = dbuf->buf;

    if (ptr[0] == 'R' && ptr[1] == 'I' && ptr[2]  == 'F' && ptr[3]  == 'F' &&
        ptr[8] == 'W' && ptr[9] == 'A' && ptr[10] == 'V' && ptr[11] == 'E')
    {
        /*
         * Assume that we have an WAVE file.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming WAV format\n");
        codec = AUDIO_CLIENT_CODEC_WAV;
    }
    else if (ptr[0] == 'f' && ptr[1] == 'L' && ptr[2] == 'a' && ptr[3] == 'C')
    {
        /*
         * Assume that we have an FLAC file.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming FLAC format\n");
        codec = AUDIO_CLIENT_CODEC_FLAC;
    }
    else if (ptr[4] == 'f' && ptr[5] == 't' && ptr[6] == 'y' && ptr[7] == 'p')
    {
        /*
         * Assume that we have an MP4 file.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming MP4 format\n");
        codec = AUDIO_CLIENT_CODEC_AAC_M4A;
    }
    else if (ptr[0] == 'I'  && ptr[1] == 'D' && ptr[2] == '3')
    {
        /*
         * Assume that we have an MP3.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming MP3 format\n");
        codec = AUDIO_CLIENT_CODEC_MP3;
    }
    else if ((ptr[0] == 0xFF) && ((ptr[1] & 0xF0) == 0xF0))
    {
        /*
         * First 12 bits are all 1's. Could be MP3 or could also be ADTS.
         * Check the layer bits...they should be 0 for ADTS.
         */

        if ((ptr[1] & 0x06) == 0)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming ADTS format\n");
            codec = AUDIO_CLIENT_CODEC_AAC_ADTS;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Assuming MP3 format\n");
            codec = AUDIO_CLIENT_CODEC_MP3;
        }
    }

    if (codec == AUDIO_CLIENT_CODEC_NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Unable to determine codec: %02x %02x %02x %02x\n", ptr[0], ptr[1], ptr[2], ptr[3]);
    }

    return codec;
}
