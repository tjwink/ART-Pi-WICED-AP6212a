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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "apollocore.h"
#include "apollo_cmd_common.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_CMD_RTP_TIMING_NONE  = 0,
    APOLLO_CMD_RTP_TIMING_INIT  = 1,
    APOLLO_CMD_RTP_TIMING_START = 2,
    APOLLO_CMD_RTP_TIMING_STOP  = 3,
    APOLLO_CMD_RTP_TIMING_RESET = 4,
    APOLLO_CMD_RTP_TIMING_DUMP  = 5,

    APOLLO_CMD_RTP_TIMING_MAX
} APOLLO_CMD_RTP_TIMING_T;

typedef enum
{
    APOLLO_CMD_EVENT_QUERY_SPEAKER,
    APOLLO_CMD_EVENT_SET_SPEAKER,
    APOLLO_CMD_EVENT_SET_VOLUME,
    APOLLO_CMD_EVENT_SET_LOG_SERVER,
    APOLLO_CMD_EVENT_SET_LOG_LEVEL,
    APOLLO_CMD_EVENT_RTP_TIMING,
    APOLLO_CMD_EVENT_METADATA,
    APOLLO_CMD_EVENT_AMBILIGHT,
} APOLLO_CMD_EVENT_T;

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * RTP timing log command event structure used for APOLLO_CMD_EVENT_RTP_TIMING events.
 */

typedef struct
{
    APOLLO_CMD_RTP_TIMING_T cmd;
    uint32_t num_entries;                   /* Only valid for APOLLO_CMD_RTP_TIMING_INIT command */
} apollo_cmd_rtp_timing_t;

/**
 * Speaker structure used with QUERY_SPEAKER and GET_SPEAKER events.
 * For QUERY_SPEAKER events, the callback routine is
 * expected to fill in the structure elements.
 * For SET_SPEAKER events, the apollo command component
 * is passing the received information to the callback.
 */

typedef struct apollo_cmd_speaker_s
{
    char *speaker_name;                     /* Speaker name string (not nul terminated) */
    int speaker_name_len;                   /* Length of speaker name string            */
    APOLLO_CHANNEL_MAP_T speaker_channel;   /* Speaker channel mask                     */
} apollo_cmd_speaker_t;


/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef wiced_result_t (*apollo_cmd_callback_t)(void* handle, void* userdata, APOLLO_CMD_EVENT_T event, void* arg);

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize the Apollo command library.
 *
 * @param[in] interface   : Interface for the command listener socket.
 * @param[in] userdata    : Userdata pointer passed back in event callback.
 * @param[in] callback    : Callback handler for command events.
 *
 * @return Pointer to the command instance or NULL
 */
void* apollo_cmd_init(wiced_interface_t interface, void* userdata, apollo_cmd_callback_t callback);


/** Deinitialize the Apollo command library.
 *
 * @param[in] handle  : handle to the command instance.
 *
 * @return    Status of the operation.
 */
wiced_result_t apollo_cmd_deinit(void *handle);

#ifdef __cplusplus
} /* extern "C" */
#endif
