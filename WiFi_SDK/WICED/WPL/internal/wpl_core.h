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
 */
#pragma once

#include <stdint.h>
#include <string.h>
#include "wiced_defaults.h"
#include "wiced_result.h"
#include "wpl_packet.h"
#include "wiced_power_logger.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                      WPL-Macros
 ******************************************************/
#define WPL_SYN_DATA                                   0x5A5A
#define WPL_SYN_DATA_SIZE                              2

#define WPL_SYN_BYTE1                                  0x5A
#define WPL_SYN_BYTE2                                  0x5A

#define POWER_LOGGER_HEADER_SIZE                       3

#define TARGET_WPL_VERSION                             "v1"

#define WPL_POLL_MULTIPLY_FACTOR                       10

//PAD-WPL Commands
typedef enum {
    CMD_LOG_POLL_PERIOD,
    CMD_TARGET_DETECTION,
    CMD_GET_TARGET_WPL_VERSION,
    CMD_GET_PROCESSOR_LIST,
    CMD_GET_EVENTS_LIST,
    CMD_GET_EVENT_DESCRIPTOR_LIST,
    CMD_START_LOGGING,
    CMD_STOP_LOGGING,
    CMD_STOP_PAD = 'L',
    CMD_LOG_REQUEST = 0x56,
} pad_commands_t;

//CPL callback commands
typedef enum {
    CPL_GET_EVENTS_LIST,
    CPL_GET_EVENTS_DESC_LIST,
    CPL_START_LOG,
    CPL_STOP_LOG,
    CPL_SEND_LOG_REQUEST,
} cpl_callback_commands_t;

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef wiced_result_t ( *cpl_callback_handler_t )( uint8_t cmd_id, uint32_t* len, uint8_t* in_data, uint8_t** out_data );
typedef uint32_t ( *cpl_get_timestamp_handler_t )( void );

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
        uint8_t proc_count;                     /* Number of processor ids */
        uint8_t* proc_id;                       /* List of processor ids */
        uint32_t log_size;                      /* Size of log data */
        cpl_callback_handler_t cpl_callback;
}cpl_data_t;


/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wpl_register_cpl( cpl_data_t* cpl );
wiced_result_t wpl_init( void );
wiced_result_t packets_from_wpl_host_handler( wpl_packet_t* packet );
wiced_result_t wpl_send_pad_cmd( uint8_t packet_byte );
wiced_bool_t wpl_logging_status( void );
void wpl_set_deep_sleep_state( wiced_bool_t state );
//extern uint32_t wpl_get_time_stamp( void );

#ifdef __cplusplus
} /* extern "C" */
#endif


