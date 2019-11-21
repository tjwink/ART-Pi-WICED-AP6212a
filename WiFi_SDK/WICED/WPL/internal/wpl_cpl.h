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
/*******************************************************************************
 * All common definitions for Platform specific CPL
 *******************************************************************************/

#ifndef _PLATFORM_CPL_H_
#define _PLATFORM_CPL_H_

#include "wiced_power_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    uint8_t proc_id;
    uint8_t event_id;
    uint8_t event_desc;
} event_lookup_table_entry_t;

typedef struct
{
    uint8_t event_id;
    uint8_t *event_desc_list;
    uint8_t lut_event_desc_cnt;
    uint8_t max_event_desc;
    wiced_bool_t log_enabled;
    uint8_t current_event_desc;
    uint32_t previous_time_stamp;
}cpl_event_id_struct_t;

typedef struct
{
    uint8_t proc_id;
    cpl_event_id_struct_t *event_data;
    uint8_t *event_list;
    uint8_t lut_event_id_cnt;
    uint8_t max_event_id;
}cpl_proc_id_struct_t;

typedef struct
{
    cpl_proc_id_struct_t *proc_data;
    uint8_t *proc_list;
    uint8_t lut_proc_id_cnt;
    uint8_t max_proc_id;
}cpl_state_ctrl_t;

//8.4.4    Power Log Event Format (Architecture Document)
//Processor ID    Event ID    Event Descriptor    Time Stamp/Power Data    Extended Data
//8 Bits                  8 Bits    8 Bits                  32 Bits                         Byte0(Length, N Bytes), Byte1, Byte2, …., ByteN
#pragma pack(1)
typedef struct
{
    uint8_t proc_id;
    uint8_t event_id;
    uint8_t event_desc;
    uint32_t event_duration;
}cpl_packet_t;

typedef struct
{
    uint32_t log_count;
    cpl_packet_t *cpl_log_packet;
}cpl_log_buffer_t;
#pragma pack()

/******************************************************
 *               Function Declarations
 ******************************************************/
void wpl_cpl_init( void );
#ifndef WICED_POWER_LOGGER_ENABLE
void wpl_cpl_init( void )
{
}
#endif

void wpl_wifi_get_power_data(void);

//For individual event updates, where time duration is not considered, used in case of Wi-Fi power data
void cpl_log_update( uint8_t proc_id, uint8_t event_id, uint8_t event_state, uint32_t event_data );
void cpl_log_reset_event( uint8_t proc_id, uint8_t event_id, uint8_t event_state );

void cpl_set_powerstate( uint8_t proc_id, uint8_t event_id, uint8_t event_state );
#ifdef __cplusplus
extern "C" }
#endif

#endif
