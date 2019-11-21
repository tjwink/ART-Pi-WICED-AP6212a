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


#ifdef __cplusplus
extern "C" {
#endif

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
    WPL_PAD_REQUEST,
    WPL_PAD_RESPONSE
}wpl_packet_group_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct wpl_packet      wpl_packet_t;

/******************************************************
 *                    Structures
 ******************************************************/
#pragma pack( 1 )
struct wpl_packet
{
    uint8_t             packet_group;
    uint8_t             packet_id;
    uint32_t            packet_size;
    uint8_t*            payload_start;
    uint8_t             packet_start[1];
};

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t wpl_dynamic_allocate_packet( wpl_packet_t** packet, wpl_packet_group_t pkt_group, uint8_t id, uint32_t data_size );
wiced_result_t wpl_free_packet( wpl_packet_t* packet );

#ifdef __cplusplus
} /* extern "C" */
#endif
