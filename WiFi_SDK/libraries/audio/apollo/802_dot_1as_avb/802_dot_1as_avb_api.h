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

/**
 * @file 802.1as AVB API
 */

#pragma once

#include "wiced_result.h"

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
    AVB_FRAME_TYPE_NONE,
    AVB_FRAME_TYPE_ETHERNET_MULTICAST,
    AVB_FRAME_TYPE_ETHERNET_UNICAST,
    AVB_FRAME_TYPE_IP_UDP_MULTICAST
} avb_frame_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct avb_timestamp_s
{
        uint32_t    lock;           /* Lock flag, writing when locked */
        uint32_t    avb_ts_ref1;    /* Reference 1 write-back */
        uint16_t    pad;            /* alignment padding */
        uint16_t    ts_sec_h;       /* high 16-bits of timestamp seconds */
        uint32_t    ts_sec_l;       /* low 32-bits of timestamp seconds */
        uint32_t    ts_nsec;        /* timestamp nanoseconds */
        uint32_t    avb_ts;         /* AVB timer reading corresponding to timestamp*/
        uint32_t    avb_ts_ref2;    /* Reference2 write-back */
} avb_timestamp_t;

/******************************************************************************
* @purpose Initialize the AVB library and network.
*
* @param    frame_type   Level 2 Ethernet multicast/unicast PTP or UDP/IP multicast frames
* @param    iface        WICED network interface @ref wiced_interface_t
* @param    client_addr  WICED IPv4 address
* @param    udp_port     UDP port
* @returns  @ref wiced_result_t
*
* @comments
*
*
* @end
*******************************************************************************/
wiced_result_t avblib_init(avb_frame_type_t frame_type, wiced_interface_t iface, wiced_ip_address_t *client_addr, uint16_t udp_port);


/******************************************************************************
* @purpose  Shut down the AVB library
*
* @param    none
*
* @returns  @ref wiced_result_t
*
* @comments
*
*
* @end
*******************************************************************************/
wiced_result_t avblib_deinit(void);


/******************************************************************************
* @purpose  Send a multicast 802.1as Sync Message
*
* @param    none
*
* @returns  @ref wiced_result_t
*
* @comments
*
*
* @end
*******************************************************************************/
wiced_result_t avblib_sync_send(void);


/******************************************************************************
* @purpose  Get the AVB timestamp
*
* @param    unsigned int *AVB_timestamp
*
* @returns  @ref wiced_result_t
*
* @comments
*
*
* @end
*******************************************************************************/
wiced_result_t avblib_ts_get(avb_timestamp_t *avb_timestamp);


#ifdef __cplusplus
} /* extern "C" */
#endif
