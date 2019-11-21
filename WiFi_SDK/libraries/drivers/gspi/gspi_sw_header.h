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

#ifndef GSPI_SW_HEADER_H
#define GSPI_SW_HEADER_H

#include <stdint.h>
#include "wiced_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
/* To gspi slave SB Mail interrupt */
#define I_GSPI_SMB_SW0       (1 << 0)  /* To gspi slave SB Mail S/W interrupt 0 */
#define I_GSPI_SMB_SW1       (1 << 1)  /* To gspi slave SB Mail S/W interrupt 1 */
#define I_GSPI_SMB_SW2       (1 << 2)  /* To gspi slave SB Mail S/W interrupt 2 */
#define I_GSPI_SMB_SW3       (1 << 3)  /* To gspi slave SB Mail S/W interrupt 3 */
#define I_GSPI_MSB_SW_MASK   0x0000000f /* To gspi slave SB Mail S/W interrupt mask */

#define I_GSPI_SMB_NOTIFY_SLAVE I_GSPI_SMB_SW0 /* Notify slave the data from host is ready */

#define I_GSPI_HMB_SW0       (1 << 0)  /* To gspi Host Mail S/W interrupt 0 */
#define I_GSPI_HMB_SW1       (1 << 1)  /* To gspi Host Mail S/W interrupt 1 */
#define I_GSPI_HMB_SW2       (1 << 2)  /* To gspi Host Mail S/W interrupt 2 */
#define I_GSPI_HMB_SW3       (1 << 3)  /* To gspi Host Mail S/W interrupt 3 */
#define I_GSPI_HMB_SW_SHIFT  (4)
#define I_GSPI_HMB_SW_MASK    (I_GSPI_HMB_SW0 << I_GSPI_HMB_SW_SHIFT)

#define I_GSPI_HMB_SLAVE_READY I_GSPI_HMB_SW1

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum
{
    GSPI_MASTER_TRANSFER_WRITE, /* GSPI master writes data to the GSPI slave device */
    GSPI_MASTER_TRANSFER_READ   /* GSPI master reads data from the GSPI slave device */
} gspi_slave_transfer_direction_t;

typedef enum
{
    GSPI_SLAVE_RESPONSE_GOOD,
    GSPI_SLAVE_RESPONSE_FAILED
} gspi_slave_status_t;

#pragma pack(1)

typedef struct gspi_sw_header
{
        uint16_t         direction;
        uint16_t         sequence_number;
        uint16_t         data_length;
        uint16_t         address;
} gspi_sw_header_t;

typedef struct gspi_sw_tx_header
{
        gspi_sw_header_t gspi_sw_hdr;
        uint8_t          buffer[1];
} gspi_sw_tx_header_t;


typedef struct gspi_slave_response
{
        uint16_t status;
        uint16_t    data_length;
} gspi_slave_response_t;

#pragma pack()


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef GSPI_SW_HEADER_H */
