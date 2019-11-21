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

#ifndef INCLUDED_SPI_WWD_BUS_PROTOCOL_H
#define INCLUDED_SPI_WWD_BUS_PROTOCOL_H

#include <stdint.h>
#include "internal/wwd_thread_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *             Constants
 ******************************************************/

/******************************************************
 *             Structures
 ******************************************************/

typedef uint32_t wwd_bus_gspi_header_t;


#pragma pack(1)

typedef struct
{
    wwd_bus_gspi_header_t gspi_header;
} wwd_bus_header_t;

#pragma pack()

#define WWD_BUS_HAS_HEADER

#define WWD_BUS_HEADER_SIZE                     ( sizeof(wwd_bus_header_t) )

#define WWD_BUS_USE_STATUS_REPORT_SCHEME        ( 1 == 1 )

#define WWD_BUS_MAX_BACKPLANE_TRANSFER_SIZE     ( 64 ) /* Max packet size on F1 */
#define WWD_BUS_BACKPLANE_READ_PADD_SIZE        ( 4 )

/******************************************************
 *             Function declarations
 ******************************************************/

/******************************************************
 *             Global variables
 ******************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef INCLUDED_SPI_WWD_BUS_PROTOCOL_H */
