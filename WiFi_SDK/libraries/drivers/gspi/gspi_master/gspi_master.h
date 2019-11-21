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

#ifndef GSPI_MASTER_H
#define GSPI_MASTER_H


#include <stdint.h>
#include "wiced_constants.h"
#include "gspi_basic.h"

#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************
 *                    Structures
 ******************************************************/

typedef struct gspi_master
{
        const wiced_spi_device_t*            gspi_device;
        wiced_bool_t                         in_transaction;
        wiced_semaphore_t                    response_ready_semaphore;
        uint32_t gspi_shared_address;
        uint32_t slave_to_master_buffer_address;
        uint32_t slave_to_master_buffer_size;
        uint32_t master_to_slave_buffer_address;
        uint32_t master_to_slave_buffer_size;
} gspi_master_t;

typedef uint32_t gspi_header_t;

#pragma pack(1)

typedef struct
{
    gspi_header_t      header;
    uint8_t            response_delay[GSPI_RESPONSE_DELAY];
} gspi_backplane_f1_read_header_t;

#pragma pack()

typedef struct
{
    gspi_backplane_f1_read_header_t  gspi_header;
    uint32_t                         data[1];
} gspi_backplane_f1_read_packet_t;

typedef struct
{
                 gspi_header_t gspi_header;
                 uint32_t data[1];
} transfer_bytes_packet_t;


/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t gspi_read_command( gspi_master_t* gspi_master, uint16_t address, uint16_t size, uint8_t* data_in );
wiced_result_t gspi_write_command( gspi_master_t *gspi_master,  uint16_t address, uint16_t size, uint8_t* data_out );
wiced_result_t gspi_read_register_value( gspi_function_t function, uint32_t address, uint8_t value_length, /*@out@*/ uint8_t* value );
wiced_result_t gspi_write_register_value( gspi_function_t function, uint32_t address, uint8_t value_length, uint32_t value );
wiced_result_t gspi_transfer_backplane_bytes( gspi_transfer_direction_t direction, uint32_t address, uint32_t size, /*@in@*/ /*@out@*/ uint8_t* data );
wiced_result_t gspi_read_backplane_value( uint32_t address, uint8_t register_length, /*@out@*/ uint8_t* value );
wiced_result_t gspi_write_backplane_value( uint32_t address, uint8_t register_length, uint32_t value );
wiced_result_t gspi_dump_register( void );

wiced_result_t gspi_master_init(gspi_master_t *gspi_master, wiced_spi_device_t *gspi_device );

void gspi_dump( void );


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef GSPI_MASTER_H */
