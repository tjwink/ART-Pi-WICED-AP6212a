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

#ifndef GSPI_SLAVE_H
#define GSPI_SLAVE_H

#include <stdint.h>
#include "wiced_constants.h"
#include "gspi_sw_header.h"
#include "platform_peripheral.h"


#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *             Structures
 ******************************************************/


typedef wiced_result_t (*gspi_slave_register_callback_t)( gspi_sw_tx_header_t* gspi_sw_master_cmd );

typedef struct gspi_slave_application
{
    uint32_t                        send_data_buffer_size;
    uint32_t                        receive_data_buffer_size;
    gspi_slave_register_callback_t  command_callback;
} gspi_slave_application_t;


/**
 * GSPI slave device
 */
typedef struct
{
    wiced_thread_t                        thread;
    uint8_t*                              slave_to_master_buffer;
    uint8_t*                              master_to_slave_buffer;
    uint32_t                              slave_to_master_buffer_size;
    uint32_t                              master_to_slave_buffer_size;
    wiced_bool_t                          quit;
    const gspi_slave_application_t     *gspi_slave_application;

} gspi_slave_device_t;


/******************************************************
 *               Function Declarations
 ******************************************************/

//wiced_result_t gspi_slave_init( gspi_slave_device_t *gspi_slave, gspi_slave_application_t* gspi_slave_app );
wiced_result_t gspi_slave_init( gspi_slave_device_t *gspi_slave, const gspi_slave_application_t* gspi_slave_app );
void gspi_slave_send_response_data( gspi_slave_device_t * gspi_slave, uint8_t* data, uint32_t data_length );


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef GSPI_SLAVE_H */
