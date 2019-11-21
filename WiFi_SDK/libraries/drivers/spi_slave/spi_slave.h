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

#include <stdint.h>
#include "wiced_constants.h"

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
    SPI_SLAVE_REGISTER_DATA_STATIC,
    SPI_SLAVE_REGISTER_DATA_DYNAMIC
} spi_slave_register_data_type_t;

typedef enum
{
    SPI_SLAVE_ACCESS_READ_ONLY,
    SPI_SLAVE_ACCESS_WRITE_ONLY,
    SPI_SLAVE_ACCESS_READ_WRITE
} spi_slave_access_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * SPI slave device
 */
typedef struct
{
    const struct spi_slave_device_config* config;
    wiced_thread_t                        thread;
    wiced_spi_slave_data_buffer_t*        buffer;
    uint32_t                              buffer_size;
    wiced_bool_t                          quit;
} spi_slave_t;

/**
 * SPI slave device configuration
 */
typedef struct spi_slave_device_config
{
    wiced_spi_t                      spi;
    wiced_spi_slave_config_t         config;
    const struct spi_slave_register* register_list;
    uint32_t                         register_count;
} spi_slave_device_config_t;

typedef wiced_result_t (*spi_slave_register_callback_t)( spi_slave_t* device, uint8_t* buffer );

/**
 * SPI slave register structure
 */
typedef struct spi_slave_register
{
    uint16_t                       address;
    spi_slave_access_t             access;
    spi_slave_register_data_type_t data_type;
    uint16_t                       data_length;
    uint8_t*                       static_data;
    spi_slave_register_callback_t  read_callback;
    spi_slave_register_callback_t  write_callback;
} spi_slave_register_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t spi_slave_init  ( spi_slave_t* device, const spi_slave_device_config_t* config );
wiced_result_t spi_slave_deinit( spi_slave_t* device );

#ifdef __cplusplus
} /* extern "C" */
#endif
