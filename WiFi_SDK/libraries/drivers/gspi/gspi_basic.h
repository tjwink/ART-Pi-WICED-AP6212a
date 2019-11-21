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

#ifndef GSPI_BASIC_H
#define GSPI_BASIC_H

#include <stdint.h>
#include "wiced_constants.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************

*                   Enumerations
 ******************************************************/

typedef enum
{
    GSPI_BUS_READ,
    GSPI_BUS_WRITE
} gspi_transfer_direction_t;

typedef enum
{
    GSPI_BUS_FUNCTION       = 0,
    GSPI_BACKPLANE_FUNCTION = 1
} gspi_function_t;

typedef enum
{
   GSPI_INCREMENT_ADDRESS = 1,
   GSPI_FIXED_ADDRESS     = 0
} gspi_transfer_access_t;

typedef enum
{
   GSPI_CLOCK_POLARITY_LOW      = 0,
   GSPI_CLOCK_POLARITY_HIGH     = 1
} gspi_clock_polarity_t;

typedef enum
{
   GSPI_CLOCK_PHASE_NO_DELAY      = 0,
   GSPI_CLOCK_PHASE_DELAY         = 1
} gspi_clock_phase_t;

/******************************************************
 *                      Macros
 ******************************************************/

#define GSPI_RESPONSE_DELAY             (0x4)
#define GSPI_SLAVE_REG_ADR              ( 0x18009000 )
#define SPI_CLOCK_SPEED_HZ              ( 10000000 )
#define SPI_BIT_WIDTH                   ( 8 )
#define GSPI_TOHSOT_INTMASK             (0xf0)
#define GSPI_F1_INTMASK                 GSPI_TOHSOT_INTMASK

#define VERIFY_RESULT( x )     { wiced_result_t verify_result; verify_result = ( x ); if ( verify_result != WICED_SUCCESS ) return verify_result; }

#define SWAP32_16BIT_PARTS(val) ((uint32_t)(( ((uint32_t)(val)) >> 16) + ((((uint32_t)(val)) & 0xffff)<<16)))

#define GSPI_WAIT_SLAVE_READY_DELAY  (100)

#define F1_READY_TIMEOUT_LOOPS (1000)
#define FEADBEAD_TIMEOUT_MS    (5000)
#define ALP_AVAIL_TIMEOUT_MS   (100)


#define RESPONSE_READY_TIMEOUT_MS  ( WICED_NEVER_TIMEOUT )
#define H32TO16LE(x)           ( ( uint32_t ) ( ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x000000ffU ) << 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x0000ff00U ) >> 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x00ff0000U ) << 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0xff000000U ) >> 8 ) ) )

#define BCMSWAP32(val) \
    ((uint32_t)((((uint32_t)(val) & (uint32_t)0x000000ffU) << 24) | \
          (((uint32_t)(val) & (uint32_t)0x0000ff00U) <<  8) | \
          (((uint32_t)(val) & (uint32_t)0x00ff0000U) >>  8) | \
          (((uint32_t)(val) & (uint32_t)0xff000000U) >> 24)))

#define GSPI_MAX_BACKPLANE_TRANSFER_SIZE         ( 64 ) /* Max packet size on F1 */

#define GSPI_BUS_HEADER_SIZE                     ( sizeof(gspi_header_t) )

#define GSPI_BUS_BACKPLANE_READ_PADD_SIZE        GSPI_RESPONSE_DELAY

#define GSPI_SHARED_MAGIC_NUMBER                 ( 0xbeef )
#define GSPI_SHARED_SIZE                         ( 512 )


/******************************************************
 *             Structures
 ******************************************************/
typedef struct gspi_shared_info
{
        uint32_t magic_number;
        uint32_t slave_to_master_buffer_address;
        uint32_t slave_to_master_buffer_size;
        uint32_t master_to_slave_buffer_address;
        uint32_t master_to_slave_buffer_size;
} gspi_shared_info_t;


/******************************************************
 *               Function Declarations
 ******************************************************/

void gspi_wait_slave_up( void );
void gspi_slave_powerup( wiced_bool_t power_enabled );
void gspi_slave_reset( wiced_bool_t reset_asserted );

wiced_result_t gspi_enable_master_interrupt( void );
wiced_result_t gspi_ack_slave_interrupt( void );
wiced_result_t gspi_notify_slave( void );
wiced_result_t gspi_get_shared_address( uint32_t *shared_address );
wiced_result_t gspi_wait_slave_ready( void );

#ifdef __cplusplus
} /*extern "C" */
#endif


#endif /* ifndef GSPI_BASIC_H */

