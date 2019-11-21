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
 *
 * AK4961 I2C bus implementation.
 */

/** @file
 *
 */

#include "wiced_platform.h"
#include "wwd_assert.h"
#include "ak4961.h"
#include <string.h>
#include <stdio.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define AK4961_VERIFY(x)                               {wiced_result_t res = (x); if (res != WICED_SUCCESS){wiced_assert(#x, 0==1); return res;}}

/******************************************************
 *                    Constants
 ******************************************************/

#define AK4961_COMMAND_CODE_CTREG_READ      (0x01)
#define AK4961_COMMAND_CODE_CTREG_WRITE     (0x81)

#define I2C_XFER_RETRY_COUNT                (3)
#define I2C_DISABLE_DMA                     WICED_TRUE


/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct i2c_ak4961_payload           i2c_ak4961_payload_t;


/******************************************************
 *                    Structures
 ******************************************************/

struct i2c_ak4961_payload
{
    uint8_t cmd;
    uint8_t addr_hi;
    uint8_t addr_lo;
    uint8_t data;
};

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint8_t ak4961_reg_state[0x0109 + 1];

/******************************************************
 *              Function Declarations
 ******************************************************/

extern wiced_result_t ak4961_chip_reset(ak4961_device_cmn_data_t *ak4961);


/******************************************************
 *               Function Definitions
 ******************************************************/

/* Defaults from AK4961 datasheet. */
static void ak4961_reset_reg_state( void )
{
    memset( ak4961_reg_state, 0x00, sizeof ak4961_reg_state );

    /* Non-zero defaults. */
    ak4961_reg_state[0x0034] = 0x10;
    ak4961_reg_state[0x0035] = 0x02;
    ak4961_reg_state[0x0036] = 0x19;
    ak4961_reg_state[0x0037] = 0x19;
    ak4961_reg_state[0x0038] = 0x19;
    ak4961_reg_state[0x0039] = 0x19;
    ak4961_reg_state[0x003A] = 0x75;
    ak4961_reg_state[0x003B] = 0x05;
    ak4961_reg_state[0x003C] = 0x55;
    ak4961_reg_state[0x0054] = 0x20;
    ak4961_reg_state[0x005A] = 0x20;
    ak4961_reg_state[0x0067] = 0x03;
    ak4961_reg_state[0x006E] = 0x10;
    ak4961_reg_state[0x0094] = 0x29;
    ak4961_reg_state[0x0096] = 0x0A;
    ak4961_reg_state[0x00D5] = 0x61;
    ak4961_reg_state[0x00DA] = 0xFC;
    ak4961_reg_state[0x00DF] = 0x08;
    ak4961_reg_state[0x00E3] = 0x08;
    ak4961_reg_state[0x00E7] = 0x08;
    ak4961_reg_state[0x00EB] = 0x08;
}

static wiced_result_t ak4961_i2c_reg_write(wiced_i2c_device_t *device, uint16_t reg, uint8_t value)
{
    wiced_i2c_message_t     msg[1];
    i2c_ak4961_payload_t    payload;

    payload.cmd     = AK4961_COMMAND_CODE_CTREG_WRITE;
    payload.addr_hi = (uint8_t)(reg >> 8);
    payload.addr_lo = (uint8_t)(reg);
    payload.data    = value;

    //WPRINT_LIB_INFO(("i2c write: cmd:0x%02x addr_hi:0x%02x addr_lo:0x%02x data:0x%02x (bytes %d)\n", payload.cmd, payload.addr_hi, payload.addr_lo, payload.data, sizeof payload));
    AK4961_VERIFY( wiced_i2c_init_tx_message( msg, &payload, 4, I2C_XFER_RETRY_COUNT, I2C_DISABLE_DMA ) );

    return wiced_i2c_transfer( device, msg, 1 );
}

static wiced_result_t ak4961_i2c_ram_write( wiced_i2c_device_t *device, const uint8_t *data, uint32_t data_length)
{
    wiced_i2c_message_t     msg[1];
    AK4961_VERIFY( wiced_i2c_init_tx_message( msg, data, data_length, I2C_XFER_RETRY_COUNT, I2C_DISABLE_DMA ) );

    return wiced_i2c_transfer( device, msg, 1 );
}

wiced_result_t ak4961_reg_init( ak4961_device_cmn_data_t *ak4961 )
{
    AK4961_VERIFY( wiced_i2c_init( ak4961->i2c_data ) );

    return WICED_SUCCESS;
}

wiced_result_t ak4961_reg_reset( ak4961_device_cmn_data_t *ak4961 )
{
    /* Chip reset. */
    AK4961_VERIFY( ak4961_chip_reset( ak4961 ) );

    /* Sync register cache. */
    ak4961_reset_reg_state();

    return WICED_SUCCESS;
}

/* I2C combined messages is unsupported in BCM4390x.GSIO! */
wiced_result_t ak4961_reg_read( ak4961_device_cmn_data_t *ak4961, uint16_t register_address, uint8_t *reg_data )
{
    UNUSED_PARAMETER( ak4961 );

    if ( register_address >= sizeof( ak4961_reg_state ) )
    {
        return WICED_BADVALUE;
    }

    *reg_data = ak4961_reg_state[register_address];

    return WICED_SUCCESS;
}

wiced_result_t ak4961_reg_write( ak4961_device_cmn_data_t *ak4961, uint16_t register_address, uint8_t reg_data )
{
    if ( register_address >= sizeof( ak4961_reg_state ) )
    {
        return WICED_BADVALUE;
    }

    AK4961_VERIFY( ak4961_i2c_reg_write( ak4961->i2c_data, register_address, reg_data ) );
    ak4961_reg_state[register_address] = reg_data;
    return WICED_SUCCESS;
}

wiced_result_t ak4961_ram_write( ak4961_device_cmn_data_t *ak4961, const uint8_t *data, uint32_t data_length )
{
    AK4961_VERIFY( ak4961_i2c_ram_write( ak4961->i2c_data, data, data_length ) );
    return WICED_SUCCESS;
}

wiced_result_t ak4961_print_cached_register_values( void )
{
    uint16_t i;

    for ( i = 0; i < sizeof(ak4961_reg_state); i++ )
    {
        printf("{ 0x%hX ,0x%02X },\n", i, ak4961_reg_state[i]);
    }

    return WICED_SUCCESS;
}
