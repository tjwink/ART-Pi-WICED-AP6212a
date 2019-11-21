/*
 * $ Copyright Broadcom Corporation $
 */

/** @file U8G Library Header
 *
 */

#ifndef _U8G_ARM_H

#include "u8g.h"
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Define   U8G_I2C_USE_REPEAT_START   to enable repeat-start
 *
 * Note: Repeat-start functionality is only available over
 * WICED_I2C_1 (i2c_0) because WICED_I2C_2 (i2c_1) lacks GPIO
 * control.
 */
#define _U8G_ARM_H

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

void    u8g_init_wiced_i2c_device(wiced_i2c_device_t* device);
uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#endif
