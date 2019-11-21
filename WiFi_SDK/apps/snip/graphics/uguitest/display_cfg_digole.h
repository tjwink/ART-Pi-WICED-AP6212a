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

/*
 * DIGOLE display configuration file
 */

#ifndef _DISPLAY_CFG_DIGOLE_H
#define _DISPLAY_CFG_DIGOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"
#include "ugui.h"

/* ****************************************************************** */
/*                                                                    */
/* UG BUS TYPE                                                        */
/*                                                                    */
/* Select only one bus to be used by this DIGOLE display              */
/* ****************************************************************** */

//#define USE_UGUI_BUS_I2C
#define USE_UGUI_BUS_SPI
//#define USE_UGUI_BUS_UART


/* ****************************************************************** */
/*                                                                    */
/* UG BUS CONFIG                                                      */
/*                                                                    */
/* these are the default bus config values for a DIGOLE display       */
/* change them if you need a custom speed or a custom adddress        */
/* different from the default ones.                                   */
/* ****************************************************************** */

#if defined USE_UGUI_BUS_SPI
#define SPI_CLOCK_SPEED_HZ             ( 20000000 )
#define SPI_BIT_WIDTH                  ( 8 )
#define SPI_MODE                       ( SPI_CLOCK_FALLING_EDGE | SPI_CLOCK_IDLE_LOW | SPI_MSB_FIRST | SPI_CS_ACTIVE_LOW )
#define HW_BUS                         (UG_DRV_CFG_DEV_BUS_SPI) /* needed for driver cfg */
static const wiced_spi_device_t display_spi =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_GPIO_22,
    .speed       = SPI_CLOCK_SPEED_HZ,
    .mode        = SPI_MODE,
    .bits        = SPI_BIT_WIDTH
};
#endif

#if defined USE_UGUI_BUS_I2C
#define HW_BUS                         (UG_DRV_CFG_DEV_BUS_I2C) /* needed for driver cfg */

/* NOTE: Digole does NOT work above STANDARD_SPEED_MODE */
static const wiced_i2c_device_t display_i2c =
{
    .port          = WICED_I2C_2,
    .address       = 0x27,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .flags         = I2C_DEVICE_NO_DMA,
    .speed_mode    = I2C_STANDARD_SPEED_MODE,
};
#endif


/* ****************************************************************** */
/*                                                                    */
/* UG DRIVER CONFIG                                                   */
/*                                                                    */
/* these are the default driver config values for a DIGOLE display    */
/* change these values according to your specific model               */
/* ****************************************************************** */

#define DISPLAY_W_PX (160)
#define DISPLAY_H_PX (128)

static const UG_DRV_CONFIG drv_cfg =
{
    /* x_max          */ DISPLAY_W_PX,
    /* y_max          */ DISPLAY_H_PX,
    /* fmt_color      */ UG_DRV_FMT_RGB888,
    /* hw_proto       */ UG_DRV_CFG_DEV_DIGOLE,
    /* hw_bus         */ HW_BUS, /* don't change this! */
    /* hw_addr        */ 0x00,
    /* hw_mjr         */ 2,
    /* hw_min         */ 8,
    /* hw_model_pn    */ "DS160128COLED-46",
    /* flash_enabled  */ 0,
    /* eeprom_enabled */ 0,
    /* touch_enabled  */ 0,
    /* opt:hw_usr1    */ 0,
    /* opt:hw_usr2    */ NULL,
    /* opt:reset_gpio */ 0,
    /* opt:touch_gpio */ 0
};


/* ****************************************************************** */
/*                                                                    */
/* UG DRIVER HW ACCELERATION                                          */
/*                                                                    */
/* these are the default driver config values for a DIGOLE display    */
/* change these values according to your specific model               */
/* ****************************************************************** */

#define ENABLE_UG_HW_DRIVER()                                           \
    do                                                                  \
    {                                                                   \
        UG_DriverRegister( DRIVER_DRAW_LINE,     &UG_driver_gfx_draw_line ); \
        UG_DriverRegister( DRIVER_FILL_FRAME,    &UG_driver_gfx_fill_rect ); \
        UG_DriverRegister( DRIVER_DRAW_CIRCLE,   &UG_driver_gfx_draw_circle ); \
        UG_DriverRegister( DRIVER_FILL_CIRCLE,   &UG_driver_gfx_fill_circle ); \
        UG_DriverRegister( DRIVER_SET_FORECOLOR, &UG_driver_sys_set_forecolor ); \
        UG_DriverRegister( DRIVER_SET_BACKCOLOR, &UG_driver_sys_set_backcolor ); \
        UG_DriverRegister( DRIVER_MOVE_AREA,     &UG_driver_gfx_move ); \
    }while(0);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
