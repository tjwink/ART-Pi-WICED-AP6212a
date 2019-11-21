/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_BUS_PRIVATE_H
#define _UGUI_BUS_PRIVATE_H

#include "wiced.h"
#include "ugui_bus.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define UG_BUS_WMARK ( (uint32_t) ( 0xefb0331e ) )


/* I2C */
extern void UG_bus_i2c_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);
extern void UG_bus_i2c_deinit(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);
extern void UG_bus_i2c_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, uint8_t msg_type, 
                              UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag);

/* SPI */
extern void UG_bus_spi_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);
extern void UG_bus_spi_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, uint8_t msg_type, 
                              UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag);

/* SERIAL */
extern void UG_bus_rs232_init(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle);
extern void UG_bus_rs232_msg_fn(UG_BUS_ERR *err, UG_BUS_HANDLE bus_handle, uint8_t msg_type, 
                                UG_BUS_PAYLOAD_T *arg_ptr[], uint32_t arg_num, uint32_t coalesce_flag);


#endif
