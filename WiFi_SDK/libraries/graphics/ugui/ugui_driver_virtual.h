/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_DRIVER_VIRTUAL_PRIVATE_H
#define _UGUI_DRIVER_VIRTUAL_PRIVATE_H


#include "ugui_driver.h"


extern void UG_driver_virtual_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);

extern void UG_driver_virtual_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c);


#endif
