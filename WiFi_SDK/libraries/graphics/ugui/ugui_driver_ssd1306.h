/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_DRIVER_SSD1306_PRIVATE_H
#define _UGUI_DRIVER_SSD1306_PRIVATE_H


#include "ugui_driver.h"

/* API */
extern void UG_driver_ssd1306_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_ssd1306_deinit(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_ssd1306_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c);

/* SYSTEM */
extern void UG_driver_ssd1306_sys_clear_screen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_ssd1306_sys_screen_update(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);

extern void UG_driver_ssd1306_sys_screen_scroll_start(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                                      uint8_t direction, 
                                                      uint8_t speed_x, uint8_t speed_y, 
                                                      uint16_t scroll_x, uint16_t scroll_y,
                                                      uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                                      uint8_t circular_f,
                                                      void* usr_data);

extern void UG_driver_ssd1306_sys_screen_scroll_stop(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);

/* FONT */
/* none supported */

/* TEXT */
/* none supported */

/* GRAPHICS */
/* none supported */

/* COMMUNICATION */
/* none supported */

/* POWER */
/* none supported */

/* SCREEN */
/* ******************* */
extern void UG_driver_ssd1306_scr_set_contrast(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl);

/* TOUCH PANEL */
/* none supported */

/* FLASH */
/* none supported */

/* EEPROM */
/* none supported */

#endif
