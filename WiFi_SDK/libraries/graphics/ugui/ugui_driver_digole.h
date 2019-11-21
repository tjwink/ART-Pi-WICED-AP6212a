/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_DRIVER_DIGOLE_PRIVATE_H
#define _UGUI_DRIVER_DIGOLE_PRIVATE_H


#include "ugui_driver.h"

/* internal table to map USR font slots */
#define UG_DRIVER_DIGOLE_USR_FNT_MAX (4)
static const uint8_t ug_driver_digole_usr_fnt_code[UG_DRIVER_DIGOLE_USR_FNT_MAX] = {200,201,202,203};

#define UG_DRIVER_DIGOLE_STD_FNT_MAX (7)
static const uint8_t ug_driver_digole_std_fnt_code[UG_DRIVER_DIGOLE_STD_FNT_MAX] = {0,6,10,18,51,120,123};

#define UG_DRIVER_DIGOLE_USR_FNT_BYTES (3584)

/* used for internal conversion from uGUI to DIGOLE colors on RGB332 map */
typedef enum digole_c_rgb332_{
    DG_RGB_332_CYAN     = (uint32_t)0x5B,
    DG_RGB_332_RED      = (uint32_t)0xE4,
    DG_RGB_332_ORANGE   = (uint32_t)0xEC,
    DG_RGB_332_PURPLE   = (uint32_t)0x66,
    DG_RGB_332_MAGENTA  = (uint32_t)0xA1,
    DG_RGB_332_YELLOW   = (uint32_t)0xFC,
    DG_RGB_332_WHITE    = (uint32_t)0xFF,
    DG_RGB_332_BLACK    = (uint32_t)0x00,
    DG_RGB_332_GREEN    = (uint32_t)0x3C,
    DG_RGB_332_BLUE     = (uint32_t)0x03,
    DG_RGB_332_PEACH    = (uint32_t)0xF1,
    DG_RGB_332_GREY     = (uint32_t)0x52
}DIGOLE_C_RGB332T;


/* API */
extern void UG_driver_digole_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_gfx_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c);
extern void UG_driver_digole_raw_fn( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t msg, void *data, uint32_t len);
extern void UG_driver_digole_raw_set_defaults(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);

/* SYSTEM */
extern void UG_driver_digole_sys_clear_screen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_sys_set_backcolor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint32_t c);
extern void UG_driver_digole_sys_set_forecolor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint32_t c);
extern void UG_driver_digole_sys_set_draw_mode(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode);
extern void UG_driver_digole_sys_set_draw_rot(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t deg);
extern void UG_driver_digole_sys_set_window(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y,  uint16_t w, uint16_t h);
extern void UG_driver_digole_sys_clear_window(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_sys_set_fullscreen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_sys_set_screen_refresh(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode);
extern void UG_driver_digole_sys_set_screen_norminv(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);

/* FONT */
extern void UG_driver_digole_fnt_upload(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t hw_id, void *ug_font, uint8_t *data, uint32_t data_len);
extern void UG_driver_digole_fnt_usrlut(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, UG_DRV_FONT_LUT lut[], uint8_t lut_len);
extern void UG_driver_digole_fnt_search(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *hw_id, void *ug_font);
extern void UG_driver_digole_fnt_select(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, void *ug_font);
extern void UG_driver_digole_fnt_set(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t hw_id);

/* TEXT */
extern void UG_driver_digole_txt_set_pos_rowcol(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t cx1, int16_t cy1);
extern void UG_driver_digole_txt_set_pos_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x1, int16_t y1);
extern void UG_driver_digole_txt_set_pos_back(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_txt_write(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len);
extern void UG_driver_digole_txt_write_rowcol(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len, int16_t cx, int16_t ry);
extern void UG_driver_digole_txt_write_xy(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len, int16_t cx, int16_t ry);
extern void UG_driver_digole_txt_cr(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);
extern void UG_driver_digole_txt_show_cursor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode, uint8_t onoff_f);

/* GRAPHICS */
extern void UG_driver_digole_gfx_set_pos(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y);
extern void UG_driver_digole_gfx_draw_lineto(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x2, int16_t y2, uint32_t argb);
extern void UG_driver_digole_gfx_set_linepattern(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t pattern);
extern void UG_driver_digole_gfx_draw_line(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                           int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
extern void UG_driver_digole_gfx_draw_rect(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                           int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
extern void UG_driver_digole_gfx_fill_rect(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                           int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
extern void UG_driver_digole_gfx_draw_circle(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                             int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);
extern void UG_driver_digole_gfx_fill_circle(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                             int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);
extern void UG_driver_digole_gfx_draw_image(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                            int16_t x, int16_t y, UG_DRV_CFG_FMT_COLOR type, uint8_t *pixels, uint16_t w, uint16_t h, void* usr_fmt);
extern void UG_driver_digole_gfx_move(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                      int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t off_x, int16_t off_y);

/* COMMUNICATION */
extern void UG_driver_digole_com_enable_config(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);

/* POWER */
extern void UG_driver_digole_pwr_set_brightness(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl);
extern void UG_driver_digole_pwr_set_onoff(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);
extern void UG_driver_digole_pwr_set_sleep(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);
extern void UG_driver_digole_pwr_set_deepsleep(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);

/* SCREEN */
extern void UG_driver_digole_scr_set_contrast(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl);
extern void UG_driver_digole_scr_set_splash(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *data, uint32_t data_len);
extern void UG_driver_digole_scr_enable_splash(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag);

/* TOUCH PANEL */
/* tbd */

/* FLASH */
/* tbd */

/* EEPROM */
/* tbd */

#endif
