/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_driver.h"
#include "ugui_driver_private.h"


static const UG_DRV_FN virtual_fn = {
    /* gfx_pixel_set          */ &UG_driver_virtual_set_pixel,  
    /* raw_fn                 */ NULL, 
    /* raw_set_defaults       */ NULL,  
    /* sys_clear_screen       */ NULL,  
    /* sys_set_backcolor      */ NULL, 
    /* sys_set_forecolor      */ NULL,  
    /* sys_set_draw_mode      */ NULL,  
    /* sys_set_draw_rot       */ NULL, 
    /* sys_set_window         */ NULL,  
    /* sys_clear_window       */ NULL,  
    /* sys_set_fullscreen     */ NULL, 
    /* sys_set_screen_refresh */ NULL,  
    /* sys_set_screen_norminv */ NULL, 
    /* sys_set_screen_update  */ NULL,
    /* sys_screen_scroll_start*/ NULL,
    /* sys_screen_scroll_stop */ NULL, 
    /* txt_set_pos_rowcol     */ NULL,  
    /* txt_set_pos_pixel      */ NULL,  
    /* txt_set_pos_back       */ NULL, 
    /* txt_write              */ NULL, 
    /* txt_write_rowcol       */ NULL, 
    /* txt_write_xy           */ NULL, 
    /* txt_cr                 */ NULL,
    /* txt_show_cursor        */ NULL,
    /* fnt_upload             */ NULL,
    /* fnt_usrlut             */ NULL,  
    /* fnt_search             */ NULL,  
    /* fnt_select             */ NULL,  
    /* fnt_set                */ NULL,  
    /* gfx_set_pos            */ NULL,  
    /* gfx_draw_lineto        */ NULL, 
    /* gfx_set_linepattern    */ NULL, 
    /* gfx_draw_line          */ NULL,
    /* gfx_draw_rect          */ NULL,  
    /* gfx_fill_rect          */ NULL, 
    /* gfx_draw_circle        */ NULL, 
    /* gfx_fill_circle        */ NULL,  
    /* gfx_draw_arc           */ NULL, 
    /* gfx_fill_arc           */ NULL,  
    /* gfx_draw_image         */ NULL,  
    /* gfx_move               */ NULL, 
    /* gfx_draw_poly          */ NULL,  
    /* gfx_fill_poly          */ NULL,  
    /* gfx_draw_polyreg       */ NULL,  
    /* gfx_fill_polyreg       */ NULL, 
    /* com_enable_config      */ NULL, 
    /* pwr_set_brightness     */ NULL,  
    /* pwr_set_onoff          */ NULL,  
    /* pwr_set_sleep          */ NULL, 
    /* pwr_set_deepsleep      */ NULL,  
    /* scr_set_contrast       */ NULL,  
    /* scr_set_splash         */ NULL,  
    /* scr_enable_splash      */ NULL,  
    /* tch_calibrate          */ NULL, 
    /* tch_wait               */ NULL,  
    /* tch_check              */ NULL,  
    /* flash_read             */ NULL,  
    /* flash_write            */ NULL,  
    /* flash_erase            */ NULL,  
    /* eeprom_read            */ NULL,  
    /* eeprom_write           */ NULL
};


void UG_driver_virtual_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;
    
    if(UG_DRV_WMARK != drv_handle->wmark)
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }
    else
    {
        memcpy(&drv_handle->fn, &virtual_fn, sizeof(virtual_fn));
    }
    
    if(NULL!=(err)) *err = err_tmp;

    printf("UG: DRV: virtual->init\n");
}

void UG_driver_virtual_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c)
{
    printf("UG: DRV: virtual->set_pixel\n");
}
