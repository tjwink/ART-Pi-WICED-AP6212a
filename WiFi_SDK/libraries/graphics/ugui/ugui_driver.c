/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_driver.h"
#include "ugui_driver_private.h"

/* for direct driver fn calls we do
 * sanity checks only in DEBUG mode 
 */
#if defined ( DEBUG )
#define DRV_SANITY_CHECK(x)                             \
    do                                                  \
    {                                                   \
        if ((x) == NULL)                                \
            return -1;                                  \
                                                        \
        if(UG_DRV_WMARK!=(x)->wmark)                    \
            return -1;                                  \
                                                        \
        if(UG_DRV_STATUS_READY!=(x)->status)            \
            return -1;                                  \
    }while(0);
#else
#define DRV_SANITY_CHECK(x)  { /* empty */ }
#endif


/* Pointer to the current selected driver, must be STATIC */
static UG_DRV_HANDLE _ug_drv_hdl = NULL;



/******************************************************
 *               Function Definitions
 ******************************************************/


void UG_driver_init(  UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, UG_DRV_CONFIG *config)
{
    UG_DRV_T      drv_tmp ;
    UG_DRV_HANDLE drv_ref = NULL;
    UG_DRV_ERR    err_tmp = UG_DRV_NO_ERR;
    uint8_t             i = 0;

    if(NULL==drv_handle)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(NULL==config)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* create, init and return an handle */
    drv_ref = &drv_tmp;

    /* init driver id params */
    drv_ref->id       = 0;
    drv_ref->wmark    = UG_DRV_WMARK;
    drv_ref->type     = 0;

    /* COMPONENT_SWVER() */
    drv_ref->major    = 0;
    drv_ref->minor    = 0;

    /* clear font LUT */
    for (i=0; i<UG_DRV_MAX_HW_FONT_NUM; i++)
    {
        drv_ref->font_lut[i].ug_font_ptr = NULL;
        drv_ref->font_lut[i].hw_id = 0;
    }

    /* zombie status */
    drv_ref->status    = UG_DRV_STATUS_UNKNOWN;

    memcpy( &drv_ref->cfg, config, sizeof(UG_DRV_CONFIG) );

    /* open the device and return error if needed */
    switch (config->hw_proto)
    {
        case UG_DRV_CFG_DEV_VIRTUAL :
            /* TBD */ 
            break;
        case UG_DRV_CFG_DEV_DIGOLE :
            UG_driver_digole_init(&err_tmp, drv_ref);
            break;
        case UG_DRV_CFG_DEV_SSD1306 :
            UG_driver_ssd1306_init(&err_tmp, drv_ref);
        default :
            break;
    }

    /* copy on success */
    if(err_tmp==UG_DRV_NO_ERR)
    {
        drv_ref->status = UG_DRV_STATUS_READY;

        memcpy( drv_handle, drv_ref, sizeof(UG_DRV_T) );

        _ug_drv_hdl = drv_handle;
    }

    if(NULL!=(err)) *err = err_tmp;
}


void UG_driver_deinit(  UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR    err_tmp = UG_DRV_NO_ERR;

    if(NULL==drv_handle)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* open the device and return error if needed */
    switch (drv_handle->cfg.hw_proto)
    {
        case UG_DRV_CFG_DEV_SSD1306 :
            UG_driver_ssd1306_deinit(&err_tmp, drv_handle);
            break;
        case UG_DRV_CFG_DEV_VIRTUAL :
        case UG_DRV_CFG_DEV_DIGOLE  :
        default :
            /* not needed */
            break;
    }

    /* mark the driver as unavailable */
    drv_handle->status = UG_DRV_STATUS_UNKNOWN;

    if(NULL!=(err)) *err = err_tmp;
}

void UG_driver_select( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;
    
    if(NULL!=drv_handle)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(UG_DRV_WMARK!=drv_handle->wmark)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(UG_DRV_STATUS_READY!=drv_handle->status)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }
    _ug_drv_hdl = drv_handle;

    if(NULL!=(err)) *err = err_tmp;
}


/* ********************************** */
/*      DISPLAY PRIMITIVES API        */
/* ********************************** */

void UG_driver_gfx_pixel_set(int16_t x, int16_t y, uint32_t argb)
{
#if defined ( DEBUG )
    /* sanity checks only in DEBUG mode */
    if (_ug_drv_hdl == NULL)
        return;

    if(UG_DRV_WMARK!=_ug_drv_hdl->wmark)
        return;
 
    if(UG_DRV_STATUS_READY!=_ug_drv_hdl->status)
        return;
#endif
    _ug_drv_hdl->fn.gfx_pixel_set(NULL, _ug_drv_hdl, x, y, argb);

    if(_ug_drv_hdl->foreground_color != argb) _ug_drv_hdl->foreground_color = argb;
}


/* ******************* */
/*       RAW           */
/* ******************* */
void UG_driver_raw_fn( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t msg, void *data, uint32_t len)
{
#if defined ( DEBUG )
    /* sanity checks only in DEBUG mode */
    if (drv_handle == NULL)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }
    
    if(UG_DRV_WMARK!=drv_handle->wmark)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(UG_DRV_STATUS_READY!=drv_handle->status)
    {
        if(NULL!=err) *err = UG_DRV_ERR_UNKNOWN;
        return;
    }
#endif

    if((NULL==data) || (len==0))
        return;

    if(drv_handle->fn.raw_fn)
        drv_handle->fn.raw_fn(err, drv_handle, msg, data, len);
}

void UG_driver_raw_set_defaults(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
#if defined ( DEBUG )
    /* sanity checks only in DEBUG mode */
    if (drv_handle == NULL)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }
    
    if(UG_DRV_WMARK!=drv_handle->wmark)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(UG_DRV_STATUS_READY!=drv_handle->status)
    {
        if(NULL!=err) *err = UG_DRV_ERR_UNKNOWN;
        return;
    }
#endif

    if(drv_handle->fn.raw_set_defaults)
        drv_handle->fn.raw_set_defaults(err, drv_handle);
}

/* ******************* */
/*       SYSTEM        */
/* ******************* */

int8_t UG_driver_sys_clear_screen(void)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_clear_screen)
        _ug_drv_hdl->fn.sys_clear_screen(&err, _ug_drv_hdl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_backcolor(uint32_t c)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_backcolor)
        _ug_drv_hdl->fn.sys_set_backcolor(&err, _ug_drv_hdl, c);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_forecolor(uint32_t c)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_forecolor)
        _ug_drv_hdl->fn.sys_set_forecolor(&err, _ug_drv_hdl, c);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_draw_mode(uint8_t mode)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_draw_mode)
        _ug_drv_hdl->fn.sys_set_draw_mode(&err, _ug_drv_hdl, mode);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_draw_rot(int16_t deg)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_draw_rot)
        _ug_drv_hdl->fn.sys_set_draw_rot(&err, _ug_drv_hdl,deg);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_window(int16_t x, int16_t y, uint16_t w, uint16_t h)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_window)
        _ug_drv_hdl->fn.sys_set_window(&err, _ug_drv_hdl, x, y, w, h);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_clear_window(void)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_clear_window)
        _ug_drv_hdl->fn.sys_clear_window(&err, _ug_drv_hdl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_fullscreen(void)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_fullscreen)
        _ug_drv_hdl->fn.sys_set_fullscreen(&err, _ug_drv_hdl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_screen_refresh(uint8_t mode)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_screen_refresh)
        _ug_drv_hdl->fn.sys_set_screen_refresh(&err, _ug_drv_hdl, mode);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_set_screen_norminv(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_set_screen_norminv)
        _ug_drv_hdl->fn.sys_set_screen_norminv(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_screen_update(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_screen_update)
        _ug_drv_hdl->fn.sys_screen_update(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_screen_scroll_start(uint8_t direction, 
                                         uint8_t speed_x, uint8_t speed_y, 
                                         uint16_t scroll_x, uint16_t scroll_y, 
                                         uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                         uint8_t circular_f,
                                         void* usr_data)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_screen_scroll_start)
    {
        _ug_drv_hdl->fn.sys_screen_scroll_start( &err, _ug_drv_hdl, 
                                                 direction, 
                                                 speed_x, speed_y,
                                                 scroll_x, scroll_y,
                                                 x1, y1, x2, y2,
                                                 circular_f,
                                                 usr_data);
    }
    else
    {
        return -1;
    }

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_sys_screen_scroll_stop()
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.sys_screen_scroll_stop)
        _ug_drv_hdl->fn.sys_screen_scroll_stop( &err, _ug_drv_hdl );
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*       TEXT          */
/* ******************* */

int8_t UG_driver_txt_set_pos_rowcol(int16_t cx, int16_t ry)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_set_pos_rowcol)
        _ug_drv_hdl->fn.txt_set_pos_rowcol(&err, _ug_drv_hdl, cx, ry);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_set_pos_pixel(int16_t x1, int16_t y1)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_set_pos_pixel)
        _ug_drv_hdl->fn.txt_set_pos_pixel(&err, _ug_drv_hdl, x1, y1);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_set_pos_back(void)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_set_pos_back)
        _ug_drv_hdl->fn.txt_set_pos_back(&err, _ug_drv_hdl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_write(char* text, uint32_t text_len)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_write)
        _ug_drv_hdl->fn.txt_write(&err, _ug_drv_hdl, text, text_len);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_write_rowcol(char* text, uint32_t text_len, int16_t cx, int16_t ry)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_write_rowcol)
        _ug_drv_hdl->fn.txt_write_rowcol(&err, _ug_drv_hdl, text, text_len, cx, ry);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_write_xy(char* text, uint32_t text_len, int16_t x, int16_t y)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_write_xy)
        _ug_drv_hdl->fn.txt_write_xy(&err, _ug_drv_hdl, text, text_len, x, y);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_cr(void)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_cr)
        _ug_drv_hdl->fn.txt_cr(&err, _ug_drv_hdl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_txt_show_cursor(uint8_t mode, uint8_t onoff_f)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.txt_show_cursor)
        _ug_drv_hdl->fn.txt_show_cursor(&err, _ug_drv_hdl, mode, onoff_f);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}


/* ******************* */
/*        FONT         */
/* ******************* */

int8_t UG_driver_fnt_upload(uint8_t hw_id, void *ug_font, uint8_t *data, uint32_t data_len)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data) || (data_len==0))
        return -1;

    if(_ug_drv_hdl->fn.fnt_upload)
        _ug_drv_hdl->fn.fnt_upload(&err, _ug_drv_hdl, hw_id, ug_font, data, data_len);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_fnt_usrlut(UG_DRV_FONT_LUT lut[], uint8_t lut_len)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;
    
    DRV_SANITY_CHECK(_ug_drv_hdl);
    
    if((NULL==lut) || (lut_len==0) || (lut_len>=UG_DRV_MAX_HW_FONT_NUM))
        return -1;
    
    if(_ug_drv_hdl->fn.fnt_usrlut)
        _ug_drv_hdl->fn.fnt_usrlut(&err, _ug_drv_hdl, lut, lut_len);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;
    
    return 0;
}

int8_t UG_driver_fnt_search(uint8_t *hw_id, void *ug_font)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.fnt_search)
        _ug_drv_hdl->fn.fnt_search(&err, _ug_drv_hdl, hw_id, ug_font);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_fnt_select(void *ug_font)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.fnt_select)
        _ug_drv_hdl->fn.fnt_select(&err, _ug_drv_hdl, ug_font);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_fnt_set(uint8_t hw_id)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.fnt_set)
        _ug_drv_hdl->fn.fnt_set(&err, _ug_drv_hdl, hw_id);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}


/* ******************* */
/*       GRAPHICS      */
/* ******************* */

int8_t UG_driver_gfx_set_pos(int16_t x, int16_t y)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_set_pos)
        _ug_drv_hdl->fn.gfx_set_pos(&err, _ug_drv_hdl, x, y);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_lineto(int16_t x2, int16_t y2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_lineto)
        _ug_drv_hdl->fn.gfx_draw_lineto(&err, _ug_drv_hdl, x2, y2,argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_set_linepattern(uint8_t pattern)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_set_linepattern)
        _ug_drv_hdl->fn.gfx_set_linepattern(&err, _ug_drv_hdl, pattern);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_line(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_line)
        _ug_drv_hdl->fn.gfx_draw_line(&err, _ug_drv_hdl, x1, y1, x2, y2, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_rect)
       _ug_drv_hdl->fn.gfx_draw_rect(&err, _ug_drv_hdl, x1, y1, x2, y2, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_fill_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_fill_rect)
        _ug_drv_hdl->fn.gfx_fill_rect(&err, _ug_drv_hdl, x1, y1, x2, y2, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_circle(int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_circle)
        _ug_drv_hdl->fn.gfx_draw_circle(&err, _ug_drv_hdl, cx1, cy1, r, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_fill_circle(int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_fill_circle)
        _ug_drv_hdl->fn.gfx_fill_circle(&err, _ug_drv_hdl, cx1, cy1, r, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_arc(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_arc)
        _ug_drv_hdl->fn.gfx_draw_arc(&err, _ug_drv_hdl, cx1, cy1, r, deg1, deg2, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_fill_arc(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_fill_arc)
        _ug_drv_hdl->fn.gfx_fill_arc(&err, _ug_drv_hdl, cx1, cy1, r, deg1, deg2, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_image(int16_t x, int16_t y, UG_DRV_CFG_FMT_COLOR type, uint8_t *pixels, uint16_t w, uint16_t h, void* usr_fmt)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_image)
        _ug_drv_hdl->fn.gfx_draw_image(&err, _ug_drv_hdl, x, y, type, pixels, w, h, usr_fmt);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_move(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t off_x, int16_t off_y)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_move)
        _ug_drv_hdl->fn.gfx_move(&err, _ug_drv_hdl, x, y, width, height, off_x, off_y);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_poly(int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_poly)
        _ug_drv_hdl->fn.gfx_draw_poly(&err, _ug_drv_hdl, x, y, vertex_num, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_fill_poly(int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_fill_poly)
        _ug_drv_hdl->fn.gfx_fill_poly(&err, _ug_drv_hdl, x, y, vertex_num, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_draw_polyreg(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_poly)
        _ug_drv_hdl->fn.gfx_draw_polyreg(&err, _ug_drv_hdl, cx1, cy1, r, deg, vertex_num, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_gfx_fill_polyreg(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.gfx_draw_poly)
        _ug_drv_hdl->fn.gfx_fill_polyreg(&err, _ug_drv_hdl, cx1, cy1, r, deg, vertex_num, argb);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}


/* ******************* */
/*    COMMUNICATION    */
/* ******************* */
int8_t UG_driver_com_enable_config(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.com_enable_config)
        _ug_drv_hdl->fn.com_enable_config(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*       POWER         */
/* ******************* */
int8_t UG_driver_pwr_set_brightness(uint8_t lvl)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.pwr_set_brightness)
        _ug_drv_hdl->fn.pwr_set_brightness(&err, _ug_drv_hdl, lvl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_pwr_set_onoff(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.pwr_set_onoff)
        _ug_drv_hdl->fn.pwr_set_onoff(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_pwr_set_sleep(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.pwr_set_sleep)
        _ug_drv_hdl->fn.pwr_set_sleep(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_pwr_set_deepsleep(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.pwr_set_deepsleep)
        _ug_drv_hdl->fn.pwr_set_deepsleep(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*      SCREEN         */
/* ******************* */
int8_t UG_driver_scr_set_contrast(uint8_t lvl)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.scr_set_contrast)
        _ug_drv_hdl->fn.scr_set_contrast(&err, _ug_drv_hdl, lvl);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_scr_set_splash(uint8_t *data, uint32_t data_len)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data) || (data_len==0))
        return -1;

    if(_ug_drv_hdl->fn.scr_set_splash)
        _ug_drv_hdl->fn.scr_set_splash(&err, _ug_drv_hdl, data, data_len);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_scr_enable_splash(uint8_t flag)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.scr_enable_splash)
        _ug_drv_hdl->fn.scr_enable_splash(&err, _ug_drv_hdl, flag);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*     TOUCH PANEL     */
/* ******************* */
int8_t UG_driver_tch_calibrate(uint8_t value)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.tch_calibrate)
        _ug_drv_hdl->fn.tch_calibrate(&err, _ug_drv_hdl, value);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_tch_wait(int16_t *x, int16_t *y, uint8_t press_release_f)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.tch_wait)
        _ug_drv_hdl->fn.tch_wait(&err, _ug_drv_hdl, x, y, press_release_f);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_tch_check(int16_t *x, int16_t *y)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.tch_check)
        _ug_drv_hdl->fn.tch_check(&err, _ug_drv_hdl, x, y);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*       FLASH         */
/* ******************* */
int8_t UG_driver_flash_read(uint8_t* addr, uint8_t *len , uint8_t *data)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data))
        return -1;

    if(_ug_drv_hdl->fn.flash_read)
        _ug_drv_hdl->fn.flash_read(&err, _ug_drv_hdl, addr, len, data);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_flash_write(uint8_t* addr, uint8_t *len , uint8_t *data)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data))
        return -1;

    if(_ug_drv_hdl->fn.flash_write)
        _ug_drv_hdl->fn.flash_write(&err, _ug_drv_hdl, addr, len, data);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_flash_erase(uint8_t* addr, uint8_t *len)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if(_ug_drv_hdl->fn.flash_erase)
        _ug_drv_hdl->fn.flash_erase(&err, _ug_drv_hdl, addr, len);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

/* ******************* */
/*       EEPROM        */
/* ******************* */
int8_t UG_driver_eeprom_read(uint8_t* addr, uint8_t *len , uint8_t *data)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data))
        return -1;

    if(_ug_drv_hdl->fn.eeprom_read)
        _ug_drv_hdl->fn.eeprom_read(&err, _ug_drv_hdl, addr, len, data);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}

int8_t UG_driver_eeprom_write(uint8_t* addr, uint8_t *len , uint8_t *data)
{
    UG_DRV_ERR err = UG_DRV_NO_ERR;

    DRV_SANITY_CHECK(_ug_drv_hdl);

    if((NULL==data))
        return -1;

    if(_ug_drv_hdl->fn.eeprom_write)
        _ug_drv_hdl->fn.eeprom_write(&err, _ug_drv_hdl, addr, len, data);
    else
        return -1;

    if(UG_DRV_NO_ERR!=err) return -1;

    return 0;
}







