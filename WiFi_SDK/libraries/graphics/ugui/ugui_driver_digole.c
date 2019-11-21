/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_driver.h"
#include "ugui_driver_private.h"
#include "ugui_bus.h"

#define DIGOLE_PIXEL_TIME_uSEC (34)
#define I2C_WAKEUP_RETRY (5)

static const UG_DRV_FN digole_fn = {
    /* gfx_pixel_set          */ &UG_driver_digole_gfx_set_pixel,
    /* raw_fn                 */ &UG_driver_digole_raw_fn, 
    /* raw_set_defaults       */ &UG_driver_digole_raw_set_defaults,  
    /* sys_clear_screen       */ &UG_driver_digole_sys_clear_screen,  
    /* sys_set_backcolor      */ &UG_driver_digole_sys_set_backcolor, 
    /* sys_set_forecolor      */ &UG_driver_digole_sys_set_forecolor,  
    /* sys_set_draw_mode      */ &UG_driver_digole_sys_set_draw_mode,  
    /* sys_set_draw_rot       */ &UG_driver_digole_sys_set_draw_rot, 
    /* sys_set_window         */ &UG_driver_digole_sys_set_window,  
    /* sys_clear_window       */ &UG_driver_digole_sys_clear_window,  
    /* sys_set_fullscreen     */ &UG_driver_digole_sys_set_fullscreen, 
    /* sys_set_screen_refresh */ &UG_driver_digole_sys_set_screen_refresh,  
    /* sys_set_screen_norminv */ &UG_driver_digole_sys_set_screen_norminv,  
    /* sys_set_screen_update  */ NULL,
    /* sys_screen_scroll_start*/ NULL,
    /* sys_screen_scroll_stop */ NULL,
    /* txt_set_pos_rowcol     */ &UG_driver_digole_txt_set_pos_rowcol,  
    /* txt_set_pos_pixel      */ &UG_driver_digole_txt_set_pos_pixel,  
    /* txt_set_pos_back       */ &UG_driver_digole_txt_set_pos_back, 
    /* txt_write              */ &UG_driver_digole_txt_write, 
    /* txt_write_rowcol       */ &UG_driver_digole_txt_write_rowcol, 
    /* txt_write_xy           */ &UG_driver_digole_txt_write_xy,  
    /* txt_cr                 */ &UG_driver_digole_txt_cr,  
    /* txt_show_cursor        */ &UG_driver_digole_txt_show_cursor,
    /* fnt_upload             */ &UG_driver_digole_fnt_upload,
    /* fnt_usrlut             */ &UG_driver_digole_fnt_usrlut,  
    /* fnt_search             */ &UG_driver_digole_fnt_search,  
    /* fnt_select             */ &UG_driver_digole_fnt_select,  
    /* fnt_set                */ &UG_driver_digole_fnt_set,  
    /* gfx_set_pos            */ &UG_driver_digole_gfx_set_pos,  
    /* gfx_draw_lineto        */ &UG_driver_digole_gfx_draw_lineto, 
    /* gfx_set_linepattern    */ &UG_driver_digole_gfx_set_linepattern, 
    /* gfx_draw_line          */ &UG_driver_digole_gfx_draw_line,
    /* gfx_draw_rect          */ &UG_driver_digole_gfx_draw_rect,  
    /* gfx_fill_rect          */ &UG_driver_digole_gfx_fill_rect, 
    /* gfx_draw_circle        */ &UG_driver_digole_gfx_draw_circle, 
    /* gfx_fill_circle        */ &UG_driver_digole_gfx_fill_circle,  
    /* gfx_draw_arc           */ NULL, 
    /* gfx_fill_arc           */ NULL,  
    /* gfx_draw_image         */ &UG_driver_digole_gfx_draw_image,
    /* gfx_move               */ &UG_driver_digole_gfx_move, 
    /* gfx_draw_poly          */ NULL,
    /* gfx_fill_poly          */ NULL,
    /* gfx_draw_polyreg       */ NULL,
    /* gfx_fill_polyreg       */ NULL,
    /* com_enable_config      */ &UG_driver_digole_com_enable_config, 
    /* pwr_set_brightness     */ &UG_driver_digole_pwr_set_brightness,  
    /* pwr_set_onoff          */ &UG_driver_digole_pwr_set_onoff,  
    /* pwr_set_sleep          */ &UG_driver_digole_pwr_set_sleep, 
    /* pwr_set_deepsleep      */ &UG_driver_digole_pwr_set_deepsleep,  
    /* scr_set_contrast       */ &UG_driver_digole_scr_set_contrast,  
    /* scr_set_splash         */ &UG_driver_digole_scr_set_splash,  
    /* scr_enable_splash      */ &UG_driver_digole_scr_enable_splash,  
    /* tch_calibrate          */ NULL, 
    /* tch_wait               */ NULL,  
    /* tch_check              */ NULL,  
    /* flash_read             */ NULL,  
    /* flash_write            */ NULL,  
    /* flash_erase            */ NULL,  
    /* eeprom_read            */ NULL,  
    /* eeprom_write           */ NULL
};

static inline void digole_delay(UG_DRV_HANDLE drv_handle)
{
    /* tbd: make this a parameter of ver_mjr/min for the display */
    if(drv_handle->w_byte >= 512)
    {
        wiced_rtos_delay_milliseconds(16);
        drv_handle->w_byte = 0;
    }
}

static inline void digole_i2c_wakeup(UG_DRV_HANDLE drv_handle)
{
    /* NOTE: there is a very tricky detail documented in the Digole specs */
    /* if you are using I2C the PIC cpu might be in powerdown, even after the */
    /* first i2c probing. To make sure the PIC is awake you just send zeros with */
    /* a custom delay of 10ms */
    /* --- */
    /* The module wake up automatically when new data received, but if the COM mode is I2C, some dummy data are */
    /* needed to act as waking signal, so, use few write(0) then a delay 10ms is a good practice to wake up the MCU from */
    /* deep sleep. */
    /* --- */
    /* This is a BIG delay and so this wakeup function cannot be used in primitves */
    /* that are likely be used in a FAST loop, so only calls like CLEAR_SCREN etc */
    /* will include this custom wakup call */

    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    int8_t k = 0;

    uint8_t digole_cmd_wakeup[1]= {0};
    if(drv_handle->cfg.hw_bus == (UG_DRV_CFG_DEV_BUS_I2C))
    {
        payload.tx_buffer = digole_cmd_wakeup;
        payload.length = 1;
        payload.rx_buffer = NULL;

        for(k=0; k < I2C_WAKEUP_RETRY; k++)
        {
            UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
            wiced_rtos_delay_milliseconds( 5 );
        }
    }
}

void UG_driver_digole_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;

    if(UG_DRV_WMARK != drv_handle->wmark)
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }

    /* sanity check on minimum hw rev*/
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) < 28 /*v2.8*/ )
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }

    if( err_tmp == UG_DRV_NO_ERR)
    {
        memcpy(&drv_handle->fn, &digole_fn, sizeof(digole_fn));
    }

    if(NULL!=(err)) *err = err_tmp;
}

void UG_driver_digole_gfx_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[] = {'E','S','C',0x00,0x00,0x00,'D','P',0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[] = {'D','P',0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;

    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;
    
    /* add color select command only when needed */
    if(drv_handle->last_color!=c)
    {
        drv_handle->last_color = c;
        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        // color rbg
        digole_cmd[3] = (uint8_t)((c >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((c >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((c >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**) &payloads, 1, (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_raw_fn( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t msg, void *data, uint32_t len)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* raw write to the bus */

    payload.tx_buffer = (uint8_t*)data;
    payload.length = len;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,  (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_raw_set_defaults(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* default settings will include a clear screen, disable cursor and disable splash screen */
    /* the CL command also resets the font and draw direction */
    uint8_t digole_cmd[12]= {'C','L','C','S',0x00,'D','S','S',0x00,'D','C',0x00};
    payload.tx_buffer = digole_cmd;
    payload.length = 12;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

}


/* ******************* */
/* SYSTEM */
/* ******************* */

void UG_driver_digole_sys_clear_screen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /*note: we adda TP because from the spec:
    The module will not execute this command until
    other command received.
    */
    uint8_t digole_cmd_CL[6]= {'C','L','T','P',0,0};
    payload.tx_buffer = digole_cmd_CL;
    payload.length = 6;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    wiced_rtos_delay_milliseconds(40);
}

void UG_driver_digole_sys_set_backcolor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint32_t c)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_scbcg[]=   {'S','C',0x00,'B','C','G','S','C',0x00};
    uint8_t digole_cmd_escbcg[]=  {'S','C',0x00,'B','C','G','E','S','C',0x00,0x00,0x00};
    uint8_t digole_cmd_bcg[]=  {'B','C','G',0x00};
    uint8_t *digole_cmd = NULL;

    /* only available in ver > 3.2 */
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) < 32 /*3.2*/ )
    {
        if(NULL!=err) *err = UG_DRV_ERR_NOT_SUPPORTED;
        return;
    }
    
    /* hw ver 3.2 is different from ver 3.3 */
    if( (drv_handle->cfg.hw_mjr = 3) && 
        (drv_handle->cfg.hw_min = 2) ) 
    {
        /* 3.2 */
        /* select color command based on driver color fmt config */
        switch(drv_handle->cfg.fmt_color)
        {
            case UG_DRV_FMT_RGB332:
                digole_cmd = digole_cmd_scbcg;
                payload.tx_buffer = digole_cmd;
                payload.rx_buffer = NULL;
                digole_cmd[2] = (uint8_t)((c >> 0) & 0x0FF);
                digole_cmd[8] = (uint8_t)((drv_handle->foreground_color >> 0) & 0x0FF);
                payload.length = 9;
                break;
            case UG_DRV_FMT_RGB888:
                digole_cmd = digole_cmd_escbcg;
                payload.tx_buffer = digole_cmd;
                payload.rx_buffer = NULL;
                digole_cmd[ 2] = (uint8_t)((c >> 0) & 0x0FF);
                digole_cmd[ 9] = (uint8_t)((drv_handle->foreground_color >>16) & 0xFF);
                digole_cmd[10] = (uint8_t)((drv_handle->foreground_color >> 8) & 0xFF);
                digole_cmd[11] = (uint8_t)((drv_handle->foreground_color >> 0) & 0xFF);
                payload.length = 12;
                break;
            default:
                /* unsupported color format */
                if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
                return;
        }
    }
    else
    {
        /* >3.2 */
        digole_cmd = digole_cmd_bcg;
        payload.tx_buffer = digole_cmd;
        payload.rx_buffer = NULL;
        digole_cmd[ 3] = (uint8_t)((c >> 0) & 0x0FF);
        payload.length = 4;
    }


    /* add color select command only when needed */
    if(drv_handle->background_color != c)
    {
        drv_handle->background_color = c;

        UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
        if(UG_BUS_NO_ERR==bus_err)
        {
            drv_handle->w_byte += payload.length;
            digole_delay(drv_handle);
        }
        else
        {
            if(NULL!=err) *err = UG_DRV_ERR_IO;
        }
    }
}

void UG_driver_digole_sys_set_forecolor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint32_t c)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_esc[]=  {'E','S','C',0x00,0x00,0x00};
    uint8_t digole_cmd_sc[]=   {'S','C',0x00};
    uint8_t *digole_cmd = NULL;

    /* select color command based on driver color fmt config */
    switch(drv_handle->cfg.fmt_color)
    {
        case UG_DRV_FMT_MONO:
            digole_cmd = digole_cmd_sc;
            payload.tx_buffer = digole_cmd;
            payload.rx_buffer = NULL;
            digole_cmd[2] = (uint8_t)((c >> 0) & 0x01);
            payload.length = 3;
            break;
        case UG_DRV_FMT_RGB332:
            digole_cmd = digole_cmd_sc;
            payload.tx_buffer = digole_cmd;
            payload.rx_buffer = NULL;
            digole_cmd[2] = (uint8_t)((c >> 0) & 0xFF);
            payload.length = 3;
            break;
        case UG_DRV_FMT_RGB888:
            digole_cmd = digole_cmd_esc;
            payload.tx_buffer = digole_cmd;
            payload.rx_buffer = NULL;
            digole_cmd[3] = (uint8_t)((c >>16) & 0xFF);
            digole_cmd[4] = (uint8_t)((c >> 8) & 0xFF);
            digole_cmd[5] = (uint8_t)((c >> 0) & 0xFF);
            payload.length = 6;
            break;
        default:
            /* unsupported color format */
            if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
            return;
    }

    /* add color select command only when needed */
    if(drv_handle->foreground_color != c)
    {
        drv_handle->foreground_color = c;

        UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START| UG_BUS_MSG_COALESCE_STOP));
        if(UG_BUS_NO_ERR==bus_err)
        {
            drv_handle->w_byte += payload.length;
            digole_delay(drv_handle);
        }
        else
        {
            if(NULL!=err) *err = UG_DRV_ERR_IO;
        }
    }
}

void UG_driver_digole_sys_set_draw_mode(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_CL[3]= {'D','M',mode};
    payload.tx_buffer = digole_cmd_CL;
    payload.length = 3;
    payload.rx_buffer = NULL;

    /* filter valid modes */
    //if(!( (mode=='C') || (mode=='|') || (mode=='^') || (mode=='!') || (mode=='&') ))
    //{
    //    if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
    //    return;
    //}

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START| UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_sys_set_draw_rot(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t deg)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd_CL[3]= {'S','D',0x00};
    payload.tx_buffer = digole_cmd_CL;
    payload.length = 3;
    payload.rx_buffer = NULL;
    
    /* digole has only 4 directions, 0, 90, 180, 270 */
    /* so we approximate rounding */
    int16_t digole_adj = 0;
    uint8_t digole_dir = 0;

    if(deg >= 0)
    {
        /* round to the next quarter */
        digole_adj = (deg + 45) % 360;

        digole_dir = (digole_adj >=  90) ? 1 : digole_dir;
        digole_dir = (digole_adj >= 180) ? 2 : digole_dir;
        digole_dir = (digole_adj >= 270) ? 3 : digole_dir;
    }
    else
    {
        /* round to the next quarter */
        digole_adj = (deg - 45) % 360;

        digole_dir = (digole_adj >=  -90) ? 3 : digole_dir;
        digole_dir = (digole_adj >= -180) ? 2 : digole_dir;
        digole_dir = (digole_adj >= -270) ? 1 : digole_dir;
    }

    digole_cmd_CL[2] = digole_dir;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_sys_set_window(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y,  uint16_t w, uint16_t h)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[9]= {'D','W','W','I','N',0x00,0x00,0x00,0x00};
    payload.tx_buffer = digole_cmd;
    payload.length = 5;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if( (drv_handle->cfg.hw_mjr < 3) )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    // x1
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    // w
    if(w<256){
        digole_cmd[payload.length] = w;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (w - 255);
        payload.length++;
        digole_cmd[payload.length] = (w >> 0) & 0xFF;
        payload.length++;
    }

    // h
    if(h<256){
        digole_cmd[payload.length] = h;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (h >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = (h >> 0) & 0xFF;
        payload.length++;
    }


    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_sys_clear_window(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[5]= {'W','I','N','C','L'};
    payload.tx_buffer = digole_cmd;
    payload.length = 5;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if( (drv_handle->cfg.hw_mjr < 3) )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_sys_set_fullscreen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[5]= {'R','S','T','D','W'};
    payload.tx_buffer = digole_cmd;
    payload.length = 5;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if( (drv_handle->cfg.hw_mjr < 3) )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_sys_set_screen_refresh(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[3]= {'F','S', (mode & 0x01)};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) < 28 /*v2.8*/ )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_sys_set_screen_norminv(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[4]= {'I','N', 'V', (flag & 0x01)};
    payload.tx_buffer = digole_cmd;
    payload.length = 4;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if(drv_handle->cfg.fmt_color != UG_DRV_FMT_MONO)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

/* ******************* */
/* TEXT */
/* ******************* */
void UG_driver_digole_txt_set_pos_rowcol(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t cx1, int16_t cy1)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[6]= {'T','P',0x00,0x00,0x00,0x00};
    payload.tx_buffer = digole_cmd;
    payload.length = 2;
    payload.rx_buffer = NULL;
    
    // x1
    if(cx1<256){
        digole_cmd[payload.length] = cx1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cx1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cx1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(cy1<256){
        digole_cmd[payload.length] = cy1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cy1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cy1 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_txt_set_pos_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x1, int16_t y1)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[7]= {'E','T','P',0x00,0x00,0x00,0x00};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;
    
    // x1
    if(x1<256){
        digole_cmd[payload.length] = x1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(y1<256){
        digole_cmd[payload.length] = y1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y1 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}
void UG_driver_digole_txt_set_pos_back(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[3]= {'E','T','B'};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,  (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_txt_write(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[text_len+3];

    /* digole needs a 0x00 termination on the string */
    digole_cmd[0] = 'T';
    digole_cmd[1] = 'T';
    memcpy(&digole_cmd[2], text, text_len);
    digole_cmd[text_len+2] = 0x00;

    payload.tx_buffer = digole_cmd;
    payload.length = text_len+3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_txt_write_rowcol(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len, int16_t cx, int16_t ry)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[text_len+8];

    /* digole needs a 0x00 termination on the string */
    digole_cmd[0] = 'T';
    digole_cmd[1] = 'P';
    payload.length = 2;

    // x
    if(cx<256){
        digole_cmd[payload.length] = cx;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cx - 255);
        payload.length++;
        digole_cmd[payload.length] = (cx >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(ry<256){
        digole_cmd[payload.length] = ry;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (ry - 255);
        payload.length++;
        digole_cmd[payload.length] = (ry >> 0) & 0xFF;
        payload.length++;
    }

    digole_cmd[payload.length] = 'T';
    payload.length++;
    digole_cmd[payload.length] = 'T';
    payload.length++;

    /* copy&terminate string */
    memcpy(&digole_cmd[payload.length], text, text_len);
    payload.length += text_len;
    digole_cmd[payload.length] = 0x00;
    payload.length++;

    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_txt_write_xy(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, char* text, uint32_t text_len, int16_t x, int16_t y)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[text_len+8];

    /* digole needs a 0x00 termination on the string */
    digole_cmd[0] = 'E';
    digole_cmd[1] = 'T';
    digole_cmd[2] = 'P';
    payload.length = 3;

    // x
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    digole_cmd[payload.length] = 'T';
    payload.length++;
    digole_cmd[payload.length] = 'T';
    payload.length++;

    /* copy&terminate string */
    memcpy(&digole_cmd[payload.length], text, text_len);
    payload.length += text_len;
    digole_cmd[payload.length] = 0x00;
    payload.length++;

    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}


void UG_driver_digole_txt_cr(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[3]= {'T','R','T'};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,  (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_txt_show_cursor(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t mode, uint8_t onoff_f)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd[3]= {'C','S',(onoff_f & 0x01)};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}


/* ******************* */
/* FONT */
/* ******************* */
void UG_driver_digole_fnt_upload(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t hw_id, void *ug_font, uint8_t *data, uint32_t data_len)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd[data_len+6];

    uint8_t hw_id_valid_f = 0;
    uint8_t hw_id_idx     = 0;

    uint16_t io_write_idx  = 0;

    //uint8_t digole_cmd[3]= {'S','U',F','S',(onoff_f & 0x01)};
    //payload.tx_buffer = digole_cmd;
    //payload.length = 3;
    //payload.rx_buffer = NULL;

    /* Note: the digole specification for font upload is quite cumbersome:
    If there is no flash chip on the module or 3.3 you can download your user font to 1 of 4 user font space by using this
    command. If there is a flash chip onboard and the firmware is 3.2 or older, you can’t use this function, but instead of
    it, you can save user font at any address in flash chip and use it.
    Command: SUF, follow by a byte of the index number of user font space, then 2 bytes of data length of font, then the
    font data. Please note, in 3.2 and early version, the font length send to module is: LSB, MSB formation, but since 3.3
    the format changed to MSB, LSB.
    */
    if( (drv_handle->cfg.flash_enabled) &&
        ( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) <= 32 /*v3.2*/ ))
    {
        if(NULL!=err) *err = UG_DRV_ERR_NOT_SUPPORTED;
        return;
    }
    
    /* local font space is limited to 4KB each */
    if( ( !(drv_handle->cfg.flash_enabled) ) && 
        (data_len > UG_DRIVER_DIGOLE_USR_FNT_BYTES) )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* hw_id must be valid */
    hw_id_valid_f = 0;
    hw_id_idx = 0;
    while((!hw_id_valid_f)&&(hw_id_idx < UG_DRIVER_DIGOLE_USR_FNT_MAX))
    {
        hw_id_valid_f = (hw_id == ug_driver_digole_usr_fnt_code[hw_id_idx]);
        hw_id_idx++;
    }

    if(!(hw_id_valid_f))
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* prepare the command and write chunks since from the spec: 
    40ms delay is needed after each 64 bytes of data send out, refer “Write data to flash” command
    */
    digole_cmd[0] = 'S';
    digole_cmd[1] = 'U';
    digole_cmd[2] = 'F';
    digole_cmd[3] = hw_id_idx-1 ;//hw_id;

    /* data ordering changed over fw revisions */
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) <= 32 /*v3.2*/ )
    {
        /* LSB...MSB */
        digole_cmd[4] = (data_len >> 0) & 0xFF;
        digole_cmd[5] = (data_len >> 8) & 0xFF;
    }
    else
    {
        /* MSB...LSB */
        digole_cmd[4] = (data_len >> 8) & 0xFF;
        digole_cmd[5] = (data_len >> 0) & 0xFF;
    }

    /* copy font bitstream */
    memcpy(&digole_cmd[6], data, data_len);

    /* loop over write chunks with delays */
    /* NOTE: from the documentation and from many tests it seems that we cannot
    write the font in burst (expecially on spi bus) but we have to do it byte by byte
    and we have to keep 8/10ms for each write since the onboard flash is VERY SLOW.
    Also add a delay every 64 byte write and one at the very end to make sure the
    comman input buffer has been drained.
    */

#define FONT_CHUNK_SIZE (1)
    io_write_idx  = 0;
    uint8_t tmp=0;

    uint32_t msg_flag = UG_BUS_MSG_COALESCE_START;
    while(io_write_idx < (data_len+6))
    {
        payload.tx_buffer = &digole_cmd[io_write_idx];
        payload.length    = ((io_write_idx+FONT_CHUNK_SIZE)<(data_len+6)) ? FONT_CHUNK_SIZE  : ((data_len+6)-io_write_idx);
        payload.rx_buffer = NULL;

        io_write_idx += payload.length;

        if(io_write_idx > (data_len+6))
        {
            msg_flag |= UG_BUS_MSG_COALESCE_STOP;
        }

        UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, msg_flag);
        if(UG_BUS_NO_ERR==bus_err)
        {
            drv_handle->w_byte += payload.length;
            //digole_delay(drv_handle);
            wiced_rtos_delay_milliseconds(20);
        }
        else
        {
            if(NULL!=err) *err = UG_DRV_ERR_IO;
            return;
        }

        msg_flag = UG_BUS_MSG_COALESCE_CHUNK;

        if(++tmp==64)
        {
            wiced_rtos_delay_milliseconds(100);
            tmp=0;
        }
    }

    /* if a UGUI_FONT ptr is provided update the search LUT */
    if(NULL!=ug_font)
    {
        uint8_t i =0;
        /* search for a free slot in the LUT */
        while( (i<UG_DRV_MAX_HW_FONT_NUM) && 
               (NULL!=drv_handle->font_lut[i].ug_font_ptr) )
        {
            if(ug_font == drv_handle->font_lut[i].ug_font_ptr)
            {
               break;
            }
            i++;
        }

        if(i<UG_DRV_MAX_HW_FONT_NUM)
        {
            drv_handle->font_lut[i].ug_font_ptr = ug_font;
            drv_handle->font_lut[i].hw_id = hw_id;
        }
    }

    /* after font upload the pic needs some more time */
    wiced_rtos_delay_milliseconds(400);
}

void UG_driver_digole_fnt_usrlut(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, UG_DRV_FONT_LUT lut[], uint8_t lut_len)
{
    uint8_t i = 0;
    uint8_t k = 0;

    /* ERASE PREV TABLE */
    for (i=0; i<UG_DRV_MAX_HW_FONT_NUM; i++)
    {
        drv_handle->font_lut[i].ug_font_ptr = NULL;
        drv_handle->font_lut[i].hw_id=0;
    }

    /* NEW TABLE */
    i=0;
    k=0;
    while( (k<UG_DRV_MAX_HW_FONT_NUM) &&
           (i<lut_len) )
    {
        if(NULL!=lut[i].ug_font_ptr)
        {
            drv_handle->font_lut[k].ug_font_ptr = lut[i].ug_font_ptr;
            drv_handle->font_lut[k].hw_id       = lut[i].hw_id;
            k++;
        }
        i++;
    }
}

void UG_driver_digole_fnt_search(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *hw_id, void *ug_font)
{
    uint8_t i = 0;

    /* SANITY CHECK */
    if ( (NULL==hw_id) || (NULL==ug_font) )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* FALLBACK ON HW_ID=0, the default font */
    *hw_id = 0;

    /* SEARCH MATCH */
    i=0;
    do
    {
        if(ug_font == drv_handle->font_lut[i].ug_font_ptr)
            *hw_id =  drv_handle->font_lut[i].hw_id;
        i++;
    }
    while((i<UG_DRV_MAX_HW_FONT_NUM) && (NULL!=drv_handle->font_lut[i].ug_font_ptr));
}

void UG_driver_digole_fnt_select(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, void *ug_font)
{
    uint8_t res;

    if(NULL==ug_font)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_driver_digole_fnt_search(err, drv_handle, &res, ug_font);
    if(UG_DRV_NO_ERR == *err)
    {
        UG_driver_digole_fnt_set(err, drv_handle, res);
    }
}

void UG_driver_digole_fnt_set(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t hw_id)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd[3]= {'S','F',hw_id};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    /* sanity check on font index */
    uint8_t match=0;
    uint8_t idx = 0;

    if(hw_id>=ug_driver_digole_usr_fnt_code[0])
    {
        while( (!match) && (idx< UG_DRIVER_DIGOLE_USR_FNT_MAX) )
        {
            match = (ug_driver_digole_usr_fnt_code[idx] == hw_id);
            idx++;
        }
    }
    else
    {
        while( (!match) && (idx< UG_DRIVER_DIGOLE_STD_FNT_MAX) )
        {
            match = (ug_driver_digole_std_fnt_code[idx] == hw_id);
            idx++;
        }
    }

    if(!match)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    /* on digole the forecolor might be affected by gfx operations */
    /* so on font change we reset to white and we track it         */
    drv_handle->foreground_color = (0xFF<<16) | (0xFF<<8) | 0xFE;
    UG_driver_digole_sys_set_forecolor(err,drv_handle, drv_handle->foreground_color);
}

/* ******************* */
/* GRAPHICS */
/* ******************* */

void UG_driver_digole_gfx_set_pos(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd_2x[6]= {'G','P',0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_3x[7]= {'E','T','P',0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = NULL;

    /* difference in api from the spec*/
    if( (drv_handle->cfg.hw_mjr < 3) )
    {
        digole_cmd = digole_cmd_2x;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
        payload.rx_buffer = NULL;
    }
    else
    {
        digole_cmd = digole_cmd_3x;
        payload.tx_buffer = digole_cmd;
        payload.length = 3;
        payload.rx_buffer = NULL;

    }

    
    // x
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_gfx_draw_lineto(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'L','T',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'L','T',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color!=argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x2
    if(x2<256){
        digole_cmd[payload.length] = x2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x2 >> 0) & 0xFF;
        payload.length++;
    }

    // y2
    if(y2<256){
        digole_cmd[payload.length] = y2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y2 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_gfx_set_linepattern(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t pattern)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[4]= {'S','L','P',pattern};
    payload.tx_buffer = digole_cmd;
    payload.length = 4;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,  (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_gfx_draw_line(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'L','N',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'L','N',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color!=argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x1
    if(x1<256){
        digole_cmd[payload.length] = x1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(y1<256){
        digole_cmd[payload.length] = y1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y1 >> 0) & 0xFF;
        payload.length++;
    }

    // x2
    if(x2<256){
        digole_cmd[payload.length] = x2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x2 >> 0) & 0xFF;
        payload.length++;
    }

    // y2
    if(y2<256){
        digole_cmd[payload.length] = y2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y2 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_gfx_draw_rect(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                    int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'D','R',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'D','R',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color != argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x1
    if(x1<256){
        digole_cmd[payload.length] = x1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(y1<256){
        digole_cmd[payload.length] = y1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y1 >> 0) & 0xFF;
        payload.length++;
    }

    // x2
    if(x2<256){
        digole_cmd[payload.length] = x2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x2 >> 0) & 0xFF;
        payload.length++;
    }

    // y2
    if(y2<256){
        digole_cmd[payload.length] = y2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y2 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

#if 0
    /* digole needs time to fill an are    */
    /* typ speed seems to be 22-24usec/pixel  */
    /* stall until is done to prevent overlapping of pixels */
    if((x1<x2) && (y1<y2))
    {
        uint32_t tmp_delay = ((x2-x1) * (y2-y1) * DIGOLE_PIXEL_TIME_uSEC)/1000;
        wiced_rtos_delay_milliseconds(tmp_delay);
    }
#endif
}

void UG_driver_digole_gfx_fill_rect(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'F','R',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'F','R',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color != argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x1
    if(x1<256){
        digole_cmd[payload.length] = x1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(y1<256){
        digole_cmd[payload.length] = y1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y1 >> 0) & 0xFF;
        payload.length++;
    }

    // x2
    if(x2<256){
        digole_cmd[payload.length] = x2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (x2 >> 0) & 0xFF;
        payload.length++;
    }

    // y2
    if(y2<256){
        digole_cmd[payload.length] = y2;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y2 - 255);
        payload.length++;
        digole_cmd[payload.length] = (y2 >> 0) & 0xFF;
        payload.length++;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }


    /* digole needs time to fill an are    */
    /* typ speed seems to be 22-24usec/pixel  */
    /* stall until is done to prevent overlapping of pixels */
    if((x1<x2) && (y1<y2))
    {
        uint32_t tmp_delay = ((x2-x1) * (y2-y1) * DIGOLE_PIXEL_TIME_uSEC)/1000;
        wiced_rtos_delay_milliseconds(tmp_delay);
    }
}


void UG_driver_digole_gfx_draw_circle(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                  int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'C','C',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'C','C',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color != argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x1
    if(cx1<256){
        digole_cmd[payload.length] = cx1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cx1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cx1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(cy1<256){
        digole_cmd[payload.length] = cy1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cy1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cy1 >> 0) & 0xFF;
        payload.length++;
    }

    // r
    if(r<256){
        digole_cmd[payload.length] = r;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (r - 255);
        payload.length++;
        digole_cmd[payload.length] = (r >> 0) & 0xFF;
        payload.length++;
    }

    // f=0
    digole_cmd[payload.length] = 0;
    payload.length++;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_gfx_fill_circle(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                  int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd_long[]=  {'E','S','C',0x00,0x00,0x00,'C','C',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t digole_cmd_short[]= {'C','C',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t *digole_cmd = digole_cmd_long;
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* add color select command only when needed */
    if(drv_handle->last_color != argb)
    {
        drv_handle->last_color = argb;

        digole_cmd = digole_cmd_long;
        payload.tx_buffer = digole_cmd;
        digole_cmd[3] = (uint8_t)((argb >>16) & 0xFF);
        digole_cmd[4] = (uint8_t)((argb >> 8) & 0xFF);
        digole_cmd[5] = (uint8_t)((argb >> 0) & 0xFF);
        payload.length = 8;
    }
    else
    {
        digole_cmd = digole_cmd_short;
        payload.tx_buffer = digole_cmd;
        payload.length = 2;
    }

    // x1
    if(cx1<256){
        digole_cmd[payload.length] = cx1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cx1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cx1 >> 0) & 0xFF;
        payload.length++;
    }

    // y1
    if(cy1<256){
        digole_cmd[payload.length] = cy1;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (cy1 - 255);
        payload.length++;
        digole_cmd[payload.length] = (cy1 >> 0) & 0xFF;
        payload.length++;
    }

    // r
    if(r<256){
        digole_cmd[payload.length] = r;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (r - 255);
        payload.length++;
        digole_cmd[payload.length] = (r >> 0) & 0xFF;
        payload.length++;
    }

    // f=0
    digole_cmd[payload.length] = 0x01;
    payload.length++;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    /* digole needs time to fill an area    */
    /* typ speed seems to be 22-24usec/pixel  */
    /* stall until is done to prevent overlapping of pixels */
    {
        //pi approx to 3
        uint32_t tmp_delay = ( 3 * r * r * DIGOLE_PIXEL_TIME_uSEC)/1000;
        wiced_rtos_delay_milliseconds(tmp_delay);
    }
}

void UG_driver_digole_gfx_draw_image(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                     int16_t x, int16_t y, UG_DRV_CFG_FMT_COLOR type, uint8_t *pixels, uint16_t w, uint16_t h, void* usr_fmt)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    uint8_t digole_cmd[20];
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;
    payload.length    = 0;

    uint16_t img_data_bytes = 0;

    /* SANITY CHECK */
    if((NULL==pixels)||(w==0)||(h==0))
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* only few modes supported by digole */
    if( ! ((type==UG_DRV_FMT_MONO)||(type==UG_DRV_FMT_RGB332)||
           (type==UG_DRV_FMT_RGB565)||(type==UG_DRV_FMT_RGB888) ))
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* sanity check: for mono we do not want to compute padding (tbd) */
    /* 
    DIM, follow by the top-left coordinate (x,y) of the image, then image width (w), 
    height (h), then follow the image data (…d…), each bit represent a pixel in the image.
    Data in one byte can’t cross different lines, that means, if the width of image is 
    12 pixels, you need 2 bytes for each line. MSB is on the left side.
    */
    if(( type == UG_DRV_FMT_MONO )&&(w & 0x3))
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* select initial cmd */
    switch(type)
    {
        case UG_DRV_FMT_MONO:
            digole_cmd[0]='D';
            digole_cmd[1]='I';
            digole_cmd[2]='M';
            payload.length = 3;
            img_data_bytes = (w>>3)*h;
            break;

        case UG_DRV_FMT_RGB332:
            digole_cmd[0]='E';
            digole_cmd[1]='D';
            digole_cmd[2]='I';
            digole_cmd[3]='M';
            digole_cmd[4]='1';
            payload.length = 5;
            img_data_bytes = w*h;
            break;

        case UG_DRV_FMT_RGB565:
            digole_cmd[0]='E';
            digole_cmd[1]='D';
            digole_cmd[2]='I';
            digole_cmd[3]='M';
            digole_cmd[4]='2';
            payload.length = 5;
            img_data_bytes = 2*w*h;
            break;

        case UG_DRV_FMT_RGB888:
            digole_cmd[0]='E';
            digole_cmd[1]='D';
            digole_cmd[2]='I';
            digole_cmd[3]='M';
            digole_cmd[4]='3';
            payload.length = 5;
            img_data_bytes = 3*w*h;
            break;

        default:
            img_data_bytes = 0;
            break;
    }

    /* exit on wrong size */
    if(img_data_bytes==0)
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* next add x, y, w, h */

    // x
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    // w
    if(w<256){
        digole_cmd[payload.length] = w;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (w >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = (w >> 0) & 0xFF;
        payload.length++;
    }

    // h
    if(h<256){
        digole_cmd[payload.length] = h;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (h >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = (h >> 0) & 0xFF;
        payload.length++;
    }

    /* images can be large, we do not want to memcpy in a local buf! */

    /* 1ST: send the command*/
    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    /* 2ND: send the pixels */
    payload.tx_buffer = pixels;
    payload.rx_buffer = NULL;
    payload.length    = img_data_bytes;
    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    /* digole needs time to render/fill an area             */
    /* speed is dependent on color fmt                      */
    /* typ speed seems to be 22-24usec/pixel for RGB888     */
    /* stall until is done to prevent overlapping of pixels */
    {
        //pi approx to 3
        uint32_t tmp_delay = ( w * h * DIGOLE_PIXEL_TIME_uSEC)/1000;
        wiced_rtos_delay_milliseconds(tmp_delay);
    }
}
void UG_driver_digole_gfx_move(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                               int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t off_x, int16_t off_y)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    /* MA x y w h Ox Oy */
    uint8_t digole_cmd[]=  {'M','A',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;
    payload.length = 2;

    /* sanity check: offset is limited to -127 to +127 */
    if( (off_x > 127) || (off_x < -127) ||
        (off_y > 127) || (off_y < -127) )

    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    // x
    if(x<256){
        digole_cmd[payload.length] = x;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (x - 255);
        payload.length++;
        digole_cmd[payload.length] = (x >> 0) & 0xFF;
        payload.length++;
    }

    // y
    if(y<256){
        digole_cmd[payload.length] = y;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (y - 255);
        payload.length++;
        digole_cmd[payload.length] = (y >> 0) & 0xFF;
        payload.length++;
    }

    // w
    if(width<256){
        digole_cmd[payload.length] = width;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (width >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = (width >> 0) & 0xFF;
        payload.length++;
    }

    // h
    if(height<256){
        digole_cmd[payload.length] = height;
        payload.length++;
    }
    else
    {
        digole_cmd[payload.length] = (height >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = (height >> 0) & 0xFF;
        payload.length++;
    }

    // Ox
    digole_cmd[payload.length] = off_x;
    payload.length++;

    // Oy
    digole_cmd[payload.length] = off_y;
    payload.length++;


    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }

    /* digole needs time to fill an area    */
    /* typ speed seems to be 22-24usec/pixel  */
    /* stall until is done to prevent overlapping of pixels */
    {
        //pi approx to 3
        uint32_t tmp_delay = ( width * height * DIGOLE_PIXEL_TIME_uSEC)/1000;
        wiced_rtos_delay_milliseconds(tmp_delay);
    }
}


/* ******************* */
/* COMMUNICATION */
/* ******************* */
void UG_driver_digole_com_enable_config(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[3]= {'D','C', (flag & 0x01)};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;
    
    /* sanity check on minimum hw rev*/
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) <= 28 /*v2.8*/ )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

/* ******************* */
/* POWER */
/* ******************* */

void UG_driver_digole_pwr_set_brightness(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    uint8_t digole_cmd[3]= {'B','L', lvl};
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_pwr_set_onoff(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);
    
    uint8_t digole_cmd[4]= {'S','O','O', (flag & 0x01) };
    payload.tx_buffer = digole_cmd;
    payload.length = 4;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,  (UG_BUS_MSG_COALESCE_START | UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_pwr_set_sleep(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd_DNMCU[5]= {'D','N','M','C','U' };
    uint8_t digole_cmd_BL[9]= {'B','L',99,'B','L',99,'B','L',99};
    uint8_t *digole_cmd = NULL;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* 
    we use a BL command to wake up and to be bus-agnostic, since from the DIGOLE specs:
    The module wake up automatically when new data received, but if the COM mode is I2C, some dummy data are
    needed to act as waking signal, so, use few write(0) then a delay 10ms is a good practice to wake up the MCU from
    deep sleep.
    The screen will keep on, and all content on the screen unchanged when MCU off.
    */

    if(flag&0x01)
    {
        digole_cmd = digole_cmd_DNMCU;
        payload.tx_buffer = digole_cmd;
        payload.length = 5;
        payload.rx_buffer = NULL;
    }
    else
    {
        digole_cmd = digole_cmd_BL;
        payload.tx_buffer = digole_cmd;
        payload.length = 9;
        payload.rx_buffer = NULL;
    }


    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_pwr_set_deepsleep(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd_DNMCU[5]= {'D','N','A','L','L' };
    uint8_t digole_cmd_BL[9]= {'B','L',99,'B','L',99,'B','L',99 };
    uint8_t *digole_cmd = NULL;

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* 
    we use a BL command to wake up and to be bus-agnostic, since from the DIGOLE specs:
    This command put all power off: backlight off, screen off, MCU enter deep sleep, the module will only consume
    <0.05mA of current, the wake up sequence is same with wake up MCU, the module will restore backlight and put
    screen on also after wake up, the content on the screen unchanged.
    */

    if(flag&0x01)
    {
        digole_cmd = digole_cmd_DNMCU;
        payload.tx_buffer = digole_cmd;
        payload.length = 5;
        payload.rx_buffer = NULL;
    }
    else
    {
        digole_cmd = digole_cmd_BL;
        payload.tx_buffer = digole_cmd;
        payload.length = 9;
        payload.rx_buffer = NULL;
    }


    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}


/* ******************* */
/* SCREEN */
/* ******************* */
void UG_driver_digole_scr_set_contrast(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[4]= {'C','T', lvl };
    payload.tx_buffer = digole_cmd;
    payload.length = 3;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1,(UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

void UG_driver_digole_scr_set_splash(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *data, uint32_t data_len)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[data_len+8];

    uint16_t io_write_idx  = 0;

    digole_cmd[0] = 'S';
    digole_cmd[1] = 'S';
    digole_cmd[2] = 'S';
    payload.length = 3;

    /* note: limited to 2K !!
    Set Start Screen, 1st B is the High byte of data length,
    2nd B is the Low byte of data length, following by data.
    For mono display, the start screen is bitmap data;
    For color display, the start screen is commands set,
    the first and second bytes are the commands length.
    */
    if( data_len > 2048 )
    {
        if(NULL!=err) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* Digole specs zero command for i2c only */
    digole_i2c_wakeup(drv_handle);

    /* NOTE: starting from v.3.3 the syntax is different */
    /* Command: SSS, follow by 2 bytes of data length of start screen, 
       then the data, as described before, the data structure are different 
       for monochrome module and color module. In V3.2 and earlier version 
       on color module, the command set also need 2 bytes of data to indicate 
       the command set length, when you downloading this format of start screen 
       to module, 2 bytes of length follow to SSS to indicate the length of 
       rest data, and in the rest of data, the first 2 bytes to indicate the 
       length of command set, their relationship is:
       SSS (length+2) (length) (…data…).
    */
    if( (10*drv_handle->cfg.hw_mjr + drv_handle->cfg.hw_min) < 32 /*3.2*/ )
    {
        /* LSB...MSB */

        // (length+2)
        digole_cmd[payload.length] = ((data_len+2) >> 0) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = ((data_len+2) >> 8) & 0xFF;
        payload.length++;

        // (length)
        digole_cmd[payload.length] = ((data_len) >> 0) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = ((data_len) >> 8) & 0xFF;
        payload.length++;

        /* add data: bit for monocrome or command set for color displays */
        memcpy(&digole_cmd[payload.length], data, data_len);
        payload.length+=data_len;
    }
    else
    {
        /* MSB...LSB */

        // (length)
        digole_cmd[payload.length] = ((data_len) >> 8) & 0xFF;
        payload.length++;
        digole_cmd[payload.length] = ((data_len) >> 0) & 0xFF;
        payload.length++;

        /* add data: bit for monocrome or command set for color displays */
        memcpy(&digole_cmd[payload.length], data, data_len);
        payload.length+=data_len;
        
        /* for v.3.3 and above Digole requires one more termination */
        digole_cmd[payload.length] = 255;
        payload.length++;
    }

    payload.tx_buffer = digole_cmd;
    payload.rx_buffer = NULL;

    /* loop over write chunks with delays */
    /* NOTE: from the documentation and from many tests it seems that we cannot
    write the font in burst (expecially on spi bus) but we have to do it byte by byte
    and we have to keep 8/10ms for each write since the onboard flash is VERY SLOW.
    Also add a delay every 64 byte write and one at the very end to make sure the
    comman input buffer has been drained.
    */

#define SPLASH_CHUNK_SIZE (1)
    io_write_idx  = 0;
    uint8_t tmp=0;
    uint32_t tmp32=payload.length;

    uint32_t msg_flag = (UG_BUS_MSG_COALESCE_START);
    while(io_write_idx < (tmp32))
    {
        payload.tx_buffer = &digole_cmd[io_write_idx];
        payload.length    = ((io_write_idx+SPLASH_CHUNK_SIZE)<(tmp32)) ? SPLASH_CHUNK_SIZE  : ((tmp32)-io_write_idx);
        payload.rx_buffer = NULL;

        io_write_idx += payload.length;

        if(io_write_idx >= (tmp32))
        {
            msg_flag |= UG_BUS_MSG_COALESCE_STOP;
        }

        UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, msg_flag);
        if(UG_BUS_NO_ERR==bus_err)
        {
            drv_handle->w_byte += payload.length;
            wiced_rtos_delay_milliseconds(10);
        }
        else
        {
            if(NULL!=err) *err = UG_DRV_ERR_IO;
            return;
        }

        msg_flag = UG_BUS_MSG_COALESCE_CHUNK;

        if(++tmp==64)
        {
            wiced_rtos_delay_milliseconds(100);
            tmp=0;
        }
    }

    /* after font upload the pic needs some more time */
    wiced_rtos_delay_milliseconds(400);
}

void UG_driver_digole_scr_enable_splash(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;
    
    uint8_t digole_cmd[4]= {'D','S','S', (flag&0x01) };
    payload.tx_buffer = digole_cmd;
    payload.length = 4;
    payload.rx_buffer = NULL;


    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP));
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        digole_delay(drv_handle);
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}

/* ******************* */
/* TOUCH PANEL */
/* ******************* */

/* ******************* */
/* FLASH */
/* ******************* */

/* ******************* */
/* EEPROM */
/* ******************* */
