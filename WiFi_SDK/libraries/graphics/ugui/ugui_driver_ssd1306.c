/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library HW Function
 *
 */

#include "ugui_driver.h"
#include "ugui_driver_private.h"
#include "ugui_bus.h"

/*
 * NOTE: Solomon Systech SSD1306 has no hw accelerated primitives,
 * but it does have hw support for scrolling.
 * for uGUI only pset is needed to support ug_gfx primitives,
 * but other driver calls might become useful (need to implement)
 */

static const UG_DRV_FN ssd1306_fn = {
    /* gfx_pixel_set          */ &UG_driver_ssd1306_set_pixel,  
    /* raw_fn                 */ NULL, 
    /* raw_set_defaults       */ NULL,  
    /* sys_clear_screen       */ &UG_driver_ssd1306_sys_clear_screen,  
    /* sys_set_backcolor      */ NULL, 
    /* sys_set_forecolor      */ NULL,  
    /* sys_set_draw_mode      */ NULL,  
    /* sys_set_draw_rot       */ NULL, 
    /* sys_set_window         */ NULL,  
    /* sys_clear_window       */ NULL,  
    /* sys_set_fullscreen     */ NULL, 
    /* sys_set_screen_refresh */ NULL,  
    /* sys_set_screen_norminv */ NULL,  
    /* sys_set_screen_update  */ &UG_driver_ssd1306_sys_screen_update,
    /* sys_screen_scroll_start*/ &UG_driver_ssd1306_sys_screen_scroll_start,
    /* sys_screen_scroll_stop */ &UG_driver_ssd1306_sys_screen_scroll_stop,
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
    /* scr_set_contrast       */ &UG_driver_ssd1306_scr_set_contrast,  
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

/* ******************************************************************************* */
/* data sheet defines */
/* ******************************************************************************* */
#define SSD1306_SETCONTRAST                             (0x81)
#define SSD1306_DISPLAYALLON_RESUME                     (0xA4)
#define SSD1306_DISPLAYALLON                            (0xA5)
#define SSD1306_NORMALDISPLAY                           (0xA6)
#define SSD1306_INVERTDISPLAY                           (0xA7)
#define SSD1306_DISPLAYOFF                              (0xAE)
#define SSD1306_DISPLAYON                               (0xAF)
#define SSD1306_SETDISPLAYOFFSET                        (0xD3)
#define SSD1306_SETCOMPINS                              (0xDA)
#define SSD1306_SETVCOMDETECT                           (0xDB)
#define SSD1306_SETDISPLAYCLOCKDIV                      (0xD5)
#define SSD1306_SETPRECHARGE                            (0xD9)
#define SSD1306_SETMULTIPLEX                            (0xA8)
#define SSD1306_SETLOWCOLUMN                            (0x00)
#define SSD1306_SETHIGHCOLUMN                           (0x10)
#define SSD1306_SETSTARTLINE                            (0x40)
#define SSD1306_MEMORYMODE                              (0x20)
#define SSD1306_COLUMNADDR                              (0x21)
#define SSD1306_COMSCANINC                              (0xC0)
#define SSD1306_COMSCANDEC                              (0xC8)
#define SSD1306_SEGREMAP                                (0xA0)
#define SSD1306_CHARGEPUMP                              (0x8D)
#define SSD1306_EXTERNALVCC                              (0x1)
#define SSD1306_SWITCHCAPVCC                             (0x2)
#define SSD1306_ACTIVATE_SCROLL                         (0x2F)
#define SSD1306_DEACTIVATE_SCROLL                       (0x2E)
#define SSD1306_SET_VERTICAL_SCROLL_AREA                (0xA3)
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                 (0x26)
#define SSD1306_LEFT_HORIZONTAL_SCROLL                  (0x27)
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    (0x29)
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL     (0x2A)

/* SSD1306 specific flags for i2c */
#define SSD1306_CONTROL_DATA                            (1<<6)
#define SSD1306_CONTROL_CMD                             (1<<7)
#define SSD1306_CONTROL_NONE                               (0)

/* development only */
//#define SSD1306_USE_HW_SCROLL

/* ******************************************************************************* */
/* data sheet I2C command preamble */
/* ******************************************************************************* */
#define SSD1306_CMD_PREAMBLE(cmd, i, flag)                      \
do                                                              \
{                                                               \
    if( (drv_handle->cfg.hw_bus == (UG_DRV_CFG_DEV_BUS_I2C)) && \
        (flag!=0) )                                             \
    {                                                           \
            cmd[i++] = (flag);                                  \
    }                                                           \
}while(0)

/* ******************************************************************************* */
/* internals */
/* ******************************************************************************* */
static inline void ssd1306_delay(UG_DRV_HANDLE drv_handle)
{
    wiced_rtos_delay_milliseconds( 1 );
}

static inline void ssd1306_sw_scroll_xy( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                         uint8_t direction, 
                                         uint8_t speed_x, uint8_t speed_y, 
                                         uint16_t scroll_x, uint16_t scroll_y,
                                         uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                         uint8_t circular_f)
{
    int16_t xb = x1;
    int16_t yb = y1;

    uint16_t x_sleep_usec = ((256 - speed_x)*500);
    uint16_t y_sleep_usec = ((256 - speed_y)*500);

    uint16_t refresh_usec = (x_sleep_usec >  y_sleep_usec) ?  y_sleep_usec :  x_sleep_usec;

    uint32_t w_Q10 = 0; 
    uint32_t h_Q10 = 0;

    int32_t x_step_Q10 = (speed_x >= speed_y) ? (1<<10) : ((speed_x<<10) / speed_y);
    int32_t y_step_Q10 = (speed_y >= speed_x) ? (1<<10) : ((speed_y<<10) / speed_x);

    int8_t x_step_sign = 0;
    int8_t y_step_sign = 0;

    int32_t xb_Q10 = xb<<10;
    int32_t yb_Q10 = yb<<10;


    /* set the begin position and the increment sign for VERTICAL direction */
    if(direction & UG_DRV_SCROLL_UP)
    {
        /* begin */
        yb = y1;
        yb_Q10 = yb<<10; 

        /* increment is positive */
        y_step_sign = 1;
    }
    else if(direction & UG_DRV_SCROLL_DOWN)
    {
        /* begin */
        yb = y2;
        yb_Q10 = yb<<10; 
        
        /* increment is negative */
        y_step_sign = -1;
    }
    else
    {
        y_step_Q10 = 0;
        y_step_sign = 1;
    }

    /* set the begin position and the increment sign for HORIZONTAL direction */
    if(direction & UG_DRV_SCROLL_LEFT)
    {
        /* begin */
        xb = x1;
        xb_Q10 = xb<<10; 

        /* increment is positive */
        x_step_sign = 1;
    }
    else if(direction & UG_DRV_SCROLL_RIGHT)
    {
        /* begin */
        xb = x2;
        xb_Q10 = xb<<10; 
        
        /* increment is negative */
        x_step_sign = -1;
    }
    else
    {
        x_step_Q10 = 0;
        x_step_sign = 1;
    }

    int16_t xs = xb;
    int16_t ys = yb;

    int16_t hp = (y2-y1+1);
    int16_t wp = (x2-x1+1);
    

    while( ((w_Q10>>10)<(x2-x1+1)) && ((h_Q10>>10)<(y2-y1+1)) )
    {
        if( (yb != (yb_Q10>>10)) || (xb != (xb_Q10>>10)) )
        {
            int16_t xp = xb;
            int16_t yp = yb;

            /*
             * LOOP: COPY PIXELS
             */
            int16_t h = (y2-y1+1) - (h_Q10>>10);
            int16_t w = (x2-x1+1) - (w_Q10>>10);

            yb = (yb_Q10>>10);
            xb = (xb_Q10>>10);

            int yi = 0;
            int xj = 0;
            for(yi=0; yi<h; yi++)
            {
                for(xj=0; xj<w; xj++)
                {
                    // dest current
                    int16_t xdc = xs + xj * x_step_sign;
                    int16_t ydc = ys + yi * y_step_sign;

                    // source current
                    int16_t xsc = xb + xj * x_step_sign;
                    int16_t ysc = yb + yi * y_step_sign;
                    uint8_t sc = ( 0 == ( (drv_handle->screen_buf[ ((ysc>>3)<<7) + xsc ]) & (1<<(ysc%8)) ) ) ? 0 : 1;

                    // clear bit
                    drv_handle->screen_buf[ ((ydc>>3)<<7) + xdc ] &= ~( ((uint8_t)1) << (ydc%8) );
                    // set bit
                    drv_handle->screen_buf[ ((ydc>>3)<<7) + xdc ] |= sc << (ydc%8);                    
                }
            }

            /*
             *  LOOP: CLEAR PIXELS
             */
#if 1
            int16_t wc = wp - w;
            int16_t hc = hp - h;

            yi = 0;
            xj = 0;
            for(yi=0; yi<h; yi++)
            {
                for(xj=0; xj<wc; xj++)
                {
                    // destination current
                    int16_t xdc = xp + (w + xj) * x_step_sign;
                    int16_t ydc = yp + yi * y_step_sign;
                    // clear bit
                    drv_handle->screen_buf[ ((ydc>>3)<<7) + xdc ] &= ~( ((uint8_t)1) << (ydc%8) );
                }
            }

            yi = 0;
            xj = 0;
            for(yi=0; yi<hc; yi++)
            {
                for(xj=0; xj<w; xj++)
                {
                    // destination current
                    int16_t xdc = xp + xj * x_step_sign;
                    int16_t ydc = yp + (h + yi) * y_step_sign;
                    // clear bit
                    drv_handle->screen_buf[ ((ydc>>3)<<7) + xdc ] &= ~( ((uint8_t)1) << (ydc%8) );
                }
            }

            yi = 0;
            xj = 0;
            for(yi=0; yi<hc; yi++)
            {
                for(xj=0; xj<wc; xj++)
                {
                    // destination current
                    int16_t xdc = xp + (w + xj) * x_step_sign;
                    int16_t ydc = yp + (h + yi) * y_step_sign;
                    // clear bit
                    drv_handle->screen_buf[ ((ydc>>3)<<7) + xdc ] &= ~( ((uint8_t)1) << (ydc%8) );
                }
            }

            /* track for clearing pixels */
            hp = h;
            wp = w;
#endif

            /*
             * reset position for next refresh
             */
            xb_Q10 = (xs<<10);
            yb_Q10 = (ys<<10);
            xb = xs;
            yb = ys;

            /*
             * REFRESH
             */
            UG_driver_ssd1306_sys_screen_update(err, drv_handle, 0 /* don't care */);
        }

        /*
         * increment indexes
         */
        w_Q10 += x_step_Q10;
        h_Q10 += y_step_Q10;
        xb_Q10 += x_step_Q10 * x_step_sign ;
        yb_Q10 += y_step_Q10 * y_step_sign ;

        // refresh pause
        wiced_rtos_delay_microseconds(refresh_usec);
    }
}
    
/* ******************************************************************************* */
/* SSD1306 specific methods */
/* ******************************************************************************* */
#define GCC_UNUSED __attribute__( ( __unused__ ) )

static void ssd1306_msg_add( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *msg, 
                             uint16_t *msg_idx, uint8_t cd_code, uint8_t *data, uint16_t data_len)
{
    uint16_t i = *msg_idx;
    
    SSD1306_CMD_PREAMBLE(msg, i, cd_code);

    if(data!=NULL)
    {
        memcpy(msg, data, data_len);
        i+=data_len;
    }
    else
    {
        *(msg + i) = data_len; //used as a single byte case
        i++;
    }
    
    *msg_idx = i;
}

static void ssd1306_msg_send( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t *msg, 
                              uint16_t msg_len,  uint32_t coalesce_flag)
{
    UG_BUS_PAYLOAD_T payload;
    UG_BUS_PAYLOAD_T *payloads[] = {&payload};
    UG_BUS_ERR bus_err = UG_BUS_NO_ERR;

    /* prepare each message and send it to the bus */
    payload.tx_buffer = msg;
    payload.length    = msg_len;
    payload.rx_buffer = NULL;

    UG_bus_msg_fn(&bus_err, UG_BUS_MSG_WRITE_SEQ, (UG_BUS_PAYLOAD_T**)&payloads, 1, coalesce_flag );
    if(UG_BUS_NO_ERR==bus_err)
    {
        drv_handle->w_byte += payload.length;
        ssd1306_delay(drv_handle);

        if(NULL!=err) *err = UG_DRV_NO_ERR;
    }
    else
    {
        if(NULL!=err) *err = UG_DRV_ERR_IO;
    }
}


static GCC_UNUSED void ssd1306_off(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* send (0xAE) */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DISPLAYOFF);

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_on(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* send (0xAE) */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DISPLAYON);

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_set_contrast(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t val)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* send (0xAE) */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETCONTRAST);
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, val );

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_scroll_x( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                             uint8_t direction, uint8_t begin, uint8_t end, uint16_t x_offset)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* safety: make sure there is no other scrolling running */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DEACTIVATE_SCROLL );

    /* new scroll setup */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 
                     (direction ? SSD1306_LEFT_HORIZONTAL_SCROLL : SSD1306_RIGHT_HORIZONTAL_SCROLL ));

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00);
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (begin & 0x07) );

    switch (x_offset) {
        case   2: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x07); break; // 111b
        case   3: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x04); break; // 100b
        case   4: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x05); break; // 101b
        case   5: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00); break; // 000b
        case  25: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x06); break; // 110b
        case  64: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x01); break; // 001b
        case 128: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x02); break; // 010b
        case 256: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x03); break; // 011b
        default:
            // default to 2 frame interval
            ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x07);break;
    }

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (end & 0x07) );
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00 );
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0xFF );
    
    // activate scroll
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_ACTIVATE_SCROLL );

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_scroll_set_y_area( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                                  uint8_t y1, uint8_t y2)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* safety: make sure there is no other scrolling running */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DEACTIVATE_SCROLL );

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SET_VERTICAL_SCROLL_AREA );
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, y1 );
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, y2 );

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_scroll_xy( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle,
                                          uint8_t direction, uint8_t begin, uint8_t end, 
                                          uint16_t x_offset, uint8_t v_offset)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* safety: make sure there is no other scrolling running */
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DEACTIVATE_SCROLL );

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 
                     (direction ? SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL : SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL  ));

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00);
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (begin & 0x07) );

    switch (x_offset) {
        case   2: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x07); break; // 111b
        case   3: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x04); break; // 100b
        case   4: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x05); break; // 101b
        case   5: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00); break; // 000b
        case  25: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x06); break; // 110b
        case  64: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x01); break; // 001b
        case 128: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x02); break; // 010b
        case 256: ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x03); break; // 011b
        default:
            // default to 2 frame interval
            ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x07);break;
    }

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (end & 0x07) );
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, v_offset );
    
    // activate scroll
    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_ACTIVATE_SCROLL );

    /* shoot all the commands in one bus transaction */
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_scroll_stop( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DEACTIVATE_SCROLL );
    ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );
}

static GCC_UNUSED void ssd1306_nop(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    uint8_t cmd[8] = {0};
    uint16_t i = 0;

    ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0xE3 );
}

static GCC_UNUSED void ssd1306_reset(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    /* on spi send reset on cs if needed */
}

/* ******************************************************************************* */
/* UGUI driver calls */
/* ******************************************************************************* */
void UG_driver_ssd1306_init(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;

    uint8_t cmd[256] = {0};
    uint16_t i = 0;

    /* SSD1306 allows only 128x32 and 128x64 resolutions */
    if( (64 != drv_handle->cfg.y_max) &&
        (32 != drv_handle->cfg.y_max) )
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }
    if(128 != drv_handle->cfg.x_max)
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }

    /* sanity check */
    if(UG_DRV_WMARK != drv_handle->wmark)
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }
    else
    {
        uint16_t w = drv_handle->cfg.x_max;
        uint16_t h = drv_handle->cfg.y_max;
        if ( NULL == ( drv_handle->screen_buf = (uint8_t*) calloc( (w*h/8),  sizeof( uint8_t ) ) ) )
        {
            err_tmp = UG_DRV_ERR_OUT_OF_MEMORY;
        }
    }

    /*
     * if the screen buffer was allocated proceede with init.
     */
    if( err_tmp == UG_DRV_NO_ERR )
    {
        /* set the function table */
        memcpy(&drv_handle->fn, &ssd1306_fn, sizeof(ssd1306_fn));

        /* if the display needs a reset pin this should be passed as cfg param rst_gpio */
        if( drv_handle->cfg.reset_gpio > 0)
        {
            /* TBD: handle reset here, if needed */
            //GPIO_WriteBit(GPIOB, _RES1, Bit_RESET);
            //delay_ms(100);
            //GPIO_WriteBit(GPIOB, _RES1, Bit_SET);
            //delay_ms(100);
        }

        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DISPLAYOFF);

        // Set Clock as 100 Frames/Sec
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETDISPLAYCLOCKDIV);
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x80);

        // Set Multiplex Ratio
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETMULTIPLEX);
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, ((drv_handle->cfg.x_max==32)?(0x1F):(0x3F)) );

        // Set Display Offset
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETDISPLAYOFFSET);
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x00);

        // Set Display Start Line
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (SSD1306_SETSTARTLINE | 0x00) );

        // Set Charge Pump (enabled)
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_CHARGEPUMP );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x14 );

        // Set Memory Addressing Mode : PAGE MODE
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_MEMORYMODE );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x02 );        

        // Set Segment Re-Map
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (SSD1306_SEGREMAP | 0x1) );

        // Set COM Output Scan Direction : DECREMENT
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_COMSCANDEC );

        // Set COM Pins Hardware Configuration
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETCOMPINS );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x12 );

        // Set Contrast Control
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETCONTRAST );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x7F );

        // Set Pre-Charge Period (Ext VCC => 0x22,  Int VCC => 0xF1)
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETPRECHARGE );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x22 );

        // Set VCOMH Deselect Level, Default => 0x20 (0.77*VCC)
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_SETVCOMDETECT );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, 0x30 );

        // Set Entire Display On/Off
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DISPLAYALLON_RESUME );

        // Set NORMAL/inverse display
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_NORMALDISPLAY );

        // Turn ON display
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, SSD1306_DISPLAYON );
 
        /* shoot all the commands in one bus transaction */
        ssd1306_msg_send( &err_tmp, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START|UG_BUS_MSG_COALESCE_STOP) );

        if( err_tmp == UG_DRV_NO_ERR )
        {
            /* make sure there is no spurious scrolling from a previous app/session */
            ssd1306_scroll_stop( &err_tmp, drv_handle);
        }
    }
    
    if(NULL!=(err)) *err = err_tmp;
}


void UG_driver_ssd1306_deinit(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;

    if(drv_handle != NULL)
    {
        if(drv_handle->screen_buf != NULL)
        {
            free(drv_handle->screen_buf);
            drv_handle->screen_buf = NULL;
        }
    }
    else
    {
        err_tmp = UG_DRV_ERR_INVALID_PARAM;
    }

    if(NULL!=(err)) *err = err_tmp;

}


void UG_driver_ssd1306_set_pixel(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, int16_t x, int16_t y, uint32_t c)
{
   UG_DRV_ERR err_tmp = UG_DRV_NO_ERR;

   uint16_t w = drv_handle->cfg.x_max;

   unsigned int p;

   if ( x > w )
   {
       err_tmp = UG_DRV_ERR_INVALID_PARAM;
   }
   else
   {
       p = y>>3; /* :8   */
       p = p<<7; /* 128  */
       p +=x;

       if( c != 0 /* normal mode: pixel OFF */ )
       {
           drv_handle->screen_buf[p] |= 1<<(y%8);
       }
       else /* normal mode: pixel ON */
       {
           drv_handle->screen_buf[p] &= ~(1<<(y%8));
       }
   }

    if(NULL!=(err)) *err = err_tmp;
}


/* ******************* */
/* SYSTEM */
/* ******************* */

void UG_driver_ssd1306_sys_clear_screen(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
    uint16_t w = drv_handle->cfg.x_max;
    uint16_t h = drv_handle->cfg.y_max;

    memset( drv_handle->screen_buf, 0x00, ( (w*h/8) * sizeof( uint8_t ) ) );

    UG_driver_ssd1306_sys_screen_update(err, drv_handle, 0 /* don't care */);
}

void UG_driver_ssd1306_sys_screen_update(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t flag)
{
    uint8_t cmd[256];
    uint8_t  p;
    uint16_t i;

    uint16_t h = drv_handle->cfg.y_max;

    
    /* shoot all the data, one page at a time */
    for(p=0; p<(h/8); p++)
    {
        /* COMMAND */
        i=0;

        // Set the page address
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL, (0xB0|p) );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL,      (0) );
        ssd1306_msg_add( NULL, drv_handle, cmd, &i, SSD1306_CONTROL_CMD, NULL,   (0x10) );

        /* Trick : add the DATA CONTROL BYTE to the queue of the initial message  */
        if(drv_handle->cfg.hw_bus == (UG_DRV_CFG_DEV_BUS_I2C))
        {
            SSD1306_CMD_PREAMBLE(cmd, i, SSD1306_CONTROL_DATA);
        }

        /* send the 1ST part, START coalescing transaction */
        ssd1306_msg_send(err, drv_handle, cmd, i,  (UG_BUS_MSG_COALESCE_START) );

        /* send the 2ND part, one full page, 128 bytes, STOP coalescing transaction */
        ssd1306_msg_send(err, drv_handle, &(drv_handle->screen_buf[(p<<7)]), 128,  (UG_BUS_MSG_COALESCE_STOP) );
    }
}

void UG_driver_ssd1306_sys_screen_scroll_start(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, 
                                               uint8_t direction, 
                                               uint8_t speed_x, uint8_t speed_y, 
                                               uint16_t scroll_x, uint16_t scroll_y,
                                               uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                               uint8_t circular_f,
                                               void* usr_data)
{
    if( (x1>=drv_handle->cfg.x_max) || (x2>=drv_handle->cfg.x_max) || 
        (y1>=drv_handle->cfg.y_max) || (y2>=drv_handle->cfg.y_max) )
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    if(!(direction & (UG_DRV_SCROLL_UP | UG_DRV_SCROLL_DOWN | UG_DRV_SCROLL_LEFT | UG_DRV_SCROLL_RIGHT)))
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

#ifdef  SSD1306_USE_HW_SCROLL
    /* NOTE: 
     * the hw scrolling function of SSDD1306 is USELESS unless you want to do a scrolling logo
     * for a splash screen. It cannot be controlled beside page scrolling and being ony a rolling buffer
     * it does basically only a demo gfx effect, but not much else. The code to use it is left here
     * for reference but an alternative code is provided to perform SW screen scrolling as per API 
     */

    /* SSD1306 does not support scroll down */
    if(direction & UG_DRV_SCROLL_DOWN)
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* Note: scroll for SSD1306 is defined in PAGES, where each page is a stripe of 128x8 pixels */
    /* first : check if we are doing just an horizontal scroll first */
    if( !(direction & (UG_DRV_SCROLL_UP | UG_DRV_SCROLL_DOWN) ) )
    {
        ssd1306_scroll_x( err, drv_handle,
                          ((direction & UG_DRV_SCROLL_LEFT) ? 1 : 0),
                          y1/8,
                          y2/8,
                          256 - (speed_x%256) );
    }
    else
    {
        ssd1306_scroll_set_y_area( err, drv_handle, y1, y2);

        /* second : call the horiz/vert scroll */
        ssd1306_scroll_xy( err, drv_handle,
                           ((direction & UG_DRV_SCROLL_LEFT) ? 1 : 0),
                           y1/8,
                           y2/8,
                           256 - (speed_x%256),
                           speed_y);
    }
#else
    /* sanity check (x1,y1) must be the upper left corner! */
    if( (x1>x2) || (y1>y2) )
    {
        if(NULL!=(err)) *err = UG_DRV_ERR_INVALID_PARAM;
        return;
    }

    /* screen scrolling: SW implementation (will use CPU, not hw accelerated */
    ssd1306_sw_scroll_xy( err, drv_handle,
                          direction, 
                          (speed_x+1), (speed_y+1), 
                          scroll_x, scroll_y,
                          x1, y1, x2, y2, circular_f );
#endif
}

void UG_driver_ssd1306_sys_screen_scroll_stop(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle)
{
#ifdef  SSD1306_USE_HW_SCROLL
    ssd1306_scroll_stop( err, drv_handle);
#endif
}

/* ******************* */
/* FONT */
/* ******************* */
/* none supported */

/* ******************* */
/* TEXT */
/* ******************* */
/* none supported */

/* ******************* */
/* GRAPHICS */
/* ******************* */
/* none supported */

/* ******************* */
/* COMMUNICATION */
/* ******************* */
/* none supported */

/* ******************* */
/* POWER */
/* ******************* */
/* none supported */

/* ******************* */
/* SCREEN */
/* ******************* */
void UG_driver_ssd1306_scr_set_contrast(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t lvl)
{
    ssd1306_set_contrast(err, drv_handle, lvl);
}

/* ******************* */
/* TOUCH PANEL */
/* ******************* */
/* none supported */

/* ******************* */
/* FLASH */
/* ******************* */
/* none supported */

/* ******************* */
/* EEPROM */
/* ******************* */
/* none supported */
