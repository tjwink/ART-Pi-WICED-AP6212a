/*
 * $ Copyright Broadcom Corporation $
 */

/** @file UGUI Library Header
 *
 */

#ifndef _UGUI_DRIVER_H
#define _UGUI_DRIVER_H

#include "wiced.h"

/*
 * maximum number of fonts in the font LUT.
 * must be the same as defined from uGui or bigger.
 */
#define UG_DRV_MAX_HW_FONT_NUM (16)


/**
 * @brief UG_DRV_ERR: Error types returned by driver direct calls
 */
typedef enum{
    UG_DRV_NO_ERR = 0,
    UG_DRV_ERR_UNKNOWN,
    UG_DRV_ERR_BUSY,
    UG_DRV_ERR_INVALID_PARAM,
    UG_DRV_ERR_OUT_OF_MEMORY,
    UG_DRV_ERR_IO,
    UG_DRV_ERR_NOT_SUPPORTED,
    /**/
    UGUI_DRV_ERR_MAX
}UG_DRV_ERR;

/**
 * @brief UG_DRV_CFG_PROTOCOL: supported api protocol for hw displays. 
 * List of supported hw protocol displays
 * each protocol must have a separate 
 * file implementation exposing the API
 * listed below as a struct of fn pointers.
 * If a fn is not supported ptr will be NULL.
 */
typedef enum{
    /* virtual display for debug */
    UG_DRV_CFG_DEV_VIRTUAL = 0,
    /* real displays */
    UG_DRV_CFG_DEV_DIGOLE,
    UG_DRV_CFG_DEV_SSD1306,
    /**/
    UG_DRV_CFG_DEV_MAX
}UG_DRV_CFG_PROTOCOL;

/**
 * @brief UG_DRV_CFG_BUS: 
 * sometimes the internal protocol needs specific
 * instructions depending on the hw bus_type in use
 * so it should be specified in the cfg section of the display
 */
typedef enum{
    UG_DRV_CFG_DEV_BUS_NONE = 0,
    /* real displays */
    UG_DRV_CFG_DEV_BUS_I2C,
    UG_DRV_CFG_DEV_BUS_SPI,
    UG_DRV_CFG_DEV_BUS_UART,
    /**/
    UG_DRV_CFG_DEV_BUS_MAX
}UG_DRV_CFG_BUS;

/** 
 * @brief UG_DRV_CFG_FMT_COLOR: color format supported by hw displays.
 * Pixels color format to accomodate for
 * different types of displays
 */
typedef enum{
    /* mono means 1pixel == 1bit b/w */
    UG_DRV_FMT_MONO   = 0,
    /* rgb */
    UG_DRV_FMT_RGB332,
    UG_DRV_FMT_RGB565,
    UG_DRV_FMT_RGB666,
    UG_DRV_FMT_RGB888,
    /* rgb+alpha */
    UG_DRV_FMT_ARGB332,
    UG_DRV_FMT_ARGB565,
    UG_DRV_FMT_ARGB666,
    UG_DRV_FMT_ARGB888,
    /**/
    UG_DRV_FMT_MAX
}UG_DRV_CFG_FMT_COLOR;

/**
 * @brief status of the driver after initialization
 */
typedef enum {
    UG_DRV_STATUS_UNKNOWN = 0,
    UG_DRV_STATUS_READY,
    /**/
    UG_DRV_STATUS_MAX
}UG_DRV_STATUS;

/**
 * @brief specific values for display scroll control
 */
typedef enum {
    UG_DRV_SCROLL_UP    = (1<<0),
    UG_DRV_SCROLL_DOWN  = (1<<1),
    UG_DRV_SCROLL_LEFT  = (1<<2),
    UG_DRV_SCROLL_RIGHT = (1<<3)
}UG_DRV_SCROLL_DIRECTION;
    

/*
 * @brief Driver configuration parameters. 
 * driver config for initialization.
 * list of all the properties related 
 * to hw identification and versioning.
 */
typedef struct ug_drv_config_
{
    /* display screen size in pixels */
    uint16_t  x_max;
    uint16_t  y_max;
    UG_DRV_CFG_FMT_COLOR fmt_color; 
    UG_DRV_CFG_PROTOCOL  hw_proto;
    UG_DRV_CFG_BUS       hw_bus;
    uint8_t   hw_addr;
    uint8_t   hw_mjr;
    uint8_t   hw_min;
    char      hw_model_pn[64];
    /* flash and eeprom presence flags */
    uint8_t   flash_enabled;
    uint8_t   eeprom_enabled;
    uint8_t   touch_enabled;
    /* 
     * more config params if needed 
     */
    uint32_t  hw_usr1;
    void     *hw_usr2;
    /* more optional paramters */
    uint8_t   reset_gpio;
    uint8_t   touch_gpio;
} UG_DRV_CONFIG;

/*
 * we need to keep a mapping between the UGUI font ptrs
 * and local hw default and hw user fonts on the display
 */
typedef struct ug_drv_fnt_lut_
{
    void    *ug_font_ptr;
    uint8_t hw_id;
} UG_DRV_FONT_LUT;


/*
 *  set of graphics primitives which is a sub-set of all the 
 *  function exposed by the device driver.
 *  these primitives are used by uGui and must be common to all
 *  the supported displays.
 *  eventually, if a display does not support a specific primitive,
 *  the fn pointer will be null, UGUI will fall back to pixel draw.
 */
struct ug_driver_t_;

/* 
 * list of hw accelerated functions.
 * do not reorder this struct and make sure that the list is 
 * kept consistent across ALL the hw drivers when a new fn is added.
 */
typedef struct ug_drv_fn_t
{
    void (*gfx_pixel_set)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t x, int16_t y, uint32_t argb);
    void (*raw_fn)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t msg, void *data, uint32_t len);
    void (*raw_set_defaults)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*sys_clear_screen)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*sys_set_backcolor)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint32_t c);
    void (*sys_set_forecolor)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint32_t c);
    void (*sys_set_draw_mode)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t mode);
    void (*sys_set_draw_rot)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t deg);
    void (*sys_set_window)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t x, int16_t y, uint16_t w, uint16_t h);
    void (*sys_clear_window)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*sys_set_fullscreen)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*sys_set_screen_refresh)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t mode);
    void (*sys_set_screen_norminv)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*sys_screen_update)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*sys_screen_scroll_start)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t direction, 
                                    uint8_t speed_x, uint8_t speed_y,
                                    uint16_t scroll_x, uint16_t scroll_y,
                                    uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                    uint8_t circular_f, void* usr_data);
    void (*sys_screen_scroll_stop)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*txt_set_pos_rowcol)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t cx, int16_t ry);
    void (*txt_set_pos_pixel)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t x1, int16_t y1);
    void (*txt_set_pos_back)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*txt_write)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, char* text, uint32_t text_len);
    void (*txt_write_rowcol)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, char* text, uint32_t text_len, int16_t cx, int16_t ry);
    void (*txt_write_xy)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, char* text, uint32_t text_len, int16_t x, int16_t y);
    void (*txt_cr)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle);
    void (*txt_show_cursor)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t mode, uint8_t onoff_f);
    void (*fnt_upload)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t hw_id, void *ug_font, uint8_t *data, uint32_t data_len);
    void (*fnt_usrlut)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, UG_DRV_FONT_LUT lut[], uint8_t lut_len);
    void (*fnt_search)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t *hw_id, void *ug_font);
    void (*fnt_select)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, void *ug_font);
    void (*fnt_set)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t hw_id);
    void (*gfx_set_pos)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t x, int16_t y);
    void (*gfx_draw_lineto)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t x2, int16_t y2, uint32_t argb);
    void (*gfx_set_linepattern)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t pattern);
    void (*gfx_draw_line)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                          int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
    void (*gfx_draw_rect)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                          int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
    void (*gfx_fill_rect)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                          int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);
    void (*gfx_draw_circle)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);
    void (*gfx_fill_circle)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);
    void (*gfx_draw_arc)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                         int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb);
    void (*gfx_fill_arc)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                         int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb);
    void (*gfx_draw_image)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                           int16_t x, int16_t y, UG_DRV_CFG_FMT_COLOR img_type, uint8_t *pixels, uint16_t w, uint16_t h, void* usr_fmt); 
    void (*gfx_move)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                     int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t off_x, int16_t off_y);
    void (*gfx_draw_poly)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb); 
    void (*gfx_fill_poly)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb);
    void (*gfx_draw_polyreg)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                             int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb); 
    void (*gfx_fill_polyreg)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, 
                             int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb); 
    void (*com_enable_config)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*pwr_set_brightness)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t lvl);
    void (*pwr_set_onoff)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*pwr_set_sleep)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*pwr_set_deepsleep)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*scr_set_contrast)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t lvl);
    void (*scr_set_splash)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t *data, uint32_t data_len);
    void (*scr_enable_splash)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t flag);
    void (*tch_calibrate)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t value);
    void (*tch_wait)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t *x, int16_t *y, uint8_t press_release_f);
    void (*tch_check)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, int16_t *x, int16_t *y);
    void (*flash_read)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t* addr, uint8_t *len , uint8_t *data);
    void (*flash_write)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t* addr, uint8_t *len , uint8_t *data);
    void (*flash_erase)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t* addr, uint8_t *len);
    void (*eeprom_read)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t* addr, uint8_t *len , uint8_t *data);
    void (*eeprom_write)(UG_DRV_ERR *err, struct ug_driver_t_ *drv_handle, uint8_t* addr, uint8_t *len , uint8_t *data);
} UG_DRV_FN;

/*
 * UG_DRV_T
 * driver core obj
 */
typedef struct ug_driver_t_
{
    /* COMPONENT_TAG() */
    uint32_t           id;
    uint32_t           wmark;
    uint32_t           type;

    /* COMPONENT_SWVER() */
    uint8_t            major;
    uint8_t            minor;
    char               revision[ 32 ];
    char               build[ 5 ];

    /* COMPONENT_DBG() */
    uint8_t            log_lvl;
    uint8_t            log_err;
    char               label[ 64 ];

    /* COMPONENT_ATTRIBUTES() */
    UG_DRV_STATUS      status;     /* internal use ONLY */
    uint32_t           w_byte;     /* internal use ONLY */
    UG_DRV_CONFIG      cfg;
    UG_DRV_FN          fn;
    uint32_t           foreground_color;
    uint32_t           background_color;
    uint32_t           last_color; /* internal use ONLY */
    void*              ug_font;
    /*
     * list of pointers mapping
     * uGUI fonts (which are const *ptr) 
     * to hw_id numbers (as from display internal codes)
     * where hw_id=[0..UG_DRV_MAX_HW_FONT_NUM-1]
     */
    UG_DRV_FONT_LUT    font_lut[UG_DRV_MAX_HW_FONT_NUM];
    /*
     * optional screen buffer, internally allocated
     * not all displays need this.
     */
    uint8_t*           screen_buf;

}UG_DRV_T;

typedef UG_DRV_T* UG_DRV_HANDLE;


/******************************************************
 *               display device common API
 ******************************************************/

/*
 * driver support api as per uGUI spec 
*/


/**
 * @brief initialize driver structure for a specific config
 *
 * This function will take an externally alocated drv_handle
 * and initialize his structure based on the given config parameters
 *
 * @param[out] err the (optional) error return code, returned if not NULL 
 * @param[in,out] drv_handle the pointer to the driver obj
 * @param[in] config the configuration obj for hw display parameters
 */
void UG_driver_init( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, UG_DRV_CONFIG *config);

/**
 * @brief de-initialize driver structure for a specific config
 *
 * This function will take an externally alocated drv_handle
 * and de-initialize his structure.
 *
 * @param[out] err the (optional) error return code, returned if not NULL 
 * @param[in,out] drv_handle the pointer to the driver obj
 */
void UG_driver_deinit( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);

/**
 * @brief select the driver obj to be used as current rendering device
 * 
 * @param[in] err the (optional) error return code, returned if not NULL 
 * @param[in] drv_handle the pointer to the driver obj
 */
void UG_driver_select( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);

/* 
 * hw accell display primitive api list, 
 * return 0 on success, -1 on error/unsupported as per UG_RESULT 
 */

/**
 * @brief paint a pixel on the screen with the given color
 * 
 * @param[in] x the x coordinate
 * @param[in] y the y coordinate
 * @param[in] argb the color of the pixel in UG_DRV_CFG_FMT_COLOR
 *
 * note: pset is a MANDATORY display primitive api as per uGUI spec 
 */
void UG_driver_gfx_pixel_set(int16_t x, int16_t y, uint32_t argb);


/* ************************* */
/* RAW */
/* ************************* */

/**
 * @brief send an implementation-specific message/function to the driver
 * 
 * @param[out] err the (optional) error return code, returned if not NULL 
 * @param[in,out] drv_handle the pointer to the driver obj
 * @param[in] msg the message code
 * @param[in] data the pointer to the raw buffer of data
 * @param[in] len the length in bytes of the raw buffer
 */
void UG_driver_raw_fn( UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle, uint8_t msg, void *data, uint32_t len);

/**
 * @brief auto-set the hw defaults for a specific hw display
 * 
 * @param[in] err the (optional) error return code, returned if not NULL 
 * @param[in,out] drv_handle the pointer to the driver obj
 */
void UG_driver_raw_set_defaults(UG_DRV_ERR *err, UG_DRV_HANDLE drv_handle);


/* ************************* */
/* SYSTEM */
/* ************************* */

/** 
 * @brief Clear Screen
 * 
 * Clear display and present empty framebuffer
 */
int8_t UG_driver_sys_clear_screen(void);

/**
 * @brief set the color for the background color
 *
 * @param c the color to set (hw dependent)
 */
int8_t UG_driver_sys_set_backcolor(uint32_t c); 

/**
 * @brief set the color for the foreground color
 *
 * @param color the color to set (hw dependent)
 */
int8_t UG_driver_sys_set_forecolor(uint32_t c); 

/** 
 * @brief set the Display Drawing Mode
 *
 * this command will change the display mode (overwrite/insert.xor etc)
 * based on the given mode. The value is hw dependent.
 * Returns error if not supported.
 * 
 * @param[in] m mode as described above
 */
int8_t UG_driver_sys_set_draw_mode(uint8_t mode);

/**
 * @brief set the current display orientation.
 *
 * give the requested rotation degree the display
 * will approximate the closest orientation available,
 * usually only the four values 0, 90, 180, 270 are 
 * supported by the majority of displays. 
 *
 * @param[in] deg rotation value in degree
 */
int8_t UG_driver_sys_set_draw_rot(int16_t deg);

/**
 * @brief set drawing window
 * 
 * To facilitate drawing to separate regions of the display, this routine
 * lets you define the active drawing window of the display. All coordinate
 * accesses are bound to this region once it is defined.
 * 
 * @param[in] x the x position of the upper left hand corner
 * @param[in] y the yposition of the upper left hand corner
 * @param[in] w the width of the window
 * @param[in] h the height of the window
 */
int8_t UG_driver_sys_set_window(int16_t x, int16_t y, uint16_t w, uint16_t h);

/**
 * @brief clear drawing window
 * 
 * This routine will clear the defined drawing window.
 */
int8_t UG_driver_sys_clear_window(void);

/**
 * @brief reset the drawing window to full screen
 * 
 * This routine will reset the drawing window to full screen mode.
 */
int8_t UG_driver_sys_set_fullscreen(void);

/**
 * @brief set the mode for screen refresh
 * 
 * This routine will manually enable/disable the screen refresh
 */
int8_t UG_driver_sys_set_screen_refresh(uint8_t mode);

/**
 * @brief NORMAL/INVERSE screen mode
 * 
 * This routine is mainly useful for B/W display,
 * setting the inverse mode for monocrome pixels.
 */
int8_t UG_driver_sys_set_screen_norminv(uint8_t flag);

/**
 * @brief immediate screen refresh
 * 
 * force the update of the display (forced refresh)
 * set the flag to nonzero for immediate refresh
 */
int8_t UG_driver_sys_screen_update(uint8_t flag);

/**
 * @brief display content scroll
 * 
 * hw accelerated display scrolling.
 * if supported this routine will scroll the display content based on the
 * requested direction,
 * scrolling speed on X and Y axis,
 * scroll offset on Y and Y axis,
 * and display scrolling area (x1,y1)->(x2,y2)
 * (x1,y1) is the UPPER  LEFT  corner of the scrolling area
 * (x2,y2) is the BOTTOM RIGHT corner of the scrolling area
 * usr_data: implementation dependent user data ptr
 */
int8_t UG_driver_sys_screen_scroll_start(uint8_t direction, 
                                         uint8_t speed_x, uint8_t speed_y, 
                                         uint16_t scroll_x, uint16_t scroll_y,
                                         uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                         uint8_t circular_f,
                                         void* usr_data);

int8_t UG_driver_sys_screen_scroll_stop();


/* ************************* */
/* TEXT */
/* ************************* */

/** 
 * @brief set the current (row,column) display position
 * 
 * set the current display TEXT position in row and column
 * coordinates, which is different from the (x,y) pixel position.
 *
 * @param[in] cx the x column value
 * @param[in] ry the y row value
 */
int8_t UG_driver_txt_set_pos_rowcol(int16_t cx, int16_t ry);

/** 
 * @brief set the current (x,y) display position
 * 
 * set the current display TEXT position in row and column
 * coordinates, which is different from the (X,Y) pixel position.
 *
 * @param[in] cx the x column value
 * @param[in] ry the y row value
 */
int8_t UG_driver_txt_set_pos_pixel(int16_t x1, int16_t y1);

/** 
 * @brief Set text position back one character (BS)
 * 
 * This virtual backspace allows you to repeatedly update one character
 */
int8_t UG_driver_txt_set_pos_back(void);

/** 
 * @brief print a buffer as text at the current display (text) position
 * 
 *  Print one line of null-terminated text. The current cursor
 *  position is automatically advanced to the next location.
 * 
 * @param[in] text the buffer ptr to be printed
 * @param[in] text_len the number of bytes from the buffer to be printed
 */
int8_t UG_driver_txt_write(char* text, uint32_t text_len);

/** 
 * @brief print a buffer as text at the (row,column) display position
 * 
 *  Print one line of text. The current cursor
 *  position is automatically advanced to the next location.
 *  The number of rows and columns in a display is a function of the 
 *  font size and the display size 
 * 
 * @param[in] text the buffer ptr to be printed
 * @param[in] text_len the number of bytes from the buffer to be printed
 * @param[in] cx the x column value
 * @param[in] ry the y row value
 */
int8_t UG_driver_txt_write_rowcol(char* text, uint32_t text_len,int16_t cx, int16_t ry);

/** 
 * @brief print a buffer as text at the (x,y) pixel display position
 * 
 * @param[in] text the buffer ptr to be printed
 * @param[in] text_len the number of bytes from the buffer to be printed
 * @param[in] x the x pixel value
 * @param[in] y the y pixel value
 */
int8_t UG_driver_txt_write_xy(char* text, uint32_t text_len, int16_t x, int16_t y);

/**
 * @brief send a text command for CR to advance to the next line of text
 * 
 */
int8_t UG_driver_txt_cr(void);

/** 
 * @brief Enable/Disable cursor
 * 
 * EnableDisable the HW cursor
 *
 * @param[in] mode set the cursor type (i.e. square, line, arrow, blinker)
 * @param[in] onoff_f enable/disable hw cursor on the screen
 */
int8_t UG_driver_txt_show_cursor(uint8_t mode, uint8_t onoff_f);


/* ************************* */
/* FONT */
/* ************************* */

/** 
 * @brief upload a custom font to the hw display
 *
 * if the display supports custom fonts (u8g format)
 * this function will load the byte map into a hw slot id
 *
 * @param[in] hw_id hardware id number to store the font (destination)
 * @param[in] ug_font UGUI font (optional) to be internally mapped to the hw_id
 * @param[in] data the font bytestream (usually in u8g format)
 * @param[in] data_len the length of the font data stream above
 */
int8_t UG_driver_fnt_upload(uint8_t hw_id, void *ug_font, uint8_t *data, uint32_t data_len);

/** 
 * @brief set a custom hw/ugui font look up table (LUT)
 *
 * set a custom mapping table between the hw fonts loaded on the display
 * and the uGui supported fonts. 
 *
 * @param[in] lut lookup table pointer
 * @param[in] lut_len number of entries in the lookup table (max is UG_DRV_MAX_HW_FONT_NUM)
 */
int8_t UG_driver_fnt_usrlut(UG_DRV_FONT_LUT lut[], uint8_t lut_len);

/** 
 * @brief given an uGui font search the best match for hw fonts on the display
 *
 * search the font LUT to find he best hw_id that matches the given uGui font.
 *
 * @param[out] hw_id result value for the requested font
 * @param[in] ug_font the uGui font to be matched
 */
int8_t UG_driver_fnt_search(uint8_t *hw_id, void *ug_font);

/** 
 * @brief search and set the hw_id corispondent to the given uGui font
 *
 * search and set: the next text write will be done with the font rquested,
 * if there is a match in the font LUT
 *
 * @param[in] ug_font the uGui font to be matched
 */
int8_t UG_driver_fnt_select(void *ug_font);


/** 
 * @brief set the font with the given hw_id
 *
 *
 * @param[in] hw_id display hardware font to be used
 */
int8_t UG_driver_fnt_set(uint8_t hw_id);


/* ************************* */
/* GRAPHICS */
/* ************************* */

/** 
 * @brief Setting Drawing Position
 * 
 * Set the current drawing position for graphics mode. 
 * 
 * @param[in] x the x position 
 * @param[in] y the y position 
 */
int8_t UG_driver_gfx_set_pos(int16_t x, int16_t y);

/**
 * @brief draw a line from the current position to a new position
 * 
 * This routine will draw a line from the current position (last
 * graphics position x,y) to a new position using the
 * specified color. When the line is done drawing the current graphics
 * position is updated to x2,y2.
 * 
 * @param[in] x2 the ending x coordinate
 * @param[in] y2 the ending y coordinate
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_lineto(int16_t x2, int16_t y2, uint32_t argb);

/**
 * @brief Set the line pattern for line drawing operations
 * 
 * This command allows you to specify a byte which represents a
 * bitmask for drawing operations to affect the line drawing style.
 * When line-drawing operations are performed, the display controller
 * will repeat every 8 bits represented in the line pattern to allow
 * programming a configurable dotted or dashed line style 
 * 
 * @param[in] pattern the byte value representing the bitmap
 */
int8_t UG_driver_gfx_set_linepattern(uint8_t pattern);

/**
 * @brief draw a line
 * 
 * This routine will draw a line from x1,y1 to x2,y2 using the 
 * specified color and graphics position. When the line is done drawing
 * the current graphics position is updated to x2,y2.
 * 
 * @param[in] x1 the starting x coordinate
 * @param[in] y1 the starting y coordinate
 * @param[in] x2 the ending x coordinate
 * @param[in] y3 the ending y coordinate
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_line(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);

/**
 * @brief draw a rectangle (NOT filled)
 * 
 * This routine will draw a rectangle at the specified x,y coordinate
 * with the specified width and height.
 * 
 * @param[in] x the x coordinate
 * @param[in] y the y coordinate
 * @param[in] w the width of the rectangle
 * @param[in] h the height of the rectangle
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);

/**
 * @brief draw a rectangle (filled)
 * 
 * This routine will draw a rectangle at the specified x,y coordinate
 * with the specified width and height and filled with the specified color
 * 
 * @param[in] x the x coordinate
 * @param[in] y the y coordinate
 * @param[in] w the width of the rectangle
 * @param[in] h the height of the rectangle
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_fill_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t argb);

/**
 * @brief draw a circle (NOT filled)
 * 
 * Draws a circle at specified center cx1,cy1 coordinate
 * with specified radius and color
 * 
 * @param[in] cx1 the x coordinate
 * @param[in] cy1 the y coordinate
 * @param[in] r the radius (in pixels)
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_circle(int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);

/**
 * @brief draw a circle (filled)
 * 
 * Draws a circle at specified center cx1,cy1 coordinate
 * with specified radius and filled with the specified color
 * 
 * @param[in] cx1 the x coordinate
 * @param[in] cy1 the y coordinate
 * @param[in] r the radius (in pixels)
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_fill_circle(int16_t cx1, int16_t cy1, uint16_t r, uint32_t argb);

/**
 * @brief draw an arc (sector of a circle)
 * 
 * Draws an arc at specified center cx1,cy1 coordinate
 * with specified radius and angle
 * 
 * @param[in] cx1 the x coordinate
 * @param[in] cy1 the y coordinate
 * @param[in] r the radius (in pixels)
 * @param[in] deg1 start angle in degrees
 * @param[in] deg2 stop  angle in degrees
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_arc(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb);

/**
 * @brief draw a filled arc (sector of a circle)
 * 
 * Draws an arc at specified center cx1,cy1 coordinate
 * with specified radius and angle
 * 
 * @param[in] cx1 the x coordinate
 * @param[in] cy1 the y coordinate
 * @param[in] r the radius (in pixels)
 * @param[in] deg1 start angle in degrees
 * @param[in] deg2 stop  angle in degrees
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_fill_arc(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg1, int16_t deg2, uint32_t argb);

/**
 * @brief draw image
 *
 * This routine will render a bitmap of the specified
 * width, height and color format to the specified location
 *
 * @param[in] x the x position of the upper left hand corner of the image
 * @param[in] y the y position of the upper left hand corner of the image
 * @param[in] w the width of the image
 * @param[in] h the height of the image
 * @param[in] type color format of the image 
 * @param[in] pixels pointer to the byte buffer of the picture
 * @param[in] usr_fmt user/custom parameter to specify implementation dependent
 *         options like striping or tiling
 */
int8_t UG_driver_gfx_draw_image(int16_t x, int16_t y, UG_DRV_CFG_FMT_COLOR type, uint8_t *pixels, uint16_t w, uint16_t h, void* usr_fmt); 


/**
 * @brief move screen memory area from one location to another
 * 
 * This command emulates a bit block transfer function to move
 * display pixels from one area of the display memory to another.
 * 
 * @param[in] x upper left hand x coordinate or region
 * @param[in] y upper left hand y coordinate or region
 * @param[in] w width of region
 * @param[in] h height of region
 * @param[in] off_x displacement to move image
 * @param[in] off_y displacement to move image
 */
int8_t UG_driver_gfx_move(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t off_x, int16_t off_y);

/**
 * @brief draw a polygon (not filled) with the given vertexes
 * 
 * This routine will draw a polygon at the specified x,y coordinate
 * 
 * @param[in] x array of the x coordinates
 * @param[in] y array of the y coordinates
 * @param[in] vertex_num the number of vertex to join
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_poly(int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb); 

/**
 * @brief draw a polygon (filled) with the given vertexes
 * 
 * This routine will draw a polygon at the specified x,y coordinate
 * 
 * @param[in] x array of the x coordinates
 * @param[in] y array of the y coordinates
 * @param[in] vertex_num the number of vertex to join
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_fill_poly(int16_t **x, int16_t **y, uint16_t vertex_num, uint32_t argb);

/**
 * @brief draw a regular polygon (not filled) with the given vertexes
 * 
 * This routine will draw a regular polygon inside the specified circle outline
 * 
 * @param[in] cx1 center coordinate x of the circle
 * @param[in] cy1 center coordinate y of the circle
 * @param[in] r radius of the circle
 * @param[in] deg angle of orientation of the first vertex
 * @param[in] vertex_num the number of vertex to join
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_draw_polyreg(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb); 

/**
 * @brief draw a regular polygon (filled) with the given vertexes
 * 
 * This routine will draw a regular polygon inside the specified circle outline
 * 
 * @param[in] cx1 center coordinate x of the circle
 * @param[in] cy1 center coordinate y of the circle
 * @param[in] r radius of the circle
 * @param[in] deg angle of orientation of the first vertex
 * @param[in] vertex_num the number of vertex to join
 * @param[in] argb the color to be used
 */
int8_t UG_driver_gfx_fill_polyreg(int16_t cx1, int16_t cy1, uint16_t r, int16_t deg, uint16_t vertex_num, uint32_t argb); 


/* ************************* */
/* COMMUNICATION */
/* ************************* */

/** 
 * @brief Display the display configuration on boot
 * 
 * This is an enable/disable to show implementation dependent
 * config information direclty on the display at power-up/boot.
 * 
 * @param[in] flag when non-zero, will program the controller to show
 *            display configuration on power-up/boot
 */
int8_t UG_driver_com_enable_config(uint8_t flag);


/* ************************* */
/* POWER */
/* ************************* */

/** 
 * @brief set display brightness level
 * 
 * if supported by the display this will change
 * the backlight level
 * 
 * @param[in] lvl the contrast value (0-255)
 */
int8_t UG_driver_pwr_set_brightness(uint8_t lvl);

/** 
 * @brief turn ON/OFF the display screem
 * 
 * if supported by the display this will turn Off
 * the display to save power, communication is still
 * possible to render data
 * 
 * @param[in] flag 1:on 0:0ff
 */
int8_t UG_driver_pwr_set_onoff(uint8_t flag);

/** 
 * @brief turn ON/OFF the display onboard MCU
 * 
 * if supported by the display this will turn Off
 * the display MCU to save power but the display
 * is still on and will show the previous graphics
 * communication for rendering is not possible
 * 
 * @param[in] flag 1:on 0:0ff
 */
int8_t UG_driver_pwr_set_sleep(uint8_t flag);


/** 
 * @brief turn ON/OFF the display and onboard MCU
 * 
 * @param[in] flag 1:on 0:0ff
 */
int8_t UG_driver_pwr_set_deepsleep(uint8_t flag);


/* ************************* */
/* SCREEN */
/* ************************* */

/**
 * @brief set the display contrast
 * 
 * This command sets the display contrast 
 * 
 * @param[in] lvl the contrast value (0-255)
 */
int8_t UG_driver_scr_set_contrast(uint8_t lvl);

/**
 * @brief upload splash screen data to display controller
 * 
 * Upload display bitmap for startup screen
 *
 * @param[in] data the address of the data buffer
 * @param[in] len the length of the bitmap
 */
int8_t UG_driver_scr_set_splash(uint8_t *data, uint32_t data_len);

/** 
 * @brief Display the start screen on power-up/boot
 * 
 * This is an enable/disable to show the splash screen on boot.
 * The start-up screen can be programmed to the display memory.
 * 
 * @param[in] m when non-zero, will program the controller to show
 * startup screen on power-up/boot.
 */ 
int8_t UG_driver_scr_enable_splash(uint8_t flag);


/* ************************* */
/* TOUCH PANEL */
/* ************************* */
int8_t UG_driver_tch_calibrate(uint8_t value);
int8_t UG_driver_tch_wait(int16_t *x, int16_t *y, uint8_t press_release_f);
int8_t UG_driver_tch_check(int16_t *x, int16_t *y);


/* ************************* */
/* FLASH */
/* ************************* */
int8_t UG_driver_flash_read(uint8_t* addr, uint8_t *len , uint8_t *data);
int8_t UG_driver_flash_write(uint8_t* addr, uint8_t *len , uint8_t *data);
int8_t UG_driver_flash_erase(uint8_t* addr, uint8_t *len);


/* ************************* */
/* EEPROM */
/* ************************* */
int8_t UG_driver_eeprom_read(uint8_t* addr, uint8_t *len , uint8_t *data);
int8_t UG_driver_eeprom_write(uint8_t* addr, uint8_t *len , uint8_t *data);

#endif
