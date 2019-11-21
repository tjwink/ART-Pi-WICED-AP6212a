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

/** @file
 *
 * uGui Library GraphicsTest Sample Application
 *
 *
 */
#include <math.h>
#include "wiced.h"
#include "ugui.h"

/* custom fonts binary vectors */
#include "u8g_font_walkman.h"
#include "u8g_font_unifont_76.h"
#include "u8g_font_bar_5x20.h"

/* sin/cos for rainbow computation */
#include "trig.h"

/* custom images binary vectors */
#include "img_mono.h"
#include "img_rgb332.h"
#include "img_rgb332_cover.h"
#include "img_rgb565.h"
#include "img_rgb888.h"


/******************************************************
 * IMPORTANT: INCLUDE ONE DISPLAY CONFIG (ONLY ONE!)
 * NOTE: check parameters of the display cfg file (.h)
 ******************************************************/

#include "display_cfg_digole.h"
//#include "display_cfg_ssd1306.h"

/******************************************************
 * IMPORTANT: SELECT LOADING USR_FONTS and/or USR_SPLASH
 * Note: for Digole displays this is needed the very
 *       first time you use a brand new display,
 *       after you have loaded fonts and splash data
 *       keep these defines disabled.
 ******************************************************/

//#define UGUI_LOAD_USR_FONTS
//#define UGUI_LOAD_USR_SPLASH


/******************************************************
 *                      Macros
 ******************************************************/
#define MAX_OBJECTS        ( 10 )
#define TEST_PAUSE_LONG  ( 3000 )
#define TEST_PAUSE_SHORT ( 1000 )

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
void test_text_01( );
void test_text_02( );
void test_text_03( );

void test_lines_01( uint16_t w, uint16_t h );
void test_rectangles_01( uint16_t w, uint16_t h );
void test_circles_01( uint16_t w, uint16_t h );
void load_usr_fonts( );
void load_splash_screen( );

void animation_walkman_01( );
void animation_walkman_02( );

void animation_vumeter_01( );
void animation_vumeter_02( );
void animation_vumeter_03( );
void animation_vumeter_04( );
void animation_vumeter_05( );

void test_image_01( uint16_t w, uint16_t h );

void window_1_callback( UG_MESSAGE* msg );
void ugui_test_01( uint16_t w, uint16_t h );
void ugui_test_02( );

/******************************************************
 *               Variables Definitions
 ******************************************************/

UG_GUI             gui;     // Global GUI obj
UG_BUS_T           gui_bus; // Global BUS obj
UG_DRV_T           gui_drv; // Global DRIVER obj

static uint32_t    sine_x = 0;


/******************************************************
 *               Function Definitions
 ******************************************************/
uint32_t get_color_rainbow( )
{
#define SIN_PHASE    ( 0 )
#define SIN_PERIOD ( 128 )

    uint8_t center = 128;
    uint8_t width = 127;
    float   frequency = 6.28 / ( 320 );

    uint8_t red   = (uint8_t) ( ( pseudo_sin( frequency * ( ( sine_x++ ) ) + 2 + SIN_PHASE ) * width + center ) );
    uint8_t green = (uint8_t) ( ( pseudo_sin( frequency * ( ( sine_x++ ) ) + 0 + SIN_PHASE ) * width + center ) );
    uint8_t blue  = (uint8_t) ( ( pseudo_sin( frequency * ( ( sine_x++ ) ) + 4 + SIN_PHASE ) * width + center ) );

    return ( red << 16 ) | ( green << 8 ) | blue;
}


/******************************************************
 *               MAIN
 ******************************************************/
void application_start( )
{
    UG_BUS_ERR    bus_err;
    UG_DRV_ERR    drv_err;


    /*
     *  BUS
     */
    printf( "UG_BUS: init ... \n" );
#if defined USE_UGUI_BUS_SPI
    UG_bus_init( &bus_err, &gui_bus, UG_BUS_PROTO_SPI, (wiced_spi_device_t*) &display_spi );
#endif
#if defined USE_UGUI_BUS_I2C
    UG_bus_init( &bus_err, &gui_bus, UG_BUS_PROTO_I2C, (wiced_i2c_device_t*) &display_i2c );
#endif
    if ( bus_err != UG_BUS_NO_ERR )
    {
        /* we can't continue, stop here */
        while(1)
        {
            printf( "bus failure\n" );
            wiced_rtos_delay_milliseconds( 3000 );
        }
    }
    else
    {
        printf( "success\n" );
    }

    /*
     *  DRIVER
     */
    printf( "UG_DRV: create ... \n" );
    UG_driver_init( &drv_err, &gui_drv, (UG_DRV_CONFIG*) &drv_cfg );
    if ( drv_err != UG_DRV_NO_ERR )
    {
        /* we can't continue, stop here */
        while(1)
        {
            printf( "driver failure\n" );
            wiced_rtos_delay_milliseconds( 3000 );
        }
    }
    else
    {
        printf( "success\n" );
    }

    /*
     * IMPORTANT: make sure we wakeup the display
     */
    UG_driver_pwr_set_sleep(0);

    /*
     * initial delay to allow the splash screen to be visible
     * since we do a clear screen immediately after this
     */

    wiced_rtos_delay_milliseconds( 3000 ); /* pause */

    /*
     * DRIVER: set defaults (will clear screen)
     */
#if defined ( UGUI_LOAD_USR_SPLASH )
    UG_driver_raw_set_defaults(NULL, &gui_drv);
#endif
    UG_driver_txt_show_cursor( 0, 0 );
    UG_driver_sys_clear_screen( );


    /*
     * LOAD USR FONTS
     * Note: needs to be done only once for each
     * display device, skip this if already loaded.
     */
    load_usr_fonts( );

    /*
     * LOAD & ENABLE SPLASH SCREEN
     */
    load_splash_screen( );

    /*
     * TEST PHASE ONE:
     * testing only direct driver calls
     */
    UG_driver_sys_clear_screen( );

    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_RED );
    UG_driver_txt_write_rowcol( "IMPORTANT", 9, 0, 1 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_write_rowcol( "First test UG_DRV   ", 20, 0, 3 );
    UG_driver_txt_write_rowcol( "using *DIRECT* calls", 20, 0, 4 );
    wiced_rtos_delay_milliseconds( 4000 );

    /*
     * UG_FONT match search
     */
    UG_driver_sys_clear_screen( );
    printf( "font search for %p\n", ugui_u8g_font_unifont_76 );
    uint8_t       hw_id_new = 255;
    UG_driver_fnt_search( &hw_id_new, (uint8_t*) ugui_u8g_font_unifont_76 );
    printf( " hw_id=%d\n", hw_id_new );

    /*
     * text
     */
    test_text_01( );
    test_text_02( );
    test_text_03( );

    /*
     * lines
     */
    test_lines_01( DISPLAY_W_PX,  DISPLAY_H_PX );

    /*
     * rectangles
     */
    test_rectangles_01( DISPLAY_W_PX,  DISPLAY_H_PX );

    /*
     * circles
     */
    test_circles_01( DISPLAY_W_PX,  DISPLAY_H_PX );

    /*
     * UG_DRV: animation with USR fonts
     */

    animation_walkman_01( );
    animation_walkman_02( );

    /*
     * UG_DRV: animation with USR fonts for VU METER
     */

    animation_vumeter_01( );
    // animation_vumeter_02();
    animation_vumeter_03( );
    // animation_vumeter_04();
    animation_vumeter_05( );

    /*
     * UG_DRV: image draw demo
     */
    test_image_01( DISPLAY_W_PX,  DISPLAY_H_PX );

    /*
     * TEST PHASE TWO:
     * testing only uGUI api calls
     */
    UG_driver_sys_clear_screen( );

    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_RED );
    UG_driver_txt_write_rowcol( "IMPORTANT", 9, 0, 1 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_write_rowcol( "Second test uGUI api", 20, 0, 3 );
    UG_driver_txt_write_rowcol( "windows & buttons   ", 20, 0, 4 );
    wiced_rtos_delay_milliseconds( 4000 );

    /*
     * uGUI: create one gui obj
     */
    UG_Init( &gui, UG_driver_gfx_pixel_set, DISPLAY_W_PX,  DISPLAY_H_PX );
    UG_SelectGUI( &gui );

    /*
     * uGUI: register driver hw accelerated primitives
     * note: this is a macro declared into each
     *       display include file at the top.
     */
    ENABLE_UG_HW_DRIVER();

    /*
     * GUI: window example
     */
    ugui_test_01( DISPLAY_W_PX,  DISPLAY_H_PX );
    ugui_test_02( );


    /*
     * release the driver, bus and exit
     */
    UG_bus_deinit( &bus_err, &gui_bus);
    UG_driver_deinit( &drv_err, &gui_drv);

    printf( "bye bye.\n" );
}


/******************************************************
 *               Tests
 ******************************************************/
void test_text_01( )
{
    UG_driver_sys_clear_screen( );

    /* use hw_ids for Digole API: 6, 10, 18 51 */

    /*
     * code  font_name Font code Font name
     * 6     u8g_font_4x6
     * 10    u8g_font_6x10
     * 18    u8g_font_9x18B
     * 51    u8g_font_osr18
     * 120   u8g_font_gdr20
     * 123   u8g_font_osr35n
     * 0(default) u8g_font_unifont
     */

    UG_driver_fnt_set( 6 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_write_rowcol( "Hello World", 11, 0, 0 );

    UG_driver_fnt_set( 10 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_write_rowcol( "Hello World", 11, 0, 1 );

    UG_driver_fnt_set( 18 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_write_rowcol( "Hello World", 11, 0, 2 );

    UG_driver_fnt_set( 51 );
    UG_driver_txt_write_rowcol( "Hello World", 11, 0, 2 );

    UG_driver_fnt_set( 120 );
    UG_driver_txt_write_rowcol( "Hello World", 11, 0, 2 );

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
}


void test_text_02( )
{
    UG_driver_sys_clear_screen( );

    UG_driver_fnt_set( 201 );
    UG_driver_sys_set_forecolor( C_GREEN );
    UG_driver_txt_set_pos_rowcol( 0, 0 );

    char text[] = { 48 + 0, 48 + 1, 48 + 2, 48 + 3, 48 + 4 };
    UG_driver_txt_set_pos_rowcol( 0, 0 );
    UG_driver_txt_write( &text[ 0 ], 5 );

    char text2[ 250 ];
    uint i;
    for ( i = 0; i < 100; i++ )
    {
        text2[ i ] = i + 40;
    }

    UG_driver_fnt_set( 200 );
    UG_driver_sys_set_forecolor( C_RED );
    UG_driver_txt_set_pos_rowcol( 0, 6 );
    UG_driver_txt_write( &text2[ 0 ], 100 );

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
}


void test_text_03( )
{
    uint8_t x_off = rand( ) % 4;
    uint8_t i = 0;

    UG_driver_sys_clear_screen( );

    UG_driver_fnt_set( 0 );

    for ( i = 0; i < 20; i++ )
    {
        UG_driver_sys_set_draw_rot( 90 * ( i % 4 ) );
        UG_driver_sys_set_forecolor( get_color_rainbow( ) );
        x_off = rand( ) % 4;
        UG_driver_txt_write_rowcol( "Hello World", 11, x_off, ( i % 4 ) + ( i >> 2 ) );
    }

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
}


void test_lines_01( uint16_t w, uint16_t h )
{
    uint32_t x, y;

    UG_driver_sys_clear_screen( );

    for ( x = 0; x < ( w - 2 ); x += 2 )
    {
        UG_driver_gfx_draw_line( 0, ( h - 1 ), x, 0, C_WHITE );
    }
    for ( y = 0; y < ( h - 2 ); y += 2 )
    {
        UG_driver_gfx_draw_line( 0, ( h - 1 ), ( w - 1 ), y, C_WHITE );
    }

    wiced_rtos_delay_milliseconds( 3000 );
    UG_driver_sys_clear_screen( );

    for ( x = 0; x < ( w - 2 ); x += 2 )
    {
        UG_driver_gfx_draw_line( ( w - 1 ), ( h - 1 ), ( w - 1 - x ), 0, get_color_rainbow( ) );
    }
    for ( y = 0; y < ( h - 2 ); y += 2 )
    {
        UG_driver_gfx_draw_line( ( w - 1 ), ( h - 1 ), 0, y, get_color_rainbow( ) );
    }

    wiced_rtos_delay_milliseconds( 3000 );
}


void test_rectangles_01( uint16_t w, uint16_t h )
{
    uint8_t  x_r = ( rand( ) % w );
    uint8_t  y_r = ( rand( ) % h );

    uint8_t  w_r = ( rand( ) % w );
    uint8_t  h_r = ( rand( ) % h );

    uint32_t i = 0;

    UG_driver_sys_clear_screen( );

    for ( i = 0; i < 100; i++ )
    {
        x_r = ( rand( ) % w );
        y_r = ( rand( ) % h );
        w_r = ( rand( ) % w );
        h_r = ( rand( ) % h );
        UG_driver_gfx_draw_rect( x_r, y_r, x_r + w_r, y_r + h_r, get_color_rainbow( ) );
    }

    wiced_rtos_delay_milliseconds( 3000 );

    UG_driver_sys_clear_screen( );
    for ( i = 0; i < 80; i++ )
    {
        x_r = ( rand( ) % w );
        y_r = ( rand( ) % h );
        w_r = ( rand( ) % ( w / 2 ) );
        h_r = ( rand( ) % ( h / 2 ) );
        UG_driver_gfx_fill_rect( x_r, y_r, x_r + w_r, y_r + h_r, get_color_rainbow( ) );
    }
    wiced_rtos_delay_milliseconds( 3000 );
}


void test_circles_01( uint16_t w, uint16_t h )
{
    uint8_t  x_r = ( rand( ) % w );
    uint8_t  y_r = ( rand( ) % h );
    uint8_t  h_r = ( rand( ) % h );

    uint32_t i = 0;

    UG_driver_sys_clear_screen( );

    for ( i = 0; i < 100; i++ )
    {
        x_r = ( rand( ) % w );
        y_r = ( rand( ) % h );
        h_r = ( rand( ) % h );
        UG_driver_gfx_draw_circle( x_r, y_r, h_r, get_color_rainbow( ) );
    }

    wiced_rtos_delay_milliseconds( 3000 );

    UG_driver_sys_clear_screen( );
    for ( i = 0; i < 80; i++ )
    {
        x_r = ( rand( ) % w );
        y_r = ( rand( ) % h );
        h_r = ( rand( ) % ( h / 4 ) );
        UG_driver_gfx_fill_circle( x_r, y_r, h_r, get_color_rainbow( ) );
    }
    wiced_rtos_delay_milliseconds( 3000 );
}


void load_usr_fonts( )
{
#if defined ( UGUI_LOAD_USR_FONTS )

    /*
     * upload user font on slot hw_id=200
     */

    UG_driver_txt_set_pos_rowcol( 0, 5 );
    UG_driver_txt_write( "loading USR font:", 17 );
    UG_driver_txt_set_pos_rowcol( 0, 6 );
    UG_driver_txt_write( "u8g_walkman", 12 );
    UG_driver_fnt_upload( 201,
                          (uint8_t*) ugui_u8g_font_walkman,
                          (uint8_t*) ugui_u8g_font_walkman,
                          sizeof( ugui_u8g_font_walkman ) );

    /*
     * upload user font on slot hw_id=201
     */

    wiced_rtos_delay_milliseconds( 1000 ); /* pause */

    UG_driver_sys_clear_screen( );
    UG_driver_txt_set_pos_rowcol( 0, 5 );
    UG_driver_txt_write( "loading USR font:", 17 );
    UG_driver_txt_set_pos_rowcol( 0, 6 );
    UG_driver_txt_write( "u8g_unifont_76", 14 );
    UG_driver_fnt_upload( 200,
                          (uint8_t*) ugui_u8g_font_unifont_76,
                          (uint8_t*) ugui_u8g_font_unifont_76,
                          sizeof( ugui_u8g_font_unifont_76 ) );

    /*
     * upload user font on slot hw_id=202
     */

    wiced_rtos_delay_milliseconds( 1000 ); /* pause */

    UG_driver_txt_set_pos_rowcol( 0, 5 );
    UG_driver_txt_write( "loading USR font:", 17 );
    UG_driver_txt_set_pos_rowcol( 0, 6 );
    UG_driver_txt_write( "ugui_u8g_font_bar_5x20", 12 );
    UG_driver_fnt_upload( 202,
                          (uint8_t*) ugui_u8g_font_bar_5x20,
                          (uint8_t*) ugui_u8g_font_bar_5x20,
                          sizeof( ugui_u8g_font_bar_5x20 ) );

    wiced_rtos_delay_milliseconds( 1000 ); /* pause */

#else

    /*
     * LOAD USR FONTS LUT
     * customize the font corrispondance hw_id:ug_font
     * which is needed every time we boot
     */

    UG_DRV_FONT_LUT lut[] =
    {
        /* FONT_PTR             , hw_id */
        { (void*) &FONT_6X8,                        6 },
        { (void*) &ugui_u8g_font_walkman[ 0 ],    201 },
        { (void*) &ugui_u8g_font_unifont_76[ 0 ], 200 },
    };

    UG_driver_fnt_usrlut( lut, 3 );
#endif
}


void load_splash_screen( )
{
#if defined ( UGUI_LOAD_USR_SPLASH )
    /* NOTE: */
    /* splash command set for DIGOLE color display ONLY */
    /* change this if you have a different display!!! */

    /*
     * - hello world
     * - square box
     */
    uint8_t splash_cmd_set[] =
    {
        'C', 'L',
        'S', 'C', 0x38,
        'S', 'F',    0,
        'T', 'P', 0x04, 0x04,
        'T', 'T','H',  'e',  'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', 0x00,
        'S', 'C', 0xE0,
        'D', 'R',   10,   10,( 160 - 10 ),  ( 128 - 10 )
    };

    printf( "uploading splash screen..\n" );

    UG_driver_scr_set_splash( splash_cmd_set, sizeof( splash_cmd_set ) );
    UG_driver_scr_enable_splash( 1 );

    printf( "done.\n" );

    wiced_rtos_delay_milliseconds( 3000 ); /* pause */
#endif
}


void animation_walkman_01( )
{
    uint8_t i = 0;
    uint8_t ii = 0;

    char    text[] = { 48 + 0, 48 + 1, 48 + 2, 48 + 3, 48 + 4 };


    /* Note: the MIN delay in between writes depends on your
     * display capabilities to refresh/redraw large areas depening
     * on the font size. But usually the refresh of the animation
     * is bigger than the display delay */
#define WALK_DELAY ( 100 ) /*10fps*/

    printf( "walkman animation\n" );

    UG_driver_sys_clear_screen( );
    UG_driver_fnt_set( 201 );
    UG_driver_sys_set_forecolor( C_GREEN );

    for ( i = 0; i < 5; i++ )
    {
        for ( ii = 0; ii < 5; ii++ )
        {
            UG_driver_txt_write_xy( &text[ ii % 5 ], 1, 70, 100 );
            wiced_rtos_delay_milliseconds( WALK_DELAY );
        }
    }

    /* HW ACCELL on the display is not multi-thread.
     * if the display MCU is busy it will not service the
     * intput command buffer. Delay to make sure the display
     * input buffer is drained before continuing*/
    wiced_rtos_delay_milliseconds( 500 );
}


void animation_walkman_02( )
{
    uint8_t i = 0;
    uint8_t ii = 0;
    char    text[] = { 48 + 0, 48 + 1, 48 + 2, 48 + 3, 48 + 4 };

    uint8_t cx = 0;

    /* Note: the MIN delay in between writes depends on your
     * display capabilities to refresh/redraw large areas depening
     * on the font size. But usually the refresh of the animation
     * is bigger than the display delay */
#define WALK_LONG ( 80 ) /*3fps*/

#define WALK_STEP ( 3 )

    printf( "walkman animation\n" );

    UG_driver_sys_clear_screen( );
    UG_driver_fnt_set( 201 );
    UG_driver_sys_set_forecolor( C_GREEN );

    for ( i = 0; i < 9; i++ )
    {
        for ( ii = 0; ii < 5; ii++ )
        {
            UG_driver_sys_set_forecolor( C_BLACK );
            UG_driver_txt_write_xy( &text[ ii % 5 ], 1, cx, 100 );

            cx = ( ( cx + WALK_STEP ) > ( 160 - 20 ) ) ? 0 : cx + WALK_STEP;

            UG_driver_sys_set_forecolor( C_GREEN );
            UG_driver_txt_write_xy( &text[ ii % 5 ], 1, cx, 100 );

            wiced_rtos_delay_milliseconds( WALK_LONG );
        }
    }

    /* HW ACCELL on the display is not multi-thread.
     * if the display MCU is busy it will not service the
     * intput command buffer. Delay to make sure the display
     * input buffer is drained before continuing*/
    wiced_rtos_delay_milliseconds( 500 );
}


void animation_vumeter_01( )
{
#define VU_BAR_MAX     ( 256 )
#define VU_BAR_STEP    ( 256 / 4 )

    uint32_t loop = 0;
    int8_t   i = 0;
    int8_t   k = 0;

    uint32_t tmp32;

    UG_driver_sys_clear_screen( );

    /* simulate a stereo VU-METER  */
    /* 16 bands per channels       */
    /* using custom fonts for bars */

    uint8_t  lvl[ 32 ] = { 0 };
    char     bar[ 32 ] = { 0 };
    uint32_t barcolor[ 4 ] = { C_DARK_GREEN, C_GREEN, C_YELLOW, C_RED };

    /* title on top */
    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_WHITE );
    UG_driver_txt_write_rowcol( "2x16 LR VU-METER sim", 20, 0, 0 );
    UG_driver_txt_write_rowcol( "custom u8g font bars", 20, 0, 1 );


    /* we use the bar font, chars ABCDE */
    UG_driver_fnt_set( 202 );
    UG_driver_sys_set_forecolor( C_BLACK );


    /* use the custom font */
    UG_driver_fnt_set( 202 );

    for ( loop = 0; loop < 256; loop++ )
    {
        wiced_rtos_delay_milliseconds( 60 ); /* 15fps */

        /* generate two vectors for vu meter levels */
        for ( i = 0; i < 16; i++ )
        {
#if 1
            tmp32 = 255 - loop + (uint8_t) ( rand( ) % 64 );
            lvl[ i ] = ( 255 < tmp32 ) ? 255 : tmp32;

            tmp32 = loop + (uint8_t) ( rand( ) % 64 );
            lvl[ i + 16 ] = ( 255 < tmp32 ) ? 255 : tmp32;
#else
            lvl[ i ]    = i * 15;
            lvl[ i + 16 ] = ( 15 - i ) * 15;
#endif
        }

        /* update bars, 4 lines of 32 chars each, 4 levels for each char (usr font)*/
        for ( k = 3; k >= 0; k-- )
        {
            for ( i = 0; i < 32; i++ )
            {
                bar[ i ] = ( ( lvl[ i ] >> 6 ) > k ) ? 'E' : 'A';

                if ( ( lvl[ i ] >> 6 ) == k )
                {
                    switch ( ( lvl[ i ] >> 4 ) & 0x03 )
                    {
                        case 0x03:
                            bar[ i ] = 'E';
                            break;

                        case 0x02:
                            bar[ i ] = 'D';
                            break;

                        case 0x01:
                            bar[ i ] = 'C';
                            break;

                        case 0x00:
                            bar[ i ] = 'B';
                            break;
                    }
                }
            }

            UG_driver_sys_set_forecolor( barcolor[ k ] );
            UG_driver_txt_write_rowcol( (char*) &bar[ 0 ], 32, 0, (int16_t) ( 2 + ( 3 - k ) ) );
        }
    }

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
}


void animation_vumeter_02( )
{
    uint32_t loop = 0;
    int8_t   i = 0;


    UG_driver_sys_clear_screen( );

    /* simulate a stereo VU-METER  */
    /* 16 bands per channels       */
    /* using custom fonts for bars */

    uint8_t  lvl[ 32 ] = { 0 };

    uint32_t tmp32 = 0;


    /* title on top */
    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_WHITE );
    UG_driver_txt_write_rowcol( "2x16 LR VU-METER sim", 20, 0, 0 );
    UG_driver_txt_write_rowcol( "gfx fill_rect calls ", 20, 0, 1 );


    for ( loop = 0; loop < 256; loop++ )
    {
        wiced_rtos_delay_milliseconds( 60 ); /* 15fps */

        /* generate two vectors for vu meter levels */
        for ( i = 0; i < 16; i++ )
        {
            tmp32 = 255 - loop + (uint8_t) ( rand( ) % 64 );
            lvl[ i ] = ( 255 < tmp32 ) ? 255 : tmp32;

            tmp32 = loop + (uint8_t) ( rand( ) % 64 );
            lvl[ i + 16 ] = ( 255 < tmp32 ) ? 255 : tmp32;
        }

        /* update bars, monocrome (colored would be slower) */

        for ( i = 0; i < 32; i++ )
        {
            /* squeeze 255 to 85 to compare to vumeter_01 */
            UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ i ] / 3, ( i * 5 + 3 ), 127,                C_GREEN );
            UG_driver_gfx_fill_rect( ( i * 5 ), 127 - ( 255 / 3 ),  ( i * 5 + 3 ), 127 - lvl[ i ] / 3, C_BLACK );
        }
    }

    wiced_rtos_delay_milliseconds( 2000 ); /* pause */
}


void animation_vumeter_03( )
{
    uint32_t loop = 0;
    int8_t   i = 0;


    UG_driver_sys_clear_screen( );

    /* simulate a stereo VU-METER  */
    /* 16 bands per channels       */
    /* using custom fonts for bars */

    uint8_t  lvl[ 2 ][ 32 ];
    uint8_t  idx  = 0;
    uint8_t  idxo = 1;

    uint32_t tmp32 = 0;

    /* clear array */
    for ( idx = 0; idx < 2; idx++ )
    {
        for ( idxo = 0; idxo < 32; idxo++ )
        {
            lvl[ idx ][ idxo ] = 0;
        }
    }

    /* title on top */
    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_WHITE );
    UG_driver_txt_write_rowcol( "2x16 LR VU-METER sim", 20, 0, 0 );
    UG_driver_txt_write_rowcol( "gfx fill_rect optim.", 20, 0, 1 );
    UG_driver_txt_write_rowcol( "(only re-draw delta)", 20, 0, 2 );

    /* init indexes */
    idx  = 0;
    idxo = 1;

    /* we use the bar font, chars ABCDE */
    for ( loop = 0; loop < 256; loop++ )
    {
        wiced_rtos_delay_milliseconds( 60 ); /* 15fps */

        /* swap level buffer */
        idxo = idx;
        idx  = ( idx + 1 ) & 0x01;

        /* generate two vectors for vu meter levels */
        for ( i = 0; i < 16; i++ )
        {
            tmp32 = 255 - loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i ] = ( 255 < tmp32 ) ? 255 : tmp32;

            tmp32 = loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i + 16 ] = ( 255 < tmp32 ) ? 255 : tmp32;
        }

        /* update bars, monocrome (colored would be slower) */
        for ( i = 0; i < 32; i++ )
        {
            /* speed opt: only patch the missing part */
            if ( lvl[ idx ][ i ] > lvl[ idxo ][ i ] )
            {
                UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ idx ][ i ] / 3, ( i * 5 + 3 ), 127 - lvl[ idxo ][ i ] / 3, C_GREEN );
            }
            else if ( lvl[ idx ][ i ] < lvl[ idxo ][ i ] )
            {
                UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ idxo ][ i ] / 3, ( i * 5 + 3 ), 127 - lvl[ idx ][ i ] / 3, C_BLACK );
            }
        }
    }

    wiced_rtos_delay_milliseconds( 2000 ); /* pause */
}


void animation_vumeter_04( )
{
    uint32_t loop = 0;
    int8_t   i = 0;


    UG_driver_sys_clear_screen( );

    /* simulate a stereo VU-METER  */
    /* 16 bands per channels       */
    /* using custom fonts for bars */

    uint8_t  lvl[ 2 ][ 32 ];
    uint8_t  idx  = 0;
    uint8_t  idxo = 1;

    uint32_t tmp32 = 0;

    /* clear lvl array */
    for ( idx = 0; idx < 2; idx++ )
    {
        for ( idxo = 0; idxo < 32; idxo++ )
        {
            lvl[ idx ][ idxo ] = 0;
        }
    }

    /* cover art 64x64 pixesl*/
    UG_driver_gfx_draw_image( 0, 0, UG_DRV_FMT_RGB332, (uint8_t*) img_rgb332_cover, 64, 64, NULL );

    /* title on top */
    UG_driver_fnt_set( 0 );
    UG_driver_sys_set_forecolor( C_WHITE );
    UG_driver_txt_write_rowcol( " author   ", 10, 10, 0 );
    UG_driver_txt_write_rowcol( " album    ", 10, 10, 1 );
    UG_driver_txt_write_rowcol( "trk/title ", 10, 10, 2 );
    UG_driver_txt_write_rowcol( " hh:mm.ss ", 10, 10, 4 );

    UG_driver_sys_set_forecolor( C_GREEN );

    /* init indexes */
    idx  = 0;
    idxo = 1;

    /* we use the bar font, chars ABCDE */
    for ( loop = 0; loop < 256; loop++ )
    {
        wiced_rtos_delay_milliseconds( 60 ); /* 15fps */

        /* swap level buffer */
        idxo = idx;
        idx  = ( idx + 1 ) & 0x01;

        /* generate two vectors for vu meter levels */
        for ( i = 0; i < 16; i++ )
        {
            tmp32 = 255 - loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i ] = ( 255 < tmp32 ) ? 255 : tmp32;

            tmp32 = loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i + 16 ] = ( 255 < tmp32 ) ? 255 : tmp32;
        }

        /* update bars, monocrome (colored would be slower) */
        for ( i = 0; i < 32; i++ )
        {
            // wiced_rtos_delay_milliseconds(60); /* 30fps */
            /* speed opt: only patch the missing part */
            if ( lvl[ idx ][ i ] > lvl[ idxo ][ i ] )
            {
                UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ idx ][ i ] / 5, ( i * 5 + 3 ), 127 - lvl[ idxo ][ i ] / 5, C_GREEN );
            }
            else if ( lvl[ idx ][ i ] < lvl[ idxo ][ i ] )
            {
                UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ idxo ][ i ] / 5, ( i * 5 + 3 ), 127 - lvl[ idx ][ i ] / 5, C_BLACK );
            }
        }
    }

    wiced_rtos_delay_milliseconds( 2000 ); /* pause */
}


void animation_vumeter_05( )
{
#define VUMETER_BAR_MAX       ( 255 / 5 )

#define VUMETER_BAR_RED_SZ        ( 3 )  /* RED        */
#define VUMETER_BAR_YELLOW_SZ     ( 7 )  /* YELLOW     */
#define VUMETER_BAR_GREEN_SZ     ( 14 )  /* GREEN      */
#define VUMETER_BAR_DGREEN_SZ    ( 28 )  /* DARK GREEN */

#define VUMETER_BAR_RED_TH      ( VUMETER_BAR_YELLOW_SZ + VUMETER_BAR_GREEN_SZ + VUMETER_BAR_DGREEN_SZ )
#define VUMETER_BAR_YELLOW_TH   ( VUMETER_BAR_GREEN_SZ + VUMETER_BAR_DGREEN_SZ )
#define VUMETER_BAR_GREEN_TH    ( VUMETER_BAR_DGREEN_SZ )
#define VUMETER_BAR_DGREEN_TH   ( 0 )

    uint32_t loop = 0;
    int8_t   i = 0;
    int8_t   k = 0;

    uint32_t vumeter_color[ 4 ] = { C_DARK_GREEN, C_GREEN, C_YELLOW, C_RED };
    uint8_t  vumeter_th[ 4 ]    =
    {
        VUMETER_BAR_DGREEN_TH, VUMETER_BAR_GREEN_TH,
        VUMETER_BAR_YELLOW_TH, VUMETER_BAR_RED_TH
    };

    UG_driver_sys_clear_screen( );

    /* simulate a stereo VU-METER  */
    /* 16 bands per channels       */
    /* using custom fonts for bars */

    uint8_t  lvl[ 2 ][ 32 ];
    uint8_t  idx  = 0;
    uint8_t  idxo = 1;

    uint32_t tmp32 = 0;
    uint8_t  tmp8 = 0;

    /* clear lvl array */
    for ( idx = 0; idx < 2; idx++ )
    {
        for ( idxo = 0; idxo < 32; idxo++ )
        {
            lvl[ idx ][ idxo ] = 0;
        }
    }

    /* cover art 64x64 pixesl*/
    // UG_driver_gfx_draw_image(0, 0, UG_DRV_FMT_RGB332, (uint8_t*)img_rgb332_cover, 64, 64, NULL);
    UG_driver_gfx_draw_image( 0, 0, UG_DRV_FMT_RGB332, (uint8_t*) img_rgb332_cover, 64, 64, NULL );

    // UG_driver_gfx_draw_rect(66, 0, 159, 64, C_DARK_GRAY);

    /* title on top */
    UG_driver_fnt_set( 10 );
    UG_driver_sys_set_forecolor( C_WHITE );
    UG_driver_txt_write_rowcol( "Author Name   ", 14, 12, 0 );
    UG_driver_txt_write_rowcol( "Album Title/YY", 14, 12, 1 );
    UG_driver_txt_write_rowcol( "trk/title     ", 14, 12, 2 );
    UG_driver_txt_write_rowcol( "hh:mm.ss      ", 14, 12, 4 );

    /* init indexes */
    idx  = 0;
    idxo = 1;

    /* we use the bar font, chars ABCDE */
    for ( loop = 0; loop < 256; loop++ )
    {
        wiced_rtos_delay_milliseconds( 60 ); /* 15fps */

        /* swap level buffer */
        idxo = idx;
        idx  = ( idx + 1 ) & 0x01;

        /* generate two vectors for vu meter levels */
        for ( i = 0; i < 16; i++ )
        {
            tmp32 = 255 - loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i ] = ( 255 < tmp32 ) ? 255 : tmp32;

            tmp32 = loop + (uint8_t) ( rand( ) % 64 );
            lvl[ idx ][ i + 16 ] = ( 255 < tmp32 ) ? 255 : tmp32;
        }

        /* update bars, monocrome (colored would be slower) */
        for ( i = 0; i < 32; i++ )
        {
            // wiced_rtos_delay_milliseconds(60); /* 30fps */
            /* speed opt: only patch the missing part */
            if ( lvl[ idx ][ i ] > lvl[ idxo ][ i ] )
            {
                /* increasing the bar needs color steps (log)             */
                /* each color is a draw command .. don;t use too many     */
                /* bar max is 255/5 = 51 pixels                           */
                /* we use 4 colors, in log scale, so 28, 14, 7, 3 in size */

                tmp8 = lvl[ idx ][ i ] / 5;

                for ( k = 3; k >= 0; k-- )
                {
                    if ( ( tmp8 > vumeter_th[ k ] ) && ( tmp8 > lvl[ idxo ][ i ] / 5 ) )
                    {
                        tmp32 = ( lvl[ idxo ][ i ] / 5 > vumeter_th[ k ] ) ? lvl[ idxo ][ i ] / 5 : vumeter_th[ k ];
                        UG_driver_gfx_fill_rect( ( i * 5 ), 127 - tmp8, ( i * 5 + 3 ), 127 - tmp32, vumeter_color[ k ] );
                        tmp8 = vumeter_th[ k ];
                    }
                }
            }
            else if ( lvl[ idx ][ i ] < lvl[ idxo ][ i ] )
            {
                UG_driver_gfx_fill_rect( ( i * 5 ), 127 - lvl[ idxo ][ i ] / 5, ( i * 5 + 3 ), 127 - lvl[ idx ][ i ] / 5, C_BLACK );
            }
        }
    }

    wiced_rtos_delay_milliseconds( 2000 ); /* pause */
}


void test_image_01( uint16_t w, uint16_t h )
{
    uint8_t  x_r = ( rand( ) % w );
    uint8_t  y_r = ( rand( ) % h );

    uint32_t i = 0;

    /* mono colors */
    UG_driver_sys_clear_screen( );
    for ( i = 0; i < 300; i++ )
    {
        x_r = ( rand( ) % w );
        y_r = ( rand( ) % h );

        UG_driver_sys_set_forecolor( get_color_rainbow( ) );
        UG_driver_gfx_draw_image( x_r, y_r, UG_DRV_FMT_MONO, (uint8_t*) img_mono, 32, 32, NULL );
    }
    wiced_rtos_delay_milliseconds( 1500 );

    /* 256 colors */
    UG_driver_sys_clear_screen( );
    UG_driver_gfx_draw_image( 0, 0, UG_DRV_FMT_RGB332, (uint8_t*) img_rgb332, 160, 128, NULL );
    wiced_rtos_delay_milliseconds( 3000 );
    /* 65k colors */
    UG_driver_sys_clear_screen( );
    UG_driver_gfx_draw_image( 0, 0, UG_DRV_FMT_RGB565, (uint8_t*) img_rgb565, 160, 128, NULL );
    wiced_rtos_delay_milliseconds( 3000 );
    /* 262k/TrueColors */
    UG_driver_sys_clear_screen( );
    UG_driver_gfx_draw_image( 0, 0, UG_DRV_FMT_RGB888, (uint8_t*) img_rgb888, 160, 128, NULL );
    wiced_rtos_delay_milliseconds( 3000 );
}


void window_1_callback( UG_MESSAGE* msg )
{
    /* nothing to do */
}


void ugui_test_01( uint16_t w, uint16_t h )
{
    int16_t x = 0;
    int16_t y = 0;

    for ( x = 0; x < ( w - 2 ); x += 2 )
    {
        UG_DrawLine( 0, ( h - 1 ), x, 0, C_WHITE );
    }

    for ( y = 0; y < ( h - 2 ); y += 2 )
    {
        UG_DrawLine( ( w - 1 ), ( h - 1 ), 0, y, get_color_rainbow( ) );
    }

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */

    UG_FillScreen( C_RED );
    UG_SetBackcolor( C_BLACK );
    UG_SetForecolor( C_CYAN );
    UG_FontSelect( &FONT_8X8 );
    UG_PutString( w / 8, h / 4, "Hello World!" );

    UG_SetBackcolor( C_BLUE );
    UG_SetForecolor( C_GREEN );
    UG_FontSelect( &FONT_7X12 );
    UG_PutString( w / 10, h / 2, "Hello World!" );

    UG_SetBackcolor( C_YELLOW );
    UG_SetForecolor( C_BLACK );
    UG_FontSelect( &FONT_16X26 );
    UG_PutString( 0, h / 4 * 3, "Hello 123" );

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
    UG_FillScreen( C_RED );

    /* round frame test */
    for ( x = 0; x < ( w / 4 ); x += 2 )
    {
        UG_DrawRoundFrame( x, x, w / 2 - x, h - x, 10, C_GREEN );
        wiced_rtos_delay_milliseconds( 10 );
    }
    for ( x = 0; x < ( w / 4 ); x += 2 )
    {
        uint32_t c = ( ( x >> 1 ) & 0x01 ) ? C_RED : C_GREEN;
        UG_FillRoundFrame( w / 2 + x, x, w - x, h - x, 10, c );
        wiced_rtos_delay_milliseconds( 10 );
    }

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */

    UG_FillScreen( C_BLUE );

    UG_DrawMesh( w / 4, h / 4, w / 4 * 3, h / 4 * 3, C_WHITE );

    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */

    /* round frame test */
    for ( x = 0; x < ( w / 4 ); x += 2 )
    {
        UG_DrawCircle( w / 4, h / 4, x, C_WHITE );
        wiced_rtos_delay_milliseconds( 10 );
    }
    for ( x = 0; x < ( w / 4 ); x += 2 )
    {
        uint32_t c = ( ( x >> 1 ) & 0x01 ) ? C_RED : C_GREEN;
        UG_FillCircle( w / 4 * 3, h / 4 * 3, w / 4 - x, c );
        wiced_rtos_delay_milliseconds( 10 );
    }
    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG ); /* pause */
}


void ugui_test_02( )
{
    UG_WINDOW  window_1;
    UG_BUTTON  button_1;
    UG_BUTTON  button_2;
    UG_BUTTON  button_3;
    UG_TEXTBOX textbox_1;
    UG_OBJECT  obj_buff_wnd_1[ MAX_OBJECTS ];

    UG_driver_sys_clear_screen( );

    UG_WindowCreate( &window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback );
    // Modify the window title
    UG_WindowSetTitleTextFont( &window_1, &FONT_8X8 );
    UG_WindowSetTitleText( &window_1, "uGUI Digole" );

    UG_WindowShow( &window_1 );

    // Create some buttons
    UG_ButtonCreate( &window_1, &button_1, BTN_ID_0, 6, 6, 46, 26 );
    UG_ButtonSetForeColor( &window_1, BTN_ID_0, C_WHITE );
    UG_ButtonSetBackColor( &window_1, BTN_ID_0, C_DARK_GRAY );
    UG_ButtonSetFont( &window_1, BTN_ID_0, &FONT_5X8 );
    UG_ButtonSetText( &window_1, BTN_ID_0, "Setup" );
    UG_ButtonShow( &window_1, BTN_ID_0 );

    UG_ButtonCreate( &window_1, &button_2, BTN_ID_1, 6, 36, 46, 56 );
    UG_ButtonSetForeColor( &window_1, BTN_ID_1, C_WHITE );
    UG_ButtonSetBackColor( &window_1, BTN_ID_1, C_DARK_GRAY );
    UG_ButtonSetFont( &window_1, BTN_ID_1, &FONT_5X8 );
    UG_ButtonSetText( &window_1, BTN_ID_1, "File" );
    UG_ButtonShow( &window_1, BTN_ID_0 );

    UG_ButtonCreate( &window_1, &button_3, BTN_ID_2, 6, 66, 46, 86 );
    UG_ButtonSetForeColor( &window_1, BTN_ID_2, C_BLACK );
    UG_ButtonSetBackColor( &window_1, BTN_ID_2, C_YELLOW );
    UG_ButtonSetFont( &window_1, BTN_ID_2, &FONT_5X8 );
    UG_ButtonSetText( &window_1, BTN_ID_2, "Exit" );
    UG_ButtonShow( &window_1, BTN_ID_0 );

    // Create a Textbox /
    UG_TextboxCreate( &window_1, &textbox_1, TXB_ID_0, 55, 20, 150, 100 );
    UG_TextboxSetFont( &window_1, TXB_ID_0, &FONT_5X8 );
    UG_TextboxSetForeColor( &window_1, TXB_ID_0, C_BLACK );
    UG_TextboxSetAlignment( &window_1, TXB_ID_0, ALIGN_CENTER );
    UG_TextboxSetText( &window_1, TXB_ID_0, "This is just \na very simple\n window \n uGUI ! " );

    UG_WindowShow( &window_1 );
    UG_Update( );

    wiced_rtos_delay_milliseconds( 5000 );

    /* resize window */
    UG_WindowResize( &window_1, 20, 20, 160 - 20, 128 - 20 );
    UG_Update( );

    wiced_rtos_delay_milliseconds( 5000 );

    /* move window : moving is a fake resize (hw accell) */
    UG_WindowResize( &window_1, 20 - 18, 20 - 18, 160 - 20 - 18, 128 - 20 - 18 );
    UG_Update( );
}
