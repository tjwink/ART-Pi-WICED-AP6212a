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

/* display config */
#include "display_cfg_ssd1306.h"


/******************************************************
 *                      Macros
 ******************************************************/
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
void ugui_scroll_fullscreen( uint16_t w, uint16_t h );
void ugui_scroll_window( uint16_t w, uint16_t h );

/******************************************************
 *               Variables Definitions
 ******************************************************/

UG_GUI   gui;     // Global GUI obj
UG_BUS_T gui_bus; // Global BUS obj
UG_DRV_T gui_drv; // Global DRIVER obj


/******************************************************
 *               Function Definitions
 ******************************************************/


/******************************************************
 *               MAIN
 ******************************************************/
void application_start( )
{
    UG_BUS_ERR bus_err;
    UG_DRV_ERR drv_err;

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
        while ( 1 )
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
        while ( 1 )
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
    UG_driver_pwr_set_sleep( 0 );

    UG_driver_sys_clear_screen( );

    /*
     * uGUI: create one gui obj
     */
    UG_Init( &gui, UG_driver_gfx_pixel_set, DISPLAY_W_PX, DISPLAY_H_PX );
    UG_SelectGUI( &gui );

    /*
     * uGUI: register driver hw accelerated primitives
     * note: this is a macro declared into each
     *       display include file at the top.
     */
    ENABLE_UG_HW_DRIVER( );

    /*
     * GUI: window example
     */

    /* info message */
    UG_driver_sys_clear_screen( );

    UG_SetBackcolor( C_BLACK );
    UG_SetForecolor( C_WHITE );
    UG_FontSelect( &FONT_8X8 );

    UG_PutString( ( DISPLAY_W_PX / 20 ), ( DISPLAY_H_PX / 8 ),     "This test is" );
    UG_PutString( ( DISPLAY_W_PX / 20 ), ( DISPLAY_H_PX / 8 ) * 2, "specific for" );
    UG_PutString( ( DISPLAY_W_PX / 20 ), ( DISPLAY_H_PX / 8 ) * 3, "SS1306 scroll" );
    UG_PutString( ( DISPLAY_W_PX / 20 ), ( DISPLAY_H_PX / 8 ) * 4, "(sw version)" );
    UG_Update( );
    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG * 2 );

    /*
     * full screen scroll example
     */
    ugui_scroll_fullscreen( DISPLAY_W_PX, DISPLAY_H_PX );

    /*
     * inner window scroll example
     */
    ugui_scroll_window( DISPLAY_W_PX, DISPLAY_H_PX );

    /*
     * release the driver, bus and exit
     */
    UG_driver_deinit( &drv_err, &gui_drv );
    UG_bus_deinit( &bus_err, &gui_bus );

    printf( "bye bye.\n" );
}


/******************************************************
 *               Tests
 ******************************************************/
void ugui_scroll_fullscreen( uint16_t w, uint16_t h )
{
    UG_SetBackcolor( C_BLACK );
    UG_SetForecolor( C_WHITE );
    UG_FontSelect( &FONT_8X8 );

    /* info message */
    UG_driver_sys_clear_screen( );
    UG_PutString( ( w / 20 ), ( h / 8 ) * 2, "  SCROLLING " );
    UG_PutString( ( w / 20 ), ( h / 8 ) * 3, " FULL SCREEN" );
    UG_Update( );
    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG / 2 );

    /* horizontal scroll : RIGHT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello RIGHT!" );
    UG_Update( );
    /* scroll the whole screen from (0,0) to (127,64) */
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_RIGHT,
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );


    /* horizontal scroll : LEFT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello LEFT!" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_LEFT,
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );


    /* vertical scroll : UP */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello UP!" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_UP,
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    /* vertical scroll : DOWN */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello DOWN!" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_DOWN,
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );



    /* vertical scroll : UP+LEFT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello UP+LX!" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( ( UG_DRV_SCROLL_UP | UG_DRV_SCROLL_LEFT ),
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    /* vertical scroll : DOWN+RIGHT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( w / 10, h / 2, "Hello DW+RX!" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( ( UG_DRV_SCROLL_DOWN | UG_DRV_SCROLL_RIGHT ),
                                       250, 250,                                 /* speed_x, speed_y   */
                                       DISPLAY_W_PX, DISPLAY_H_PX,               /* scroll_x, scroll_y */
                                       0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, /* (x1,y1), (x2,y2)   */
                                       0,                                        /* circular_f         */
                                       NULL );                                   /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    UG_driver_sys_clear_screen( );
}


void ugui_scroll_window( uint16_t w, uint16_t h )
{
    UG_SetBackcolor( C_BLACK );
    UG_SetForecolor( C_WHITE );
    UG_FontSelect( &FONT_8X8 );

    /* info message */
    UG_driver_sys_clear_screen( );
    UG_PutString( ( w / 20 ), ( h / 8 ) * 2, "  SCROLLING " );
    UG_PutString( ( w / 20 ), ( h / 8 ) * 3, "INNER WINDOW" );
    UG_PutString( ( w / 20 ), ( h / 8 ) * 4, "    ONLY" );
    UG_Update( );
    wiced_rtos_delay_milliseconds( TEST_PAUSE_LONG / 2 );

    /* horizontal scroll : RIGHT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), h / 2, "--RIGHT--" );
    UG_Update( );
    /* scroll the whole screen from (0,0) to (127,64) */
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_RIGHT,
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );


    /* horizontal scroll : LEFT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), h / 2, "--LEFT---" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_LEFT,
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );


    /* vertical scroll : UP */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 16 ), "---UP----" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_UP,
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    /* vertical scroll : DOWN */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), ( h / 8 + 10 ), "--DOWN---" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( UG_DRV_SCROLL_DOWN,
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );


    /* vertical scroll : UP+LEFT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 16 ), "--UP+LX--" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( ( UG_DRV_SCROLL_UP | UG_DRV_SCROLL_LEFT ),
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    /* vertical scroll : DOWN+RIGHT */
    UG_driver_sys_clear_screen( );
    UG_DrawRoundFrame( 0, 0, DISPLAY_W_PX - 1, DISPLAY_H_PX - 1, DISPLAY_H_PX / 4, C_WHITE );
    UG_DrawFrame( DISPLAY_W_PX / 8 - 1, DISPLAY_H_PX / 8 - 1, ( DISPLAY_W_PX * 7 ) / 8 + 1, ( DISPLAY_H_PX * 7 ) / 8 + 1, C_WHITE );
    UG_FontSelect( &FONT_8X14 );
    UG_PutString( ( w / 8 + 1 ), ( h / 8 + 10 ), "--DW+RX--" );
    UG_Update( );
    UG_driver_sys_screen_scroll_start( ( UG_DRV_SCROLL_DOWN | UG_DRV_SCROLL_RIGHT ),
                                       250, 250,                                                   /* speed_x, speed_y   */
                                       DISPLAY_W_PX * 6 / 8, DISPLAY_H_PX * 6 / 8,                 /* scroll_x, scroll_y */
                                       ( w / 8 + 1 ), ( h / 8 + 1 ),                               /* (x1,y1)            */
                                       DISPLAY_W_PX - ( w / 8 + 1 ), DISPLAY_H_PX - ( h / 8 + 1 ), /* (x2,y2)            */
                                       0,                                                          /* circular_f         */
                                       NULL );                                                     /* usr_data           */
    UG_driver_sys_screen_scroll_stop( );

    UG_driver_sys_clear_screen( );
}
