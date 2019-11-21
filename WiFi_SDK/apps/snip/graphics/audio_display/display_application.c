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

/** @file Audio Display Test App
 *
 * Audio Display Sample Application
 *
 * Features demonstrated:
 * - How to use the Audio Display library
 *
 * On startup this demo:
 * - Creates the audio display management thread
 * - Updates the screen to simulate use
 * - Demonstrates several options of the audio display library
 *
 * Application Instructions:
 * - Plug ssd1306_128x64_i2c_oled display into "display" header on BCM943907WAE_1 board
 *     - This uses I2C_1
 * - Power up the board, then build and download the application
 * - The display should cycle through several scenes
 *     - Header contains: signal strength, bluetooth, battery
 *     - Footer contains: song information or status messages
 *
 * General Audio Display Notes:
 * - The management thread updates battery and signal strength automatically
 *     - Battery is updated once per second, signal strength once every three seconds
 *     - User must update options for signal strength and battery
 * - Thread handles all of the drawing - user just needs to update icons
 *
 */

#include "audio_display.h"

/******************************************************
 *                      Macros
 ******************************************************/

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start()
{
    wiced_thread_t thread;

    audio_display_create_management_thread(&thread, WICED_AUDIO_DISPLAY_ENABLE_WIFI);

    while (1)
    {
        /* Display header icons - Signal strength, bluetooth, battery */
        audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT);
        u8g_Delay(3000);
        audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT | SIGNAL_STRENGTH_IS_VISIBLE);
        u8g_Delay(3000);
        audio_display_header_update_bluetooth(BLUETOOTH_IS_CONNECTED);
        u8g_Delay(5000);

        /* Song information can be updated all at once. */
        audio_display_update_footer("Song Title", "Artist", 34, 124, FOOTER_IS_VISIBLE | FOOTER_CENTER_ALIGN);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(35, 124);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(36, 124);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(37, 124);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(38, 124);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(39, 124);
        u8g_Delay(3000);

        /* Song information can also be updated separately */
        audio_display_footer_update_song_info("New Song", "New Artist");
        audio_display_footer_update_time_info(0, 234);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(1, 234);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(2, 234);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(3, 234);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(4, 234);
        u8g_Delay(1000);
        audio_display_footer_update_time_info(5, 234);
        u8g_Delay(3000);

        /* Song duration can be toggled off.
         * The battery charging icon and bluetooth connected icon can change colors.
         */
        audio_display_footer_update_song_info("Battery and Bluetooth", "Colors can be toggled");
        audio_display_footer_update_options(FOOTER_IS_VISIBLE | FOOTER_CENTER_ALIGN | FOOTER_HIDE_SONG_DURATION);
        u8g_Delay(3000);
        audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT | INVERT_BATTERY_CHARGING_COLORS | BLUETOOTH_IS_CONNECTED | INVERT_BLUETOOTH_ICON_COLORS | SIGNAL_STRENGTH_IS_VISIBLE);
        u8g_Delay(5000);

        /* All icons can be hidden or made visible. */
        audio_display_footer_update_options(0x00);
        audio_display_header_update_options(0x00);
        u8g_Delay(1000);
    }
}
