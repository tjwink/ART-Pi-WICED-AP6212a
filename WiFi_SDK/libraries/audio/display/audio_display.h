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

/** @file Apollo Display Library Header
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AUDIO_DISPLAY_H

#include "u8g_arm.h"
#include "power_management.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_DISPLAY_H

/* Header options - Battery, Bluetooth, RSSI */
#define BATTERY_ICON_IS_VISIBLE        0x01
#define BATTERY_ICON_SHOW_PERCENT      0x02
#define INVERT_BATTERY_CHARGING_COLORS 0x04
#define BATTERY_ICON_IS_CHARGING       0x08 /* The update function automatically sets/clears this flag for internal use */
#define BLUETOOTH_IS_CONNECTED         0x10
#define INVERT_BLUETOOTH_ICON_COLORS   0x20
#define SIGNAL_STRENGTH_IS_VISIBLE     0x40
#define HEADER_BOTTOM_BAR_IS_VISIBLE   0x80

/* Footer options - Song information appearance */
#define FOOTER_IS_VISIBLE              0x01
#define FOOTER_CENTER_ALIGN            0x02
#define FOOTER_TOP_BAR_IS_VISIBLE      0x04
#define FOOTER_HIDE_SONG_DURATION      0x08
#define FOOTER_OPTION_APOLLO_TX        0x10
#define FOOTER_OPTION_APOLLO_RX        0x20


/******************************************************
 *                   Enumerations
 ******************************************************/

/* flags for how the audio display is initialized
 *
 * DEFAULT Functionality includes
 *         Continually updates when options are set to display (call audio_display_header_update_options() to manage)
 *             RSSI display (signal strength) - always 0 unless ENABLE_WIFI init flag is used
 *             Bluetooth connectivity icon
 *             Battery level icon
 *             Song Tile and Artist info
 *
 * ENABLE_WIFI - enables calling wiced_wlan_connectivity_init() to get signal strength
 *             - continually updates:
 *                 - RSSI (signal strength) display
 *                 - if FOOTER_OPTION_APOLLO_RX or FOOTER_OPTION_APOLLO_TX
 *                  - update song time info
 */
#define WICED_AUDIO_DISPLAY_ENABLE_DEFAULT    (      0 )
#define WICED_AUDIO_DISPLAY_ENABLE_WIFI       ( 1 << 0 )

typedef uint32_t    wiced_audio_display_init_flags_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Creates a thread to handle the drawing and state management of the display.
 *
 * The thread keeps the battery updated, but the battery options can still be changed by the user.
 * This thread also updates signal strength while allowing the user to change the options.
 * Battery is updated twice per second; signal strength is updated once every three seconds.
 *
 * All other icons must be updated by the user using the audio display API update functions.
 *
 * Note: Updating options overwrites previously set options.
 */
wiced_result_t audio_display_create_management_thread(wiced_thread_t* thread, wiced_audio_display_init_flags_t flags);

/* Initializes the display settings and icons */
wiced_result_t audio_display_init(u8g_t* u8g_copy);

/* set the optional bux mutex for bus access serialization */
wiced_result_t audio_display_set_bus_mtx(wiced_mutex_t *bus_mtx);

/* Header update functions */
void audio_display_header_update_battery(power_management_status_t* battery_status, uint8_t options);
void audio_display_header_update_bluetooth(uint8_t options);
void audio_display_header_update_signal_strength(int32_t rssi, uint8_t options);
void audio_display_header_update_options(uint8_t options);
/* Header draw functions */
void audio_display_header_draw_battery();
void audio_display_header_draw_bluetooth();
void audio_display_header_draw_signal_strength();
/* Complete Header functions */
void audio_display_update_header(power_management_status_t* battery_status, int32_t rssi, uint8_t options);
void audio_display_draw_header();

/* Footer update functions */
void audio_display_footer_update_song_info(char* title, char* artist);
void audio_display_footer_update_time_info(uint32_t current_time, uint32_t duraiton);
void audio_display_footer_update_options(uint8_t options);
uint8_t audio_display_get_footer_options(void);
/* Complete Footer functions */
void audio_display_update_footer(char* title, char* artist, uint32_t current_time, uint32_t duration, uint8_t options);
void audio_display_draw_footer();

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
