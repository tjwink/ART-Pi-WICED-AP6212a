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

#include "audio_display.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define SPACE_CHAR_STR " "

/******************************************************
 *                    Constants
 ******************************************************/

#define BATTERY_ICON_SOC_PER_INTERIOR_PIXEL 6.25 /* Battery interior is 16 pixels wide, 100% / 16pixels = 6.25 percent per pixel */
#define BATTERY_ICON_PERCENT_STRING_LENGTH  5    /* SOC percent string includes SOC [0, 100], '%' character, and null character */

#define SIGNAL_STRENGTH_MAX_BARS            5    /* The number of bars for the best signal strength */
#define SIGNAL_STRENGTH_STARTING_HEIGHT     3    /* The height in pixels of the first (left-most) bar of signal strength */
#define SIGNAL_STRENGTH_HEIGHT_INCREMENT    2    /* Height difference in pixels between each bar */
#define SIGNAL_STRENGTH_BAR_WIDTH           3    /* Width of each bar in pixels (there is a 1 pixel gap between bars for definition */

#define SONG_DETAILS_BORDER_PIXELS          3    /* Pixels between top of one string and bottom of another string in footer */
#define SONG_DETAILS_DURATION_STRLEN        18   /* Song duration must be less than 1000 minutes */
#define SONG_DETAILS_STRING_SIZE_LIMIT      32   /* Max number of characters that will fit on the screen with the current font */

#define DISPLAY_MANAGEMENT_THREAD_SIZE      5000 /* Stack size for the display management thread - library uses snprintf() */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint8_t x;
    uint8_t y;
} coordinates_t;

typedef struct
{
    const uint8_t* bitmap;
    uint8_t        width;
    uint8_t        height;
} generic_bitmap_t;

typedef struct
{
    generic_bitmap_t bitmap;
    coordinates_t    position;
} generic_icon_t;

typedef struct
{
    generic_icon_t icon;
    coordinates_t  charge_position; /* Interior rectangle X and Y coordinates */
    uint8_t        charge_width;    /* [0,16] */
    uint8_t        state_of_charge;
    uint8_t        options;
} battery_icon_t;

typedef struct
{
    generic_icon_t icon;
    uint8_t        options;
} bluetooth_icon_t;

typedef struct
{
    generic_icon_t icon;
    uint8_t        bar_count;
    uint8_t        options;
} signal_strength_icon_t;

typedef struct
{
    char     title[SONG_DETAILS_STRING_SIZE_LIMIT];
    char     artist[SONG_DETAILS_STRING_SIZE_LIMIT];
    uint16_t current_time;
    uint16_t duration;
} song_details_t;

typedef struct
{
    song_details_t song;
    coordinates_t  title_position;
    coordinates_t  artist_position;
    coordinates_t  duration_position;
    char           duration_string[SONG_DETAILS_DURATION_STRLEN];
    uint8_t        options;
} audio_footer_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

static void audio_display_update_icon(generic_icon_t* icon, const generic_bitmap_t* map,  uint8_t x, uint8_t y);
static void audio_display_draw_icon(generic_icon_t* icon);
static uint8_t audio_display_header_rssi_to_bars(int32_t rssi);
static void audio_display_header_draw_battery_percent();
static void audio_display_main(uint32_t arg);

/******************************************************
 *               Variables Definitions
 ******************************************************/

const static uint8_t battery_shell_bitmap[] =
{
    0x7F, 0xFF, 0xF8,// | ####################   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x40, 0x00, 0x08,// | # |Battery       | #   |
    0x40, 0x00, 0x0F,// | # |interior or   | ####|
    0x40, 0x00, 0x09,// | # |"charge" of   | #  #|
    0x40, 0x00, 0x09,// | # |battery.      | #  #|
    0x40, 0x00, 0x0F,// | # |Filled in     | ####|
    0x40, 0x00, 0x08,// | # |draw_battery()| #   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x7F, 0xFF, 0xF8,// | ####################   |
};

const static uint8_t inverted_battery_charging_bitmap[] =
{
    0x7F, 0xFF, 0xF8,// | ####################   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x5F, 0xFF, 0xE8,// | # ################ #   |
    0x5E, 0x1E, 0x2F,// | # ####    ####   # ####|
    0x5C, 0x4C, 0x69,// | # ###   #  ##   ## #  #|
    0x58, 0xC8, 0xE9,// | # ##   ##  #   ### #  #|
    0x51, 0xE1, 0xEF,// | # #   ####    #### ####|
    0x5F, 0xFF, 0xE8,// | # ################ #   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x7F, 0xFF, 0xF8,// | ####################   |
};

const static uint8_t battery_charging_bitmap[] =
{
    0x7F, 0xFF, 0xF8,// | ####################   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x40, 0xC0, 0x68,// | #      ##       ## #   |
    0x41, 0xE0, 0xCF,// | #     ####     ##  ####|
    0x43, 0x71, 0x89,// | #    ## ###   ##   #  #|
    0x46, 0x3B, 0x09,// | #   ##   ### ##    #  #|
    0x4C, 0x1E, 0x0F,// | #  ##     ####     ####|
    0x58, 0x0C, 0x08,// | # ##       ##      #   |
    0x40, 0x00, 0x08,// | #                  #   |
    0x7F, 0xFF, 0xF8,// | ####################   |
};

const static uint8_t inverted_bluetooth_connected_bitmap[] =
{
    0x03, 0xC0,// |      ####      |
    0x0E, 0x70,// |    ###  ###    |
    0x1E, 0xB8,// |   #### # ###   |
    0x36, 0xDC,// |  ## ## ## ###  |
    0x7A, 0x3E,// | #### #   ##### |
    0x7C, 0x7E,// | #####   ###### |
    0x7A, 0x3E,// | #### #   ##### |
    0x36, 0xDC,// |  ## ## ## ###  |
    0x1E, 0xB8,// |   #### # ###   |
    0x0E, 0x70,// |    ###  ###    |
    0x03, 0xC0,// |      ####      |
};

const static uint8_t bluetooth_connected_bitmap[] =
{
    0x03, 0x00,// |      ##        |
    0x03, 0xC0,// |      ####      |
    0x33, 0x70,// |  ##  ## ###    |
    0x1B, 0x38,// |   ## ##  ###   |
    0x0F, 0x70,// |    #### ###    |
    0x03, 0xC0,// |      ####      |
    0x0F, 0x70,// |    #### ###    |
    0x1B, 0x38,// |   ## ##  ###   |
    0x33, 0x70,// |  ##  ## ###    |
    0x03, 0xC0,// |      ####      |
    0x03, 0x00,// |      ##        |
};

static u8g_t* u8g = NULL;

static battery_icon_t         audio_battery;
static bluetooth_icon_t       audio_bluetooth;
static signal_strength_icon_t audio_signal_strength;
static audio_footer_t         audio_footer;
static int32_t g_rssi = 0;
static uint32_t g_channel = 0;

const static generic_bitmap_t battery_shell =
{
    .bitmap = battery_shell_bitmap,
    .width  = 24,
    .height = 10,
};

const static generic_bitmap_t battery_charging =
{
    .bitmap = battery_charging_bitmap,
    .width  = 24,
    .height = 10,
};

const static generic_bitmap_t bluetooth_connected =
{
    .bitmap = bluetooth_connected_bitmap,
    .width  = 16,
    .height = 11,
};

const static generic_bitmap_t inverted_battery_charging =
{
    .bitmap = inverted_battery_charging_bitmap,
    .width  = 24,
    .height = 10,
};

const static generic_bitmap_t inverted_bluetooth_connected =
{
    .bitmap = inverted_bluetooth_connected_bitmap,
    .width  = 16,
    .height = 11,
};

/* neeed to serialize I2C bus access when other devices */
/* are connected on the same bus */

static wiced_mutex_t *audio_display_bus_mtx = NULL;
#define BUS_LOCK_MAX (5)


/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t audio_display_init(u8g_t* u8g_copy)
{
    if (u8g_copy != NULL)
    {
        u8g = u8g_copy;

        u8g_SetFont(u8g, u8g_font_6x10);
        u8g_SetFontPosTop(u8g);

        audio_display_update_header(NULL, 0, 0);
        audio_display_update_footer("", "", 0, 0, 0);

        return WICED_SUCCESS;
    }

    return WICED_BADARG;
}

wiced_result_t audio_display_set_bus_mtx(wiced_mutex_t *bus_mtx)
{

    if ( (u8g != NULL) && (bus_mtx !=NULL) )
    {
        audio_display_bus_mtx = bus_mtx;

        return WICED_SUCCESS;
    }

    return WICED_BADARG;

}

static void audio_display_update_icon(generic_icon_t* icon, const generic_bitmap_t* map,  uint8_t x, uint8_t y)
{
    if (map != NULL)
    {
        icon->bitmap = *map;
    }
    icon->position.x = x;
    icon->position.y = y;
}

static void audio_display_draw_icon(generic_icon_t* icon)
{
    u8g_DrawBitmap(u8g, icon->position.x, icon->position.y, icon->bitmap.width / 8, icon->bitmap.height, icon->bitmap.bitmap);
}

static uint8_t audio_display_header_rssi_to_bars(int32_t rssi)
{
    uint8_t bars;

    if (rssi >= 0)
    {
        bars = 0;
    }
    else if (rssi >= -32)
    {
        bars = 5;
    }
    else if (rssi >= -44)
    {
        bars = 4;
    }
    else if (rssi >= -56)
    {
        bars = 3;
    }
    else if (rssi >= -68)
    {
        bars = 2;
    }
    else if (rssi >= -80)
    {
        bars = 1;
    }
    else
    {
        bars = 0;
    }

    return bars;
}

void audio_display_header_update_signal_strength(int32_t rssi, uint8_t options)
{
    audio_signal_strength.options = options;

    if (rssi >= 0 && rssi <= 5)
    {
        audio_signal_strength.bar_count = rssi;
    }
    else
    {
        audio_signal_strength.bar_count = audio_display_header_rssi_to_bars(rssi);
    }

    audio_display_update_icon(&audio_signal_strength.icon, NULL, 0, (SIGNAL_STRENGTH_MAX_BARS - 1) * SIGNAL_STRENGTH_HEIGHT_INCREMENT);
}

void audio_display_header_update_bluetooth(uint8_t options)
{
    audio_bluetooth.options = options;

    if (audio_bluetooth.options & INVERT_BLUETOOTH_ICON_COLORS)
    {
        audio_display_update_icon(&audio_bluetooth.icon, &inverted_bluetooth_connected, (u8g->width / 2) - (bluetooth_connected.width / 2), audio_battery.icon.position.y);
    }
    else
    {
        audio_display_update_icon(&audio_bluetooth.icon, &bluetooth_connected, (u8g->width / 2) - (bluetooth_connected.width / 2), audio_battery.icon.position.y);
    }
}

/* A valid power_management_status_t struct is obtained by calling power_management_update() */
void audio_display_header_update_battery(power_management_status_t* battery_status, uint8_t options)
{
    audio_battery.options = options;

    if (battery_status == NULL)
    {
        return;
    }

    if (battery_status->state_of_charge > 100)
    {
        audio_battery.state_of_charge = 100;
    }
    else
    {
        audio_battery.state_of_charge = battery_status->state_of_charge;
    }

    /* Calculates width of interior "charge" bar to represent SOC */
    audio_battery.charge_width = (uint8_t) ceil(battery_status->state_of_charge / BATTERY_ICON_SOC_PER_INTERIOR_PIXEL);
    if (audio_battery.charge_width > 16)
    {
        audio_battery.charge_width = 16;
    }

    /* Check whether or not battery is charging - update flags and bitmap accordingly */
    if (battery_status->return_flags & CHARGER_RETURN_FAST_CHARGING || battery_status->return_flags & CHARGER_RETURN_TOPPING_OFF)
    {
        audio_battery.options |= BATTERY_ICON_IS_CHARGING;
        if (audio_battery.options & INVERT_BATTERY_CHARGING_COLORS)
        {
            audio_display_update_icon(&audio_battery.icon, &inverted_battery_charging, u8g->width - battery_charging.width, 0);
        }
        else
        {
            audio_display_update_icon(&audio_battery.icon, &battery_charging, u8g->width - battery_charging.width, 0);
        }
    }
    else
    {
        audio_battery.options &= ~BATTERY_ICON_IS_CHARGING;
        audio_display_update_icon(&audio_battery.icon, &battery_shell, u8g->width - battery_shell.width, 0);
    }

    audio_battery.charge_position.x = audio_battery.icon.position.x + 3;
    audio_battery.charge_position.y = audio_battery.icon.position.y + 2;
}

void audio_display_header_update_options(uint8_t options)
{
    audio_display_update_header(NULL, audio_signal_strength.bar_count, options);
}

void audio_display_update_header(power_management_status_t* battery_status, int32_t rssi, uint8_t options)
{
    audio_display_header_update_battery(battery_status, options);
    audio_display_header_update_bluetooth(options);
    audio_display_header_update_signal_strength(rssi, options);
}

void audio_display_footer_update_song_info(char* title, char* artist)
{
    if ( (title != NULL) && (title[0] != '\0') )
    {
        strlcpy(audio_footer.song.title, title, sizeof(audio_footer.song.title));
    }
    else
    {
        strlcpy(audio_footer.song.title, SPACE_CHAR_STR, sizeof(audio_footer.song.title));
    }

    if ( (artist != NULL) && (artist[0] != '\0') )
    {
        strlcpy(audio_footer.song.artist, artist, sizeof(audio_footer.song.artist));
    }
    else
    {
        strlcpy(audio_footer.song.artist, SPACE_CHAR_STR, sizeof(audio_footer.song.artist));
    }

    if (audio_footer.options & FOOTER_CENTER_ALIGN)
    {
        audio_footer.artist_position.x = (u8g->width / 2) - (uint8_t) ceil(strnlen(audio_footer.song.artist, SONG_DETAILS_STRING_SIZE_LIMIT) / 2.0 * u8g_GetFontBBXWidth(u8g));
        audio_footer.title_position.x = (u8g->width / 2) - (uint8_t) ceil(strnlen(audio_footer.song.title, SONG_DETAILS_STRING_SIZE_LIMIT) / 2.0 * u8g_GetFontBBXWidth(u8g));
    }
    else
    {
        audio_footer.artist_position.x = audio_footer.title_position.x = 0;
    }

    audio_footer.artist_position.y = u8g->height - u8g_GetFontBBXHeight(u8g) - SONG_DETAILS_BORDER_PIXELS - u8g_GetFontBBXHeight(u8g);
    audio_footer.title_position.y = audio_footer.artist_position.y - SONG_DETAILS_BORDER_PIXELS - u8g_GetFontBBXHeight(u8g);
}

uint8_t audio_display_get_footer_options(void)
{
    return audio_footer.options;
}

void audio_display_format_time_info(void)
{
    if (audio_footer.options & FOOTER_OPTION_APOLLO_TX) {
        snprintf(audio_footer.duration_string, SONG_DETAILS_DURATION_STRLEN, "%ld %ld - %d TX", g_channel, g_rssi, audio_footer.song.current_time);
    }
    else if (audio_footer.options & FOOTER_OPTION_APOLLO_RX) {
        snprintf(audio_footer.duration_string, SONG_DETAILS_DURATION_STRLEN, "%ld %ld  %d/%d", g_channel, g_rssi, audio_footer.song.current_time, audio_footer.song.duration);
    } else {
        snprintf(audio_footer.duration_string, SONG_DETAILS_DURATION_STRLEN, "%d:%02d/%d:%02d", audio_footer.song.current_time / 60, audio_footer.song.current_time % 60, audio_footer.song.duration / 60, audio_footer.song.duration % 60);
    }
}
void audio_display_footer_update_time_info(uint32_t current_time, uint32_t duration)
{
    audio_footer.song.current_time = current_time;
    audio_footer.song.duration = duration;
    audio_display_format_time_info();
    if (audio_footer.options & FOOTER_CENTER_ALIGN)
    {
        audio_footer.duration_position.x = (u8g->width / 2) - (uint8_t) ceil(strnlen(audio_footer.duration_string, SONG_DETAILS_DURATION_STRLEN) / 2.0 * u8g_GetFontBBXWidth(u8g));
    }
    else
    {
        audio_footer.duration_position.x = 0;
    }

    audio_footer.duration_position.y = u8g->height - u8g_GetFontBBXHeight(u8g);
}

void audio_display_footer_update_options(uint8_t options)
{
    audio_display_update_footer(audio_footer.song.title, audio_footer.song.artist, audio_footer.song.current_time, audio_footer.song.duration, options);
}

void audio_display_update_footer(char* title, char* artist, uint32_t current_time, uint32_t duration, uint8_t options)
{
    audio_footer.options = options;
    audio_display_footer_update_song_info(title, artist);
    audio_display_footer_update_time_info(current_time, duration);
}

static void audio_display_header_draw_battery_percent()
{
    char percent_string[BATTERY_ICON_PERCENT_STRING_LENGTH];
    uint8_t percent_offset = 0;

    snprintf(percent_string, BATTERY_ICON_PERCENT_STRING_LENGTH, "%d%c", audio_battery.state_of_charge, '%');

    /* Account for different number of characters to print [2,4]
     * depending on the current state of charge. */
    if (audio_battery.state_of_charge < 10)
    {
        percent_offset = 2;
    }
    else if (audio_battery.state_of_charge < 100)
    {
        percent_offset = 3;
    }
    else
    {
        percent_offset = 4;
    }

    percent_offset *= u8g_GetFontBBXWidth(u8g);
    u8g_DrawStr(u8g, audio_battery.icon.position.x - percent_offset, audio_battery.icon.position.y, percent_string);
}

void audio_display_header_draw_signal_strength()
{
    uint8_t bars_drawn = 0;

    if (audio_signal_strength.options & SIGNAL_STRENGTH_IS_VISIBLE)
    {
        /* Draws small base bars to visually identify max number of bars */
        for (bars_drawn = 0; bars_drawn < SIGNAL_STRENGTH_MAX_BARS; ++bars_drawn)
        {
            u8g_DrawBox(u8g, audio_signal_strength.icon.position.x + (bars_drawn * (SIGNAL_STRENGTH_BAR_WIDTH + 1)),
             audio_signal_strength.icon.position.y + SIGNAL_STRENGTH_STARTING_HEIGHT - 1, SIGNAL_STRENGTH_BAR_WIDTH, 1);
        }

        /* Draws actual signal strength bars */
        for (bars_drawn = 0; bars_drawn < audio_signal_strength.bar_count; ++bars_drawn)
        {
            u8g_DrawBox(u8g, audio_signal_strength.icon.position.x + (bars_drawn * (SIGNAL_STRENGTH_BAR_WIDTH + 1)), audio_signal_strength.icon.position.y - (bars_drawn * SIGNAL_STRENGTH_HEIGHT_INCREMENT),
             SIGNAL_STRENGTH_BAR_WIDTH, SIGNAL_STRENGTH_STARTING_HEIGHT + (bars_drawn * SIGNAL_STRENGTH_HEIGHT_INCREMENT));
        }
    }
}

void audio_display_header_draw_bluetooth()
{
    if (audio_bluetooth.options & BLUETOOTH_IS_CONNECTED)
    {
        audio_display_draw_icon(&audio_bluetooth.icon);
    }
}

void audio_display_header_draw_battery()
{
    if (audio_battery.options & BATTERY_ICON_IS_VISIBLE)
    {
        if (audio_battery.options & BATTERY_ICON_SHOW_PERCENT)
        {
            audio_display_header_draw_battery_percent();
        }

        audio_display_draw_icon(&audio_battery.icon);

        if (!(audio_battery.options & BATTERY_ICON_IS_CHARGING))
        {
            u8g_DrawBox(u8g, audio_battery.charge_position.x, audio_battery.charge_position.y, audio_battery.charge_width, audio_battery.icon.bitmap.height - 4);
        }
    }
}

void audio_display_draw_header()
{
    audio_display_header_draw_signal_strength();
    audio_display_header_draw_bluetooth();
    audio_display_header_draw_battery();

    if (audio_battery.options & HEADER_BOTTOM_BAR_IS_VISIBLE)
    {
        u8g_DrawLine(u8g, 0, audio_bluetooth.icon.bitmap.height + 1, u8g->width, audio_bluetooth.icon.bitmap.height + 1);
    }
}

void audio_display_draw_footer()
{
    if (audio_footer.options & FOOTER_IS_VISIBLE)
    {
        /* Creates the rows for title, artist, and duration from top to bottom of screen */
        if (audio_footer.options & FOOTER_TOP_BAR_IS_VISIBLE)
        {
            u8g_DrawLine(u8g, 0, audio_footer.title_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0), u8g->width, audio_footer.title_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0));
        }
        u8g_DrawLine(u8g, 0, audio_footer.artist_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0), u8g->width, audio_footer.artist_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0));
        u8g_DrawLine(u8g, 0, audio_footer.duration_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0), u8g->width, audio_footer.duration_position.y - ceil(SONG_DETAILS_BORDER_PIXELS / 2.0));

        /* Draws the text for title, artist, and duration from top to bottom of screen */
        u8g_DrawStr(u8g, audio_footer.title_position.x, audio_footer.title_position.y, audio_footer.song.title);
        u8g_DrawStr(u8g, audio_footer.artist_position.x, audio_footer.artist_position.y, audio_footer.song.artist);

        if ( !(audio_footer.options & FOOTER_HIDE_SONG_DURATION) )
        {
            u8g_DrawStr(u8g, audio_footer.duration_position.x, audio_footer.duration_position.y, audio_footer.duration_string);
        }
    }
}

static void audio_display_main(uint32_t arg)
{
    wiced_result_t  rv = WICED_SUCCESS;

    u8g_t thread_u8g;
    wiced_i2c_device_t oled_display =
    {
        .port          = WICED_I2C_2,
        .address       = 0x3C,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .flags         = 0,
        .speed_mode    = I2C_HIGH_SPEED_MODE,
    };
    power_management_status_t battery_status;
#ifndef USE_NO_WIFI
    wiced_result_t  wifi_init_status = WICED_ERROR;
    uint32_t        update_rssi_counter = 0;
#endif
    wiced_audio_display_init_flags_t    flags = (wiced_audio_display_init_flags_t)arg;

    u8g_init_wiced_i2c_device(&oled_display);
    u8g_InitComFn(&thread_u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
    power_management_init(0);
    audio_display_init(&thread_u8g);
#ifndef USE_NO_WIFI
    if ((flags & WICED_AUDIO_DISPLAY_ENABLE_WIFI) == WICED_AUDIO_DISPLAY_ENABLE_WIFI)
    {
        wifi_init_status = wiced_wlan_connectivity_init(); // Required to get rssi and update signal strength
    }
#endif

    while(1)
    {
        /* lock bus */
        if(NULL!= audio_display_bus_mtx)
        {
            rv = wiced_rtos_lock_mutex(audio_display_bus_mtx);

            if(rv != WICED_SUCCESS )
            {
                int8_t trycount = BUS_LOCK_MAX;
                do
                {
                    wiced_rtos_delay_milliseconds(1);
                    rv = wiced_rtos_lock_mutex(audio_display_bus_mtx);
                    trycount--;
                }
                while( (trycount>0) && (rv != WICED_SUCCESS));
            }
        }

        if(rv==WICED_SUCCESS)
        {

            u8g_FirstPage(&thread_u8g);
            power_management_update(&battery_status, MAXIM_CHARGE_SPEED_FAST, 0);
            audio_display_header_update_battery(&battery_status, audio_battery.options);

#ifndef USE_NO_WIFI
            if (wifi_init_status == WICED_SUCCESS)
            {
                if ( !(update_rssi_counter++ % 6) )
                {
                    wwd_wifi_get_rssi(&g_rssi);
                    wwd_wifi_get_channel(WWD_STA_INTERFACE, &g_channel);
                    audio_display_header_update_signal_strength(g_rssi, audio_signal_strength.options);
                    if (audio_footer.options & (FOOTER_OPTION_APOLLO_TX | FOOTER_OPTION_APOLLO_RX)) {
                        audio_display_format_time_info();
                    }
                }
            }
#endif

            do {
                audio_display_draw_header();
                audio_display_draw_footer();
            } while (u8g_NextPage(&thread_u8g));

            /* unlock bus */
            if(NULL!= audio_display_bus_mtx)
            {
                rv = wiced_rtos_unlock_mutex(audio_display_bus_mtx);

                if(rv != WICED_SUCCESS )
                {
                    int8_t trycount = BUS_LOCK_MAX;
                    do
                    {
                        wiced_rtos_delay_milliseconds(1);
                        rv = wiced_rtos_unlock_mutex(audio_display_bus_mtx);
                        trycount--;
                    }
                    while( (trycount>0) && (rv != WICED_SUCCESS));
                }
            }
        }

        u8g_Delay(500);
    }
}

wiced_result_t audio_display_create_management_thread(wiced_thread_t* thread, wiced_audio_display_init_flags_t flags)
{
    wiced_result_t result;

    result = wiced_rtos_create_thread(thread, WICED_DEFAULT_LIBRARY_PRIORITY, "Audio Display Thread", audio_display_main, DISPLAY_MANAGEMENT_THREAD_SIZE, (void*)flags);
    /* Delays until thread has completed initialization */
    while (u8g == NULL)
    {
        u8g_Delay(10);
    }

    return result;
}
