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
 */

#include "wiced.h"
#include "wiced_log.h"

#include "headset_config.h"
#include "headset_dct.h"


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
static wiced_result_t headset_config_load_dct_wifi(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_load_dct_network(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_unload_dct_wifi(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_unload_dct_network(headset_dct_collection_t* dct_tables);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t headset_config_load_dct_wifi(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_lock((void**)&dct_tables->dct_wifi, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get WIFi configuration!\n");
    }
    return result;
}

static wiced_result_t headset_config_load_dct_network(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_lock( (void**)&dct_tables->dct_network, WICED_TRUE, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get Network configuration!\n");
    }
    return result;
}

wiced_result_t headset_config_init(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    /* wifi */
    result = headset_config_load_dct_wifi(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /* network */
    result = headset_config_load_dct_network(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /* App */
    result = wiced_dct_read_lock( (void**)&dct_tables->dct_app, WICED_TRUE, DCT_APP_SECTION, 0, sizeof(headset_dct_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get App configuration!\n");
        return result;
    }

    return result;
}

static wiced_result_t headset_config_unload_dct_wifi(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if (dct_tables != NULL && dct_tables->dct_wifi != NULL)
    {
        result = wiced_dct_read_unlock(dct_tables->dct_wifi, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_wifi = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't Free/Release WiFi Configuration !\n");
        }
    }

    return result;
}

static wiced_result_t headset_config_unload_dct_network(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if (dct_tables != NULL && dct_tables->dct_network != NULL)
    {
        result = wiced_dct_read_unlock(dct_tables->dct_network, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_network = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't Free/Release Network Configuration !\n");
        }
    }

    return result;
}

wiced_result_t headset_config_deinit(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_unlock(dct_tables->dct_app, WICED_TRUE);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't Free/Release App Configuration !\n");
        return result;
    }

    result = headset_config_unload_dct_network(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    result = headset_config_unload_dct_wifi(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    return result;
}

wiced_result_t headset_config_dump(headset_dct_collection_t* dct_tables)
{
    wiced_result_t                  result = WICED_SUCCESS;
    platform_dct_network_config_t*  dct_network = NULL;
    headset_dct_t*                  dct_app = NULL;
    powersave_dct_t                 powersave_dct = {0};
    audio_dct_t                     audio_dct = {0};

    if (dct_tables == NULL)
    {
        return WICED_ERROR;
    }

    dct_network = dct_tables->dct_network;
    dct_app = dct_tables->dct_app;

    if (dct_network == NULL || dct_app == NULL)
    {
        return WICED_ERROR;
    }

    powersave_dct = dct_app->powersave_dct;
    audio_dct = dct_app->audio_dct;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE,
                  "\n\n===================DCT Dump==================\n"
                  "WLAN Interface               : %s\n"
                  "WLAN Powersave Mode          : %s\n"
                  "WLAN PM2 RTS High            : %d\n"
                  "WLAN PM2 RTS Low             : %d\n"
                  "MCU Powersave Enable         : %d\n"
                  "MCU Sleep Mode               : %s\n"
                  "MCU Sleep Tick Mode          : %s\n"
                  "MCU Clock Freq               : %s\n"
                  "AUDIO Volume                 : %d\n"
                  "AUDIO Buffer Num             : %d\n"
                  "AUDIO Buffer Size            : %d\n"
                  "AUDIO Period Size            : %d\n"
                  "AUDIO HTTP Buffer Num        : %d\n"
                  "AUDIO HTTP Threshold High    : %d\n"
                  "AUDIO HTTP Threshold Low     : %d\n"
                  "AUDIO HTTP Read Inhibit      : %d\n"
                  "TCP RX Window Size           : %d\n"
                  "TCP RX Depth Queue           : %d\n"
                  "=============================================\n\n",
                   dct_network->interface == WICED_STA_INTERFACE ? "STA" :
                   dct_network->interface == WICED_AP_INTERFACE ? "AP" :
                   dct_network->interface == WICED_P2P_INTERFACE ? "P2P" : "Unknown",
                   powersave_dct.wlan_pm_mode == NO_POWERSAVE_MODE ? "No Powersave" :
                   powersave_dct.wlan_pm_mode == PM1_POWERSAVE_MODE ? "PM1" :
                   powersave_dct.wlan_pm_mode == PM2_POWERSAVE_MODE ? "PM2" : "Unknown",
                   powersave_dct.wlan_pm2_ret_on_high, powersave_dct.wlan_pm2_ret_on_low,
#ifndef WICED_DISABLE_MCU_POWERSAVE
                   WICED_TRUE,
#else
                   WICED_FALSE,
#endif
                   powersave_dct.mcu_sleep_mode == PLATFORM_MCU_POWERSAVE_MODE_DEEP_SLEEP ? "Deep Sleep" :
                   powersave_dct.mcu_sleep_mode == PLATFORM_MCU_POWERSAVE_MODE_SLEEP ? "Sleep" :
                   powersave_dct.mcu_sleep_mode == PLATFORM_MCU_POWERSAVE_MODE_MAX ? "NO Sleep" : "Unknown",
                   powersave_dct.mcu_sleep_tick_mode == PLATFORM_TICK_POWERSAVE_MODE_TICKLESS_ALWAYS ? "Tickless Always" :
                   powersave_dct.mcu_sleep_tick_mode == PLATFORM_TICK_POWERSAVE_MODE_TICKLESS_NEVER ? "Tickless Never" :
                   powersave_dct.mcu_sleep_tick_mode == PLATFORM_TICK_POWERSAVE_MODE_TICKLESS_IF_MCU_POWERSAVE_ENABLED ? "Tickless if MCU in Powersave" : "Unknown",
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_320_MHZ ? "320MHz" :
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_160_MHZ ? "160MHz" :
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_120_MHZ ? "120MHz" :
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_60_MHZ ? "60MHz" :
#if PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_48_MHZ ? "48MHz" :
                   powersave_dct.mcu_clock_freq == PLATFORM_CPU_CLOCK_FREQUENCY_24_MHZ ? "24MHz" :
#endif /* PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED */
                   "Unknown",
                   audio_dct.volume, audio_dct.audio_buffer_num, audio_dct.audio_buffer_size, audio_dct.audio_period_size,
                   audio_dct.http_buffer_num, audio_dct.http_threshold_high, audio_dct.http_threshold_low,
                   powersave_dct.wlan_pm_mode == NO_POWERSAVE_MODE ? WICED_FALSE : audio_dct.http_read_inhibit,
#ifdef WICED_TCP_WINDOW_SIZE
                   WICED_TCP_WINDOW_SIZE,
#else
                   WICED_DEFAULT_TCP_WINDOW_SIZE,
#endif
#ifdef WICED_TCP_RX_DEPTH_QUEUE
                   WICED_TCP_RX_DEPTH_QUEUE
#else
                   WICED_DEFAULT_TCP_RX_DEPTH_QUEUE
#endif
                   );

    return result;
}
