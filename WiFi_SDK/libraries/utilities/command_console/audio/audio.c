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

#include "ctype.h"
#include "wiced.h"
#include "platform_audio.h"
#include "command_console_audio.h"
#include "command_console.h"
#include "audio_loopback.h"
#include "wiced_log.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Runs audio loopback test
 *
 * @return  0 for success, otherwise error
 */

int audio_loopback( int argc, char* argv[] )
{
    uint32_t value = 100, route = PLATFORM_AUDIO_ROUTE_CONFIG_DEFAULT;
    wiced_result_t result;
    audio_loopback_config_t config;

    string_to_unsigned(argv[1], strlen(argv[1]), &value, 0);

    if (argc == 5 || argc == 6)
    {
        uint32_t val;
        string_to_unsigned( argv[2], strlen(argv[2]), &val, 0);
        val? (config.use_fft = WICED_TRUE):(config.use_fft = WICED_FALSE);
        string_to_unsigned( argv[3], strlen(argv[3]), &val, 0);
        val? (config.enable_sinewave = WICED_TRUE):(config.enable_sinewave = WICED_FALSE);
        string_to_unsigned( argv[4], strlen(argv[4]), &val, 0);
        val? (config.enable_data_validation = WICED_TRUE):(config.enable_data_validation = WICED_FALSE);

        /*enable filtering*/
        config.use_hamming_window = WICED_TRUE;

        /*default input and output*/
        config.tx_audio_device = PLATFORM_DEFAULT_AUDIO_OUTPUT;
        config.rx_audio_device = PLATFORM_DEFAULT_AUDIO_INPUT;

        wiced_audio_loopback_config(&config);

        wiced_log_printf("audio_loopback: fft: %d sw:%d valid:%d\n", (int)config.use_fft,
            (int)config.enable_sinewave, (int)config.enable_data_validation);

        if (argc == 6)
        {
            string_to_unsigned( argv[5], strlen(argv[5]), &route, 0);
            wiced_log_printf("audio_loopback: route config: %d\n", (int)route);
        }
    }
    else if (argc != 2)
    {
        wiced_log_printf("audio_loopback: wrong params\n");
        return ERR_CMD_OK;
    }

    wiced_log_printf("audio_loopback: iterations: %d\n", (int)value);

#ifdef WICED_AUDIO_ROUTE_RECONFIG_ENABLE
        /* override default routes.
         * Test uses i2s1 instead of default i2s0.
         */
    switch (route)
    {
        case PLATFORM_AUDIO_ROUTE_CONFIG_1:
            platform_configure_dac_route(PLATFORM_AUDIO_ROUTE_CONFIG_1, WICED_I2S_5);
            platform_configure_adc_route(PLATFORM_AUDIO_ROUTE_CONFIG_1, WICED_I2S_4);
            break;
        case PLATFORM_AUDIO_ROUTE_CONFIG_DEFAULT:
        default:
            break;
    }
#endif

    result = wiced_audio_loopback_run(value);

    if (result == WICED_SUCCESS)
    {
        wiced_log_printf("audio_loopback: success\n");
    }
    else
    {
        wiced_log_printf("audio_loopback: failure\n");
    }

    return ERR_CMD_OK;
}

