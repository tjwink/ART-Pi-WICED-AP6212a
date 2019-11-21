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

#pragma once

#include "wiced_audio.h"
#include "wiced_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define ak4954_pll_master       { ak4954_mpll, 1, 1 }
#define ak4954_pll_slave        { ak4954_spll, 1, 0 }
#define ak4954_ext_master       { ak4954_ext,  0, 1 }
#define ak4954_ext_slave        { ak4954_ext,  0, 0 }

/******************************************************
 *            Constants
 ******************************************************/

/* Maximum supported AK4954 devices. */
#define AK4954_MAX_DEVICES      1

/* standard audio device information */
#define AK4954_ADC_NAME         "ak4954_adc"
#define AK4954_ADC_DIRECTION    PLATFORM_AUDIO_DEVICE_INPUT
#define AK4954_ADC_PORT_TYPE    PLATFORM_AUDIO_LINE
#define AK4954_ADC_CHANNELS     2
#define AK4954_ADC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT)
#define AK4954_ADC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ |      \
                                 PLATFORM_AUDIO_SAMPLE_RATE_22_05KHZ | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ |  \
                                 PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ |   \
                                 PLATFORM_AUDIO_SAMPLE_RATE_96KHZ)

#define AK4954_DAC_NAME         "ak4954_dac"
#define AK4954_DAC_DIRECTION    PLATFORM_AUDIO_DEVICE_OUTPUT
#define AK4954_DAC_PORT_TYPE    PLATFORM_AUDIO_LINE
#define AK4954_DAC_CHANNELS     2
#define AK4954_DAC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define AK4954_DAC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ |      \
                                 PLATFORM_AUDIO_SAMPLE_RATE_22_05KHZ | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ |  \
                                 PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ |   \
                                 PLATFORM_AUDIO_SAMPLE_RATE_96KHZ)

#define AUDIO_DEVICE_ID_AK4954_ADC_LINE_INFO                    \
        { AUDIO_DEVICE_ID_AK4954_ADC_LINE, AK4954_ADC_NAME,     \
          AK4954_ADC_DESCRIPTION, AK4954_ADC_DIRECTION,         \
          AK4954_ADC_PORT_TYPE, AK4954_ADC_CHANNELS,            \
          AK4954_ADC_SIZES, AK4954_ADC_RATES }

#define AUDIO_DEVICE_ID_AK4954_DAC_LINE_INFO                    \
        { AUDIO_DEVICE_ID_AK4954_DAC_LINE, AK4954_DAC_NAME,     \
          AK4954_DAC_DESCRIPTION, AK4954_DAC_DIRECTION,         \
          AK4954_DAC_PORT_TYPE, AK4954_DAC_CHANNELS,            \
          AK4954_DAC_SIZES, AK4954_DAC_RATES }

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum ak4954_device_id ak4954_device_id_t;

typedef struct ak4954_clock_settings ak4954_clock_settings_t;
typedef struct ak4954_device_cmn_data ak4954_device_cmn_data_t;
typedef struct ak4954_device_route ak4954_device_route_t;
typedef struct ak4954_device_data ak4954_device_data_t;
typedef struct ak4954_audio_device_interface ak4954_audio_device_interface_t;

typedef wiced_result_t (ak4954_ckcfg_fn_t)(ak4954_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk);

/******************************************************
 *            Enumerations
 ******************************************************/

enum ak4954_device_id
{
    AK4954_DEVICE_ID_0              = 0,

    /* Not a device id! */
    AK4954_DEVICE_ID_MAX,
};

/******************************************************
 *             Structures
 ******************************************************/

struct ak4954_clock_settings
{
    ak4954_ckcfg_fn_t               *fn;
    uint8_t                         pll_enab        : 1;
    uint8_t                         is_frame_master : 1;
};

struct ak4954_device_cmn_data
{
    ak4954_device_id_t              id;
    wiced_i2c_device_t *            i2c_data;
    ak4954_clock_settings_t         ck;
    wiced_gpio_t                    pdn;
};

struct ak4954_device_data
{
    const ak4954_device_route_t     *route;
    ak4954_device_cmn_data_t        *cmn;
    wiced_i2s_t                     data_port;
};

/******************************************************
 *             Variable declarations
 ******************************************************/

extern const ak4954_device_route_t ak4954_dac_hp;
extern const ak4954_device_route_t ak4954_dac_spkr;
extern const ak4954_device_route_t ak4954_adc_mic;

/******************************************************
 *             Function declarations
 ******************************************************/

wiced_result_t ak4954_platform_configure( ak4954_device_data_t *device_data, uint32_t mclk, uint32_t fs, uint8_t width );

wiced_result_t ak4954_device_register( ak4954_device_data_t *device_data, const platform_audio_device_id_t device_id );

/* Don't use these functions directly; use macros provided. */
ak4954_ckcfg_fn_t   ak4954_mpll, ak4954_spll, ak4954_ext;

#ifdef __cplusplus
} /* extern "C" */
#endif
