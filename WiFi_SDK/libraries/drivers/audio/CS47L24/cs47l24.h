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

#include <stdint.h>
#include "wiced_result.h"
#include "wiced_audio.h"
#include "wiced_platform.h"
#include "wiced_resource.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define cs47l24_pll_master       { cs47l24_mpll, 1, 1 }
#define cs47l24_pll_slave        { cs47l24_spll, 1, 0 }
#define cs47l24_ext_master       { cs47l24_ext,  0, 1 }
#define cs47l24_ext_slave        { cs47l24_ext,  0, 0 }

/******************************************************
 *            Constants
 ******************************************************/

/* Maximum supported CS47L24 devices. */
#define CS47L24_MAX_DEVICES      1

/* standard audio device information */
#define CS47L24_ADC_NAME         "cs47l24_adc"
#define CS47L24_ADC_DIRECTION    PLATFORM_AUDIO_DEVICE_INPUT
#define CS47L24_ADC_PORT_TYPE    PLATFORM_AUDIO_LINE
#define CS47L24_ADC_CHANNELS     2
#define CS47L24_ADC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_8_BIT    | PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT)
#define CS47L24_ADC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ     | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_22_05KHZ | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_88_2KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_96KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_176_4KHZ | PLATFORM_AUDIO_SAMPLE_RATE_192KHZ)

#define CS47L24_DAC_NAME         "cs47l24_dac"
#define CS47L24_DAC_DIRECTION    PLATFORM_AUDIO_DEVICE_OUTPUT
#define CS47L24_DAC_PORT_TYPE    PLATFORM_AUDIO_LINE
#define CS47L24_DAC_CHANNELS     2
#define CS47L24_DAC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_8_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define CS47L24_DAC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ     | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_22_05KHZ | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_88_2KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_96KHZ  | \
                                  PLATFORM_AUDIO_SAMPLE_RATE_176_4KHZ | PLATFORM_AUDIO_SAMPLE_RATE_192KHZ)

#define CS47L24_ADC_DIGITAL_MIC_NAME       "cs47l24_adc_dmic"
#define CS47L24_ADC_DIGITAL_MIC_DIRECTION  CS47L24_ADC_DIRECTION
#define CS47L24_ADC_DIGITAL_MIC_PORT_TYPE  PLATFORM_AUDIO_DIGITAL_MIC
#define CS47L24_ADC_DIGITAL_MIC_CHANNELS   CS47L24_ADC_CHANNELS
#define CS47L24_ADC_DIGITAL_MIC_SIZES      CS47L24_ADC_SIZES
#define CS47L24_ADC_DIGITAL_MIC_RATES      CS47L24_ADC_RATES

#define AUDIO_DEVICE_ID_CS47L24_ADC_LINE_INFO                     \
        { AUDIO_DEVICE_ID_CS47L24_ADC_LINE, CS47L24_ADC_NAME,     \
          CS47L24_ADC_DESCRIPTION, CS47L24_ADC_DIRECTION,         \
          CS47L24_ADC_PORT_TYPE, CS47L24_ADC_CHANNELS,            \
          CS47L24_ADC_SIZES, CS47L24_ADC_RATES }

#define AUDIO_DEVICE_ID_CS47L24_DAC_LINE_INFO                     \
        { AUDIO_DEVICE_ID_CS47L24_DAC_LINE, CS47L24_DAC_NAME,     \
          CS47L24_DAC_DESCRIPTION, CS47L24_DAC_DIRECTION,         \
          CS47L24_DAC_PORT_TYPE, CS47L24_DAC_CHANNELS,            \
          CS47L24_DAC_SIZES, CS47L24_DAC_RATES }

#define AUDIO_DEVICE_ID_CS47L24_ADC_DIGITAL_MIC_INFO                              \
        { AUDIO_DEVICE_ID_CS47L24_ADC_DIGITAL_MIC, CS47L24_ADC_DIGITAL_MIC_NAME,  \
          CS47L24_ADC_DIGITAL_MIC_DESCRIPTION, CS47L24_ADC_DIGITAL_MIC_DIRECTION, \
          CS47L24_ADC_DIGITAL_MIC_PORT_TYPE, CS47L24_ADC_DIGITAL_MIC_CHANNELS,    \
          CS47L24_ADC_DIGITAL_MIC_SIZES, CS47L24_ADC_DIGITAL_MIC_RATES }

/******************************************************
 *            Enumerations
 ******************************************************/

typedef enum
{
    CS47L24_DEVICE_ID_0              = 0,
    /* Not a device id! */
    CS47L24_DEVICE_ID_MAX,
} cs47l24_device_id_t;

typedef enum
{
    CS47L24_FIRMWARE_DSP2,
    CS47L24_FIRMWARE_DSP3,
    CS47L24_FIRMWARE_DSP_MAX,        /* not a valid firmware type */
} cs47l24_dsp_firmware_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct cs47l24_device_route cs47l24_device_route_t;
typedef struct cs47l24_audio_device_interface cs47l24_audio_device_interface_t;
typedef struct cs47l24_device_data cs47l24_device_data_t;

typedef wiced_result_t (cs47l24_ckcfg_fn_t)(cs47l24_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk);

/******************************************************
 *             Structures
 ******************************************************/

typedef struct
{
    cs47l24_ckcfg_fn_t             *fn;
    uint8_t                         pll_enab        : 1;
    uint8_t                         is_frame_master : 1;
} cs47l24_clock_settings_t;

typedef struct
{
    const resource_hnd_t          **dsp_res_table;
} cs47l24_dsp_resource_t;

typedef struct
{
    cs47l24_device_id_t             id;
    wiced_spi_device_t             *spi_data;
    cs47l24_clock_settings_t        ck;
    wiced_gpio_t                    pdn;
    cs47l24_dsp_resource_t         *dsp;
} cs47l24_device_cmn_data_t;

struct cs47l24_device_data
{
    const cs47l24_device_route_t   *route;
    cs47l24_device_cmn_data_t      *cmn;
    wiced_i2s_t                     data_port;
};

/******************************************************
 *             Variable declarations
 ******************************************************/

extern const cs47l24_device_route_t cs47l24_route_dac_hp;
extern const cs47l24_device_route_t cs47l24_route_adc_dmic;

/******************************************************
 *             Function declarations
 ******************************************************/

wiced_result_t cs47l24_platform_configure( cs47l24_device_data_t *device_data, uint32_t mclk, uint32_t fs, uint8_t width );

wiced_result_t cs47l24_device_register( cs47l24_device_data_t *device_data, const platform_audio_device_id_t device_id );

/* Don't use these functions directly; use macros provided. */
cs47l24_ckcfg_fn_t cs47l24_mpll, cs47l24_spll, cs47l24_ext;

#ifdef __cplusplus
} /* extern "C" */
#endif
