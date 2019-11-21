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

#include "platform_audio.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* standard audio device info */

/* standard audio device information */
#define SPDIF_ADC_NAME         "SPDIF IN"
#define SPDIF_ADC_DIRECTION    PLATFORM_AUDIO_DEVICE_INPUT
#define SPDIF_ADC_PORT_TYPE    PLATFORM_AUDIO_SPDIF
#define SPDIF_ADC_CHANNELS     6
#define SPDIF_ADC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define SPDIF_ADC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ    | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ |    \
                                PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ |    \
                                PLATFORM_AUDIO_SAMPLE_RATE_96KHZ )

#define SPDIF_DAC_NAME         "SPDIF OUT"
#define SPDIF_DAC_DIRECTION    PLATFORM_AUDIO_DEVICE_OUTPUT
#define SPDIF_DAC_PORT_TYPE    PLATFORM_AUDIO_SPDIF
#define SPDIF_DAC_CHANNELS     6
#define SPDIF_DAC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define SPDIF_DAC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ    | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ |    \
                                PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ |    \
                                PLATFORM_AUDIO_SAMPLE_RATE_96KHZ )


#define AUDIO_DEVICE_ID_SPDIF_ADC_INFO                                    \
    {   AUDIO_DEVICE_ID_SPDIF_ADC, SPDIF_ADC_NAME, SPDIF_ADC_DESCRIPTION, \
        SPDIF_ADC_DIRECTION, SPDIF_ADC_PORT_TYPE,                         \
        SPDIF_ADC_CHANNELS,  SPDIF_ADC_SIZES, SPDIF_ADC_RATES    }

#define AUDIO_DEVICE_ID_SPDIF_DAC_INFO                                    \
    {   AUDIO_DEVICE_ID_SPDIF_DAC, SPDIF_DAC_NAME, SPDIF_DAC_DESCRIPTION, \
        SPDIF_DAC_DIRECTION, SPDIF_DAC_PORT_TYPE,                         \
        SPDIF_DAC_CHANNELS,  SPDIF_DAC_SIZES, SPDIF_DAC_RATES    }

#define I2S_0_ADC_NAME         "I2S0 IN"
#define I2S_1_ADC_NAME         "I2S1 IN"
#define I2S_ADC_DIRECTION      PLATFORM_AUDIO_DEVICE_INPUT
#define I2S_ADC_PORT_TYPE      PLATFORM_AUDIO_I2S
#define I2S_ADC_CHANNELS       2
#define I2S_ADC_SIZES          (PLATFORM_AUDIO_SAMPLE_SIZE_8_BIT    | PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | \
                                PLATFORM_AUDIO_SAMPLE_SIZE_20_BIT   | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | \
                                PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define I2S_ADC_RATES          (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ     | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_24KHZ    | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_88_2KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_96KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_176_4KHZ | PLATFORM_AUDIO_SAMPLE_RATE_192KHZ)

#define I2S_0_DAC_NAME         "I2S0 OUT"
#define I2S_1_DAC_NAME         "I2S1 OUT"
#define I2S_DAC_DIRECTION      PLATFORM_AUDIO_DEVICE_OUTPUT
#define I2S_DAC_PORT_TYPE      PLATFORM_AUDIO_I2S
#define I2S_DAC_CHANNELS       2
#define I2S_DAC_SIZES          (PLATFORM_AUDIO_SAMPLE_SIZE_8_BIT    | PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT | \
                                PLATFORM_AUDIO_SAMPLE_SIZE_20_BIT   | PLATFORM_AUDIO_SAMPLE_SIZE_24_BIT | \
                                PLATFORM_AUDIO_SAMPLE_SIZE_32_BIT)
#define I2S_DAC_RATES          (PLATFORM_AUDIO_SAMPLE_RATE_8KHZ     | PLATFORM_AUDIO_SAMPLE_RATE_16KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_24KHZ    | PLATFORM_AUDIO_SAMPLE_RATE_32KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_48KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_88_2KHZ  | PLATFORM_AUDIO_SAMPLE_RATE_96KHZ  | \
                                PLATFORM_AUDIO_SAMPLE_RATE_176_4KHZ | PLATFORM_AUDIO_SAMPLE_RATE_192KHZ)

#define AUDIO_DEVICE_ID_I2S_0_ADC_INFO                                    \
    {   AUDIO_DEVICE_ID_I2S_0_ADC, I2S_0_ADC_NAME, I2S_0_ADC_DESCRIPTION, \
        I2S_ADC_DIRECTION, I2S_ADC_PORT_TYPE,                             \
        I2S_ADC_CHANNELS,  I2S_ADC_SIZES, I2S_ADC_RATES   }

#define AUDIO_DEVICE_ID_I2S_1_ADC_INFO                                    \
    {   AUDIO_DEVICE_ID_I2S_1_ADC, I2S_1_ADC_NAME, I2S_1_ADC_DESCRIPTION, \
        I2S_ADC_DIRECTION, I2S_ADC_PORT_TYPE,                             \
        I2S_ADC_CHANNELS,  I2S_ADC_SIZES, I2S_ADC_RATES   }

#define AUDIO_DEVICE_ID_I2S_0_DAC_INFO                                    \
    {   AUDIO_DEVICE_ID_I2S_0_DAC, I2S_0_DAC_NAME, I2S_0_DAC_DESCRIPTION, \
        I2S_DAC_DIRECTION, I2S_DAC_PORT_TYPE,                             \
        I2S_DAC_CHANNELS,  I2S_DAC_SIZES, I2S_DAC_RATES    }

#define AUDIO_DEVICE_ID_I2S_1_DAC_INFO                                    \
    {   AUDIO_DEVICE_ID_I2S_1_DAC, I2S_1_DAC_NAME, I2S_1_DAC_DESCRIPTION, \
        I2S_DAC_DIRECTION, I2S_DAC_PORT_TYPE,                             \
        I2S_DAC_CHANNELS,  I2S_DAC_SIZES, I2S_DAC_RATES    }

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    platform_audio_port_type_t port_type;
    platform_audio_direction_t port_direction;
    wiced_i2s_t                data_port;
} spdif_device_data_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


wiced_result_t spdif_device_register( spdif_device_data_t* device_data, platform_audio_device_id_t device_id );


#ifdef __cplusplus
} /* extern "C" */
#endif
