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

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Control whether the library logs to console */
#ifndef WICED_AUDIO_LOOPBACK_LOG_ENABLED
#define WICED_AUDIO_LOOPBACK_LOG_ENABLED                (1)
#endif

/* If memory is a concern, disable FFT validation (adds ~6K to RAM usage)*/
#ifndef FFT_VALIDATION
#define FFT_VALIDATION      (1)
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct audio_loopback_config_s
{

/* Transmit a sine wave instead of data from the SDIN line. */
    wiced_bool_t    enable_sinewave;

/* Enable data validation when I2S SDIN/SDOUT lines are shorted (digital loopback) or
 * audio out line is connected to audio in (analog loopback).
 * This is only useful when enable_sinewave is set.
*/
    wiced_bool_t    enable_data_validation;

/* In some cases, it is not physically possible to create a digital loopback (i.e. pins
   are not accessible on the board). Analog audioloopback is easier to create.
   However, due to the analog nature, the input audio data will never match the output
   test data. Hence, instead of comparing samples one by one, fft is used to calculate
   the main signal harmonic which is compared against the test signal frequency.
   This is only useful if enable_sinewave and enable_data_validation are set.
*/
    wiced_bool_t    use_fft;
/* If FFT is enabled, hamming window helps to filter the non-periodic input data
   thus improving result accuracy.*/
    wiced_bool_t    use_hamming_window;
    platform_audio_device_id_t tx_audio_device;
    platform_audio_device_id_t rx_audio_device;
} audio_loopback_config_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_audio_loopback_run( uint32_t iterations );
wiced_result_t wiced_audio_loopback_config( audio_loopback_config_t *config);
wiced_result_t wiced_audio_loopback_start( void );
wiced_result_t wiced_audio_loopback_stop( void );

#ifdef __cplusplus
} /*extern "C" */
#endif



