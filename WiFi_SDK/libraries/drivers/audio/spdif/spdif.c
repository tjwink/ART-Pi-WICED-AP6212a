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

#include "wiced_platform.h"
#include "wiced_rtos.h"
#include "wiced_audio.h"
#include "spdif.h"
#include "platform_i2s.h"

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

static wiced_result_t spdif_init(void* device_data, wiced_audio_data_port_t* data_port);
static wiced_result_t spdif_deinit(void* device_data);
static wiced_result_t spdif_configure(void* device_data, wiced_audio_config_t* config, uint32_t* mclk);
static wiced_result_t spdif_start_play(void* device_data);
static wiced_result_t spdif_stop_play(void* device_data);
static wiced_result_t spdif_set_volume(void* device_data, double decibles);
static wiced_result_t spdif_get_volume_range(void* device_data, double* min_volume_decibels, double* max_volume_decibels);
static wiced_result_t spdif_ioctl(void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data);

/******************************************************
 *               Variables Definitions
 ******************************************************/

wiced_audio_device_interface_t spdif_interface =
{
        .audio_device_init             = spdif_init,
        .audio_device_deinit           = spdif_deinit,
        .audio_device_configure        = spdif_configure,
        .audio_device_start_streaming  = spdif_start_play,
        .audio_device_stop_streaming   = spdif_stop_play,
        .audio_device_set_volume       = spdif_set_volume,
        .audio_device_set_treble       = NULL,
        .audio_device_set_bass         = NULL,
        .audio_device_get_volume_range = spdif_get_volume_range,
        .audio_device_ioctl            = spdif_ioctl,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t spdif_init(void* device_data, wiced_audio_data_port_t* data_port)
{
    spdif_device_data_t* dd = (spdif_device_data_t*) device_data;

    data_port->port = dd->data_port;

    switch (dd->port_type)
    {
        case PLATFORM_AUDIO_SPDIF:
            data_port->type = dd->port_type;
            break;

        default:
            data_port->type = PLATFORM_AUDIO_I2S;
            break;
    }

    switch (dd->port_direction)
    {
        case PLATFORM_AUDIO_DEVICE_INPUT:
            data_port->channel = WICED_RECORD_CHANNEL;
            break;

        default:
            data_port->channel = WICED_PLAY_CHANNEL;
            break;
    }

    return WICED_SUCCESS;
}

static wiced_result_t spdif_deinit(void* device_data)
{
    (void)device_data;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_configure(void* device_data, wiced_audio_config_t* config, uint32_t* mclk)
{
    (void)device_data;
    (void)config;
    (void)mclk;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_start_play(void* device_data)
{
    (void)device_data;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_stop_play(void* device_data)
{
    (void)device_data;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_set_volume(void* device_data, double decibles)
{
    (void)device_data;
    (void)decibles;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_get_volume_range(void* device_data, double* min_volume_decibels, double* max_volume_decibels)
{
    (void)device_data;

    *min_volume_decibels = 0.0;
    *max_volume_decibels = 100.0;

    return WICED_SUCCESS;
}

static wiced_result_t spdif_ioctl(void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data)
{
    UNUSED_PARAMETER( driver_data );
    UNUSED_PARAMETER( cmd );
    UNUSED_PARAMETER( cmd_data );

    return WICED_UNSUPPORTED;
}


/* This function can only be called from the platform initialization routine */
wiced_result_t spdif_device_register( spdif_device_data_t* device_data, platform_audio_device_id_t device_id )
{
    if( device_data == NULL )
    {
        return WICED_BADARG;
    }
    spdif_interface.audio_device_driver_specific = device_data;
    spdif_interface.device_id = device_id;

    /* Register a device to the audio device list and keep device data internally from this point */
    return wiced_register_audio_device(device_id, &spdif_interface);
}
