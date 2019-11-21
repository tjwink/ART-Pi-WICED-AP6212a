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
#include "wm8533_dac.h"
#include "platform_i2s.h"

/******************************************************
 *                      Macros
 ******************************************************/
/* Default/Theoretical MIN and MAX DAC volume in decibels.
 * The platform may override to a subset of this range to ensure that audio
 * artifacts are minimized.
 *
 */
#ifndef MIN_WM8533_DB_LEVEL
#define MIN_WM8533_DB_LEVEL    ( -100.00 )/* in decibels */
#endif
#ifndef MAX_WM8533_DB_LEVEL
#define MAX_WM8533_DB_LEVEL    ( +12.00 ) /* in decibels */
#endif

#define GAIN_ADJUSTMENT_STEP   ( 0.25 ) /* in decibels */

#define GAIN_REG_VALUE_ON_ZERO_DB (0x190)

#define I2S_STANDARD_PHILLIPS

#ifdef I2S_STANDARD_PHILLIPS
 #define  CODEC_STANDARD                AIF_FMT_I2S
#elif defined(I2S_STANDARD_MSB)
 #define  CODEC_STANDARD                AIF_FMT_MSB
#elif defined(I2S_STANDARD_LSB)
 #define  CODEC_STANDARD                AIF_FMT_LSB
#else
 #error "Error: No audio communication standard selected !"
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/* Register fields */
#define DIGITAL_VOLUME_UPDATE_FIELD_SHIFT   (9)
#define DIGITAL_VOLUME_UPDATE_FIELD_BITS    (1)
#define DIGITAL_VOLUME_DB_VALUE             (0)
#define VOLUME_UP_RAMP_FIELD_SHIFT          (1)
#define VOLUME_UP_RAMP_FIELD_BITS           (1)
#define VOLUME_DOWN_RAMP_FIELD              (0)
#define VOLUME_DOWN_RAMP_FIELD_BITS         (1)
#define DAC_ZERO_CROSS_FIELD_SHIFT          (4)
#define DAC_ZERO_CROSS_FIELD_BITS           (1)
#define SYS_ENA_FIELD_SHIFT                 (0)
#define SYS_ENA_BITS                        (2)
#define SYS_ENA_POWER_OFF                   (0)
#define SYS_ENA_POWER_DOWN                  (1)
#define SYS_ENA_POWER_UP_WITH_MUTE          (2)
#define SYS_ENA_POWER_UP                    (3)
#define AIF_WL_SHIFT                        (3)
#define AIF_WL_BITS                         (2)
#define AIF_WL_16_BITS                      (0)
#define AIF_WL_20_BITS                      (1)
#define AIF_WL_24_BITS                      (2)
#define AIF_WL_32_BITS                      (3)
#define AIF_FMT_SHIFT                       (0)
#define AIF_FMT_BITS                        (2)
#define AIF_FMT_LSB                         (0)
#define AIF_FMT_MSB                         (1)
#define AIF_FMT_I2S                         (2)
#define AIF_FMT_DSP                         (3)
#define AIF_SR_SHIFT                        (0)
#define AIF_SR_BITS                         (3)
#define AIF_SR_AUTO                         (0)
#define AIF_SR_128FS                        (1)
#define AIF_SR_192FS                        (2)
#define AIF_SR_256FS                        (3)
#define AIF_SR_384FS                        (4)
#define AIF_SR_512FS                        (5)
#define AIF_SR_768FS                        (6)
#define AIF_SR_1152FS                       (7)
#define AIF_MSTR_SHIFT                      (7)
#define AIF_MSTR_BITS                       (1)
#define AIF_MSTR_SLAVE                      (0)
#define AIF_MSTR_MASTER                     (1)
#define AIF_BCLKDIV_SHIFT                   (3)
#define AIF_BCLKDIV_BITS                    (3)
#define AIF_BCLKDIV_MCLKDIV4                (0)
#define AIF_BCLKDIV_MCLKDIV8                (1)
#define AIF_BCLKDIV_32FS                    (2)
#define AIF_BCLKDIV_64FS                    (3)
#define AIF_BCLKDIV_128FS                   (4)

/* registers */
#define WM8533_REG_DAC_GAINL    (0x06)
#define WM8533_REG_DAC_GAINR    (0x07)
#define WM8533_REG_DAC_CTRL3    (0x05)
#define WM8533_REG_PSCTRL1      (0x02)
#define WM8533_REG_AIF_CTRL2    (0x04)
#define WM8533_REG_AIF_CTRL1    (0x03)
#define WM8533_REG_CHIP_ID      (0x00)
#define WM8533_REG_REVISION_ID  (0x01)

#define I2C_XFER_RETRY_COUNT    3
#define I2C_DMA_POLICY          WICED_TRUE

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint32_t mclk;
    uint32_t fs;
    uint8_t  sr;
} mclk_lrclk_map_t;

typedef struct
{
    uint8_t reg;
    uint8_t value_h;
    uint8_t value_l;
} i2c_wm8533_payload_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_audio_data_provider_t  give_more_samples_callback;
wiced_audio_device_interface_t      wm8533_interface;

/* WM8533 data-sheet table 11. */
static const mclk_lrclk_map_t mclk_lrclk_map[] =
{
    {  2048000,   8000, AIF_SR_256FS  },
    {  3072000,   8000, AIF_SR_384FS  },
    {  3072000,  12000, AIF_SR_256FS  },
    {  4096000,   8000, AIF_SR_512FS  },
    {  4096000,  16000, AIF_SR_256FS  },
    {  6144000,   8000, AIF_SR_768FS  },
    {  6144000,  12000, AIF_SR_512FS  },
    {  6144000,  16000, AIF_SR_384FS  },
    {  6144000,  24000, AIF_SR_256FS  },
    {  8192000,  16000, AIF_SR_512FS  },
    {  8192000,  32000, AIF_SR_256FS  },
    {  9216000,   8000, AIF_SR_1152FS },
    {  9216000,  12000, AIF_SR_768FS  },
    {  9216000,  24000, AIF_SR_384FS  },
    { 11289600,  44100, AIF_SR_256FS  },
    { 11289600,  88200, AIF_SR_128FS  },
    { 12288000,  16000, AIF_SR_768FS  },
    { 12288000,  24000, AIF_SR_512FS  },
    { 12288000,  32000, AIF_SR_384FS  },
    { 12288000,  48000, AIF_SR_256FS  },
    { 12288000,  64000, AIF_SR_192FS  },
    { 12288000,  96000, AIF_SR_128FS  },
    { 16384000,  32000, AIF_SR_512FS  },
    { 16384000,  64000, AIF_SR_256FS  },
    { 16384000, 128000, AIF_SR_128FS  },
    { 16934400,  44100, AIF_SR_384FS  },
    { 16934400,  88200, AIF_SR_192FS  },
    { 18432000,  16000, AIF_SR_1152FS },
    { 18432000,  24000, AIF_SR_768FS  },
    { 18432000,  48000, AIF_SR_384FS  },
    { 18432000,  96000, AIF_SR_192FS  },
    { 22579200,  44100, AIF_SR_512FS  },
    { 22579200,  88200, AIF_SR_256FS  },
    { 22579200, 176400, AIF_SR_128FS  },
    { 24576000,  32000, AIF_SR_768FS  },
    { 24576000,  48000, AIF_SR_512FS  },
    { 24576000,  64000, AIF_SR_384FS  },
    { 24576000,  96000, AIF_SR_256FS  },
    { 24576000, 128000, AIF_SR_192FS  },
    { 24576000, 192000, AIF_SR_128FS  },
    { 33868800,  44100, AIF_SR_768FS  },
    { 33868800,  88200, AIF_SR_384FS  },
    { 33868800, 176400, AIF_SR_192FS  },
    { 36864000,  32000, AIF_SR_1152FS },
    { 36864000,  48000, AIF_SR_768FS  },
    { 36864000,  96000, AIF_SR_384FS  },
    { 36864000, 192000, AIF_SR_192FS  },
};

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t wm8533_reg_write( wm8533_device_data_t* wm8533, uint8_t reg, uint16_t value );
static wiced_result_t wm8533_reg_read ( wm8533_device_data_t* wm8533, uint8_t reg, uint16_t* value );

static wiced_result_t wm8533_configure ( void* driver_data, wiced_audio_config_t *config, uint32_t* mclk);
static wiced_result_t wm8533_init      ( void* driver_data, wiced_audio_data_port_t* data_port );
static wiced_result_t wm8533_start_play( void* driver_data );
static wiced_result_t wm8533_stop_play ( void* driver_data );

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t wm8533_i2c_reg_write( wiced_i2c_device_t *device, uint8_t reg, uint16_t value )
{
    static wiced_i2c_message_t msg[1];
    i2c_wm8533_payload_t payload;

    payload.reg = reg;
    payload.value_h = ( (value & 0xFF00) >> 8 )  ;
    payload.value_l = ( value & 0x00FF );

    WICED_VERIFY( wiced_i2c_init_tx_message( msg, &payload, 3, I2C_XFER_RETRY_COUNT, I2C_DMA_POLICY ) );

   return wiced_i2c_transfer( device, msg, 1 );
}

#if 0
static wiced_result_t wm8533_i2c_reg_read(wiced_i2c_device_t *device, uint8_t reg, uint16_t *value)
{
    static wiced_i2c_message_t msg[1];
    uint8_t value_read[2];
    WICED_VERIFY( wiced_i2c_init_combined_message( msg, &reg, value_read, sizeof(reg), sizeof(value_read), I2C_XFER_RETRY_COUNT, I2C_DMA_POLICY ) );

    WICED_VERIFY( wiced_i2c_transfer(device, msg, 1) );
    *value = ( value_read[0] << 8 ) + value_read[1];

    return WICED_SUCCESS;
}
#else
static wiced_result_t wm8533_i2c_reg_read( wiced_i2c_device_t *device, uint8_t reg, uint16_t *value )
{
    wiced_i2c_message_t msg[2];
    uint8_t value_read[2];

    /* Reset device's register counter by issuing a TX. */
    WICED_VERIFY( wiced_i2c_init_tx_message( &msg[0], &reg, 1, I2C_XFER_RETRY_COUNT, I2C_DMA_POLICY ) );

    /* Initialize RX message. */
    WICED_VERIFY( wiced_i2c_init_rx_message( &msg[1], value_read, sizeof(value_read), I2C_XFER_RETRY_COUNT, I2C_DMA_POLICY ) );

    /* Transfer. */
    WICED_VERIFY( wiced_i2c_transfer( device, msg, 2 ) );

    /* Swap bytes. */
    *value = ( value_read[0] << 8 ) + value_read[1];

    return WICED_SUCCESS;
}
#endif

static wiced_result_t wm8533_init ( void* driver_data, wiced_audio_data_port_t* data_port )
{
    wm8533_device_data_t* wm8533 = ( wm8533_device_data_t* )driver_data;
    uint16_t psctrl_value;

    /* Enable I2C clocks, init I2C peripheral. */
    WICED_VERIFY( wiced_i2c_init( wm8533->i2c_data ) );

    /* Initialize GPIOs. */
//    if( wm8533->cifmode != WICED_GPIO_NONE )
//    {
//        WICED_VERIFY( wiced_gpio_init( wm8533->cifmode, OUTPUT_PUSH_PULL ) );
//    }
//    if( wm8533->addr0 != WICED_GPIO_NONE )
//    {
//        WICED_VERIFY( wiced_gpio_init( wm8533->addr0, OUTPUT_PUSH_PULL ) );
//    }

    /* Configure DAC's control interface to I2C. */
//    if( wm8533->cifmode != WICED_GPIO_NONE )
//    {
//        WICED_VERIFY( wiced_gpio_output_low( wm8533->cifmode ) );
//    }

    /* Configure DAC's I2C address to 0x1A (0011010b) */
//    if( wm8533->addr0 != WICED_GPIO_NONE )
//    {
//        WICED_VERIFY( wiced_gpio_output_low( wm8533->addr0 ) );
//    }


    psctrl_value = ( SYS_ENA_POWER_DOWN << SYS_ENA_FIELD_SHIFT );
    /* first stage, all analog block and digital signal blocks are powered off */
    /* the control interface is turned on only */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_PSCTRL1, psctrl_value ) );

    /* Reset CODEC registers. */
    /* When running as I2S master, registers don't appear to get
     * set correctly when the CODEC is being configured (seen when moving between
     * 44.1kHz to 48kHz FS).
     */
    WICED_VERIFY( wm8533_reg_write(wm8533, WM8533_REG_CHIP_ID, 0 ) );

    /* set the format to I2S */
    //if( WICED_SUCCESS == result )
    //{
    //    result = wm8533_reg_write(0x03, ( 0x03 << 0 ) );
    //}

    /* set volume to approximately 8 db = 1b0, to both channels */
    /* where 190h is 0 db, and every 0.25db step is adding extra 1 */
    /* update immediately */
    /* (1 << DIGITAL_VOLUME_UPDATE_FIELD_SHIFT) | ( uint16_t )( 0x1B0 << DIGITAL_VOLUME_DB_VALUE ) */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_DAC_GAINL, (1 << DIGITAL_VOLUME_UPDATE_FIELD_SHIFT) | ( uint16_t )( 0x190 /*0x1B0*/ << DIGITAL_VOLUME_DB_VALUE ) ) );

    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_DAC_GAINR, (1 << DIGITAL_VOLUME_UPDATE_FIELD_SHIFT) | ( uint16_t )( 0x190 /*0x1B0*/ << DIGITAL_VOLUME_DB_VALUE ) ) );

    /* enable RAMP volume increase and decrease */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_DAC_CTRL3, (1 << VOLUME_DOWN_RAMP_FIELD) | ( 1 << VOLUME_UP_RAMP_FIELD_SHIFT ) ) );

    /* use zero cross */
    WICED_VERIFY( wm8533_reg_write(wm8533, WM8533_REG_DAC_CTRL3, (1 << DAC_ZERO_CROSS_FIELD_SHIFT ) ) );

    /* Set codec to power up mode with soft mute on */
    psctrl_value = ( SYS_ENA_POWER_UP_WITH_MUTE << SYS_ENA_FIELD_SHIFT );

    /* Second stage of powering the device up, SYS_ENA= 0x02 */
    /* all blocks are enabled, the digital soft mute (100 db attenuation) is turned on */
    WICED_VERIFY( wm8533_reg_write(wm8533, WM8533_REG_PSCTRL1, psctrl_value) );

    /* Set codec to power up mode */
    psctrl_value = ( SYS_ENA_POWER_UP << SYS_ENA_FIELD_SHIFT );
    /* Third stage of powering the device up, SYS_ENA= 0x03 */
    /* digital soft mute (100 db attenuation) is released, the device blocks are all powered up */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_PSCTRL1, psctrl_value ) );

    data_port->type       = WICED_I2S_SPDIF_MODE_OFF;
    data_port->port       = wm8533->data_port;
    data_port->channel    = WICED_PLAY_CHANNEL;

    return WICED_SUCCESS;
}

/* This function can only be called from the platform initialization routine */
wiced_result_t wm8533_device_register( wm8533_device_data_t* device_data, const platform_audio_device_id_t device_id )
{
    if( device_data == NULL )
    {
        return WICED_BADARG;
    }
    wm8533_interface.audio_device_driver_specific = device_data;
    wm8533_interface.device_id = device_id;

    /* Register a device to the audio device list and keep device data internally from this point */
    return wiced_register_audio_device(device_id, &wm8533_interface);
}

wiced_result_t wm8533_configure( void *driver_data, wiced_audio_config_t *config, uint32_t* mclk)
{
    wm8533_device_data_t* wm8533 = ( wm8533_device_data_t* )driver_data;
    wiced_result_t        result;
    uint16_t              ctrl1;
    uint16_t              ctrl2;
    size_t                i;
    wiced_bool_t          mclk_is_found = WICED_FALSE;
    uint32_t              mclk_lrck_ratio = 0;
    uint16_t              psctrl_value;

    /*
     * Configure platform.
     */

    if( *mclk == 0 )
    {
        switch (wm8533->fmt & WM8533_FMT_MASTER_MASK)
        {
            case WM8533_FMT_CCS_CFM:
                /* Codec is a master and generates i2s bit and lr clocks */
                /* Codec can be a master only when an external clock source is connected to it */
                /* Find a configuration that agrees with the platform. */
                for (i=0; i<sizeof(mclk_lrclk_map)/sizeof(mclk_lrclk_map[0]); i++)
                {
                    if (mclk_lrclk_map[i].fs == config->sample_rate)
                    {
                        /* Keep trying until the platform selects a valid configuration. */
                        result = wm8533_platform_configure(wm8533, mclk_lrclk_map[i].mclk, mclk_lrclk_map[i].fs, config->bits_per_sample);
                        if ( result == WICED_SUCCESS )
                        {
                            mclk_lrck_ratio = mclk_lrclk_map[i].sr;
                            mclk_is_found = WICED_TRUE;
                            break;
                        }
                    }
                }
                if( mclk_is_found == WICED_FALSE )
                {
                    return WICED_ERROR;
                }
                break;

            case WM8533_FMT_CCS_CFS:
                /* codec is a frame slave, I2S pll has been setup already  */

                break;
            default:
                /* Everything else is unsupported. */
                return WICED_UNSUPPORTED;
        }
    }
    else
    {
        result = WICED_UNSUPPORTED;
        for (i=0; i<sizeof(mclk_lrclk_map)/sizeof(mclk_lrclk_map[0]); i++)
        {
            if ( (mclk_lrclk_map[i].mclk == *mclk) && (config->sample_rate == mclk_lrclk_map[i].fs) )
            {
                result = WICED_SUCCESS;
                mclk_lrck_ratio = mclk_lrclk_map[i].sr;
                break;
            }
        }
        WICED_VERIFY( result );
    }

    /*
     * Configure CODEC.
     */

    /* Read AIF_CTRL1. */
    WICED_VERIFY( wm8533_reg_read( wm8533, WM8533_REG_AIF_CTRL1, &ctrl1 ) );

    /* Read AIF_CTRL2. */
    WICED_VERIFY( wm8533_reg_read( wm8533, WM8533_REG_AIF_CTRL2, &ctrl2 ) );

    /* Reset AIF_CTRL1. */
    ctrl1 &= ~(
                  AIF_FMT_BITS << AIF_FMT_SHIFT   |  /* Audio data interface format. */
                  AIF_MSTR_BITS << AIF_MSTR_SHIFT |  /* Master/slave select. */
                  AIF_WL_BITS << AIF_WL_SHIFT        /* Audio data word length. */
              );

    /* Reset AIF_CTRL2. */
    ctrl2 &= ~(
                  AIF_SR_BITS << AIF_SR_SHIFT           | /* Sample rate; MCLK:LRCLK ratio. */
                  AIF_BCLKDIV_BITS << AIF_BCLKDIV_SHIFT   /* Bit clock divider. */
              );

    /* Audio data interface format. */
    /* Must agree with I2S bus settings. */
    ctrl1 |= CODEC_STANDARD << AIF_FMT_SHIFT;

    switch ( wm8533->fmt & WM8533_FMT_MASTER_MASK )
    {
        case WM8533_FMT_CCS_CFM:
            /* CODEC is frame master. */
            ctrl1 |= AIF_MSTR_MASTER << AIF_MSTR_SHIFT;
            ctrl2 |= mclk_lrck_ratio << AIF_SR_SHIFT;
            break;
        case WM8533_FMT_CCS_CFS:
            /* CODEC is frame slave. */
            ctrl1 |= AIF_MSTR_SLAVE << AIF_MSTR_SHIFT;
            /* Auto detect ratio from master. */
            ctrl2 |= mclk_lrck_ratio << AIF_SR_SHIFT;
            break;
        default:
            /* Everything else is unsupported. */
            return WICED_UNSUPPORTED;
    }

    /* XXX Dependent on STM32F4x settings (guessed per section 27.4.4 STM32F4 reference manual).
     * FIXME Move me to platform!
    */
    switch ( config->bits_per_sample )
    {
        case 16:
            ctrl1 |= AIF_WL_16_BITS   << AIF_WL_SHIFT;
            ctrl2 |= AIF_BCLKDIV_32FS << AIF_BCLKDIV_SHIFT;
            break;
        case 24:
            ctrl1 |= AIF_WL_24_BITS   << AIF_WL_SHIFT;
            ctrl2 |= AIF_BCLKDIV_64FS << AIF_BCLKDIV_SHIFT;
            break;
        case 32:
            ctrl1 |= AIF_WL_32_BITS   << AIF_WL_SHIFT;
            ctrl2 |= AIF_BCLKDIV_64FS << AIF_BCLKDIV_SHIFT;
            break;
        default:
            return WICED_UNSUPPORTED;
    }

    /* Write AIF_CTRL1. */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_AIF_CTRL1, ctrl1 ) );

    /* Write AIF_CTRL2. */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_AIF_CTRL2, ctrl2 ) );

    psctrl_value = ( SYS_ENA_POWER_DOWN << SYS_ENA_FIELD_SHIFT );
    /* first stage, all analog block and digital signal blocks are powered off */
    /* the control interface is turned on only */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_PSCTRL1, psctrl_value ) );

    /* Set codec to power up mode with soft mute on */
    psctrl_value = ( SYS_ENA_POWER_UP_WITH_MUTE << SYS_ENA_FIELD_SHIFT );
    /* Second stage of powering the device up, SYS_ENA= 0x02 */
    /* all blocks are enabled, the digital soft mute (100 db attenuation) is turned on */
    WICED_VERIFY( wm8533_reg_write( wm8533, WM8533_REG_PSCTRL1, psctrl_value ) );

    /* Set codec to power up mode */
    psctrl_value = ( SYS_ENA_POWER_UP << SYS_ENA_FIELD_SHIFT );
    /* Third stage of powering the device up, SYS_ENA= 0x03 */
    /* digital soft mute (100 db attenuation) is released, the device blocks are all powered up */
    return wm8533_reg_write(wm8533, WM8533_REG_PSCTRL1, psctrl_value);
}

#ifdef NOTYET
wiced_result_t wiced_audio_device_param_set(wiced_audio_device_parameters_t param_id, void* param_data)
{
    UNUSED_PARAMETER(param_id);
    UNUSED_PARAMETER(param_data);

    return WICED_SUCCESS;
}


wiced_result_t wiced_audio_device_param_get(wiced_audio_device_parameters_t param_id, void* param_data)
{
    UNUSED_PARAMETER(param_id);
    UNUSED_PARAMETER(param_data);

    return WICED_SUCCESS;
}

wiced_result_t wiced_audio_device_reg_write(uint8_t reg, uint8_t value)
{
    /* this function is a wrapper for cs43l22_reg_write function */
    UNUSED_PARAMETER(reg);
    UNUSED_PARAMETER(value);

    return WICED_SUCCESS;
}


wiced_result_t wiced_audio_device_reg_read(uint8_t reg, uint8_t* value)
{
    /* this function is a wrapper for cs43l22_reg_read function */
    UNUSED_PARAMETER(reg);
    UNUSED_PARAMETER(value);

    return WICED_SUCCESS;
}
#endif /* NOTYET */

wiced_result_t wm8533_start_play ( void* driver_data )
{
    return WICED_SUCCESS;
}


wiced_result_t wm8533_stop_play ( void* driver_data )
{
    wm8533_device_data_t* wm8533 = ( wm8533_device_data_t* )driver_data;
    UNUSED_PARAMETER(wm8533);
    return WICED_SUCCESS;
}


wiced_result_t wm8533_deinit ( void* driver_data )
{
    wm8533_device_data_t* wm8533 = ( wm8533_device_data_t* )driver_data;

    /* XXX: assert for running state. */

    /* XXX: This procedure doesn't follow the suggested
     * power-down sequence specified in the datasheet.
     */

    /* Reset CODEC to initial state. */
//  result = wm8533_reset();

#ifdef NOTYET
    /* Power off CODEC. */
    WICED_VERIFY( wm8533_reg_write( 0x02, 0x01 ) );
#endif /* NOTYET */

    /* Clean-up configuration if applicable. */
    if ( give_more_samples_callback != NULL )
    {
        WICED_VERIFY( wiced_i2s_deinit( wm8533->data_port ) );

        give_more_samples_callback = NULL;
    }

    /* Don't deinitialize I2C since it might be used by other modules.
     * Diddo for IOE.
     */

    return WICED_SUCCESS;
}


/* Set of Low level DAC access API, will be called from wiced_audio_device_param_set */

static inline wiced_result_t wm8533_reg_write( wm8533_device_data_t* wm8533, uint8_t address, uint16_t reg_data )
{
    return wm8533_i2c_reg_write(wm8533->i2c_data, address, reg_data);
}

static inline wiced_result_t wm8533_reg_read(wm8533_device_data_t* wm8533, uint8_t address, uint16_t* reg_data)
{
    return wm8533_i2c_reg_read(wm8533->i2c_data, address, reg_data);
}

#ifdef NOTYET
static wiced_result_t wm8533_reg_read(wm8533_device_data_t* wm8533, uint8_t address, uint8_t* reg_data)
{
    return wm8533_i2c_reg_read(wm8533->i2c_data, address, reg_data);
}

static wiced_result_t cs43l22_dac_interface_set(wm8533_device_data_t* wm8533, cs43l22_dac_interface_t type)
{
    UNUSED_PARAMETER(wm8533);
    UNUSED_PARAMETER(type);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

static wiced_result_t cs43l22_pcm_channel_mute(wm8533_device_data_t* wm8533, cs43l22_pcm_channel_t channel, wiced_bool_t enable_disable)
{
    UNUSED_PARAMETER(wm8533);
    UNUSED_PARAMETER(channel);
    UNUSED_PARAMETER(enable_disable);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

static wiced_result_t cs43l22_pcm_channel_volume(wm8533_device_data_t* wm8533, cs43l22_pcm_channel_t channel, uint8_t volume)
{
    UNUSED_PARAMETER(wm8533);
    UNUSED_PARAMETER(channel);
    UNUSED_PARAMETER(volume);

    /* this function will call generic cs43l22 register write and read functions */


    return WICED_SUCCESS;
}
#endif /* NOTYET */

static wiced_result_t wm8533_set_volume( void* driver_data, double decibels )
{
    int                   db_steps;
    uint16_t              gain_reg_value;
    wm8533_device_data_t* wm8533 = (wm8533_device_data_t*)driver_data;

    /* set to max level if the value is very high */
    if( decibels > MAX_WM8533_DB_LEVEL )
    {
        decibels = MAX_WM8533_DB_LEVEL;
      /* set to minimum possible if the level is to low */
    }
    else if( decibels < MIN_WM8533_DB_LEVEL )
    {
        decibels = MIN_WM8533_DB_LEVEL;
    }

    /* get value which will be written to the DAC_GAINL and DAC_GAINR registers */
    if( decibels < 0 )
    {
        db_steps = (int)( (decibels * (-1.0)) / GAIN_ADJUSTMENT_STEP );
        gain_reg_value = GAIN_REG_VALUE_ON_ZERO_DB - db_steps;
    }
    else if( decibels == 0 )
    {
        gain_reg_value = GAIN_REG_VALUE_ON_ZERO_DB;
    }
    else
    {
        db_steps = (int)( (decibels / GAIN_ADJUSTMENT_STEP) );
        gain_reg_value = GAIN_REG_VALUE_ON_ZERO_DB + db_steps;
    }

    WICED_VERIFY( wm8533_reg_write(wm8533, WM8533_REG_DAC_GAINL, (1 << DIGITAL_VOLUME_UPDATE_FIELD_SHIFT) | ( uint16_t )( gain_reg_value  << DIGITAL_VOLUME_DB_VALUE ) ) );

    return wm8533_reg_write(wm8533, WM8533_REG_DAC_GAINR, (1 << DIGITAL_VOLUME_UPDATE_FIELD_SHIFT) | ( uint16_t )( gain_reg_value  << DIGITAL_VOLUME_DB_VALUE ) );
}

static wiced_result_t wm8533_get_volume_range( void* driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    UNUSED_PARAMETER( driver_data );

    *min_volume_in_decibels = MIN_WM8533_DB_LEVEL;
    *max_volume_in_decibels = MAX_WM8533_DB_LEVEL;

    return WICED_SUCCESS;
}

static wiced_result_t wm8533_ioctl(void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data)
{
    UNUSED_PARAMETER( driver_data );
    UNUSED_PARAMETER( cmd );
    UNUSED_PARAMETER( cmd_data );

    return WICED_UNSUPPORTED;
}

#ifdef NOTYET
wiced_result_t cs43l22_analog_input_enable(wiced_bool_t enable, cs43l22_analog_in_channel_t achannel)
{
    UNUSED_PARAMETER(enable);
    UNUSED_PARAMETER(achannel);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

wiced_result_t cs43l22_analog_input_volume(cs43l22_analog_in_channel_t achannel, uint8_t percentage)
{
    UNUSED_PARAMETER(achannel);
    UNUSED_PARAMETER(percentage);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

/* beep control api */
wiced_result_t cs43l22_beep_config(cs43l22_beep_params_t* bparam)
{
    UNUSED_PARAMETER(bparam);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

wiced_result_t cs43l22_beep_enable(wiced_bool_t enable_or_disable)
{
    UNUSED_PARAMETER(enable_or_disable);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

/* eq api, it will enable the equalizer module as soon as one of the eq APIs gets called */
wiced_result_t cs43l22_eq_bass_set(uint8_t gain, bass_corner_freq_t corner)
{
    UNUSED_PARAMETER(gain);
    UNUSED_PARAMETER(corner);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}
wiced_result_t cs43l22_eq_trebble_set(uint8_t gain, trebble_corner_freq_t corner)
{
    UNUSED_PARAMETER(gain);
    UNUSED_PARAMETER(corner);

    /* this function will call generic cs43l22 register write and read functions */

    return WICED_SUCCESS;
}

wiced_result_t cs43l22_hadphone_volume_set(uint8_t percent)
{
    UNUSED_PARAMETER(percent);

    /* this function will call generic cs43l22 register write and read functions */

    /* we will convert the percents to db internally */

    return WICED_SUCCESS;
}

wiced_result_t cs43l22_speaker_volume_set(uint8_t percent)
{
    UNUSED_PARAMETER(percent);

    /* this function will call generic cs43l22 register write and read functions */
    /* we will convert the percents to db internally */


    return WICED_SUCCESS;
}

/* limiter configuration */
wiced_result_t cs43l22_limiter_config(cs43l22_limiter_config_t* config)
{
#ifdef NOTYET
    wiced_result_t result;
    uint8_t tmp=0;

    switch(config->threshold)
    {
    case TH_0DB:
        tmp |= (uint8_t)0<<5;   /* 000 */
        break;
    case TH_M3DB:
        tmp |= (uint8_t)1<<5;   /* 001 */
        break;
    case TH_M6DB:
        tmp |= (uint8_t)2<<5;   /* 010 */
        break;
    case TH_M9DB:
        tmp |= (uint8_t)3<<5;   /* 011 */
        break;
    case TH_M12DB:
        tmp |= (uint8_t)4<<5;   /* 100 */
        break;
    case TH_M18DB:
        tmp |= (uint8_t)5<<5;   /* 101 */
        break;
    case TH_M24DB:
        tmp |= (uint8_t)6<<5;   /* 110 */
        break;
    default:
        result = WICED_BADARG;
    }
#endif

    return WICED_SUCCESS;
}

/* battery compensation parameters */
wiced_result_t cs43l22_battery_compensation_config( wiced_device_t* device, cs43l22_battery_t*config )
{
    UNUSED_PARAMETER(config);

    /* this function will call generic cs43l22 register write and read functions */
    /* we will convert the percents to db internally */

    return WICED_SUCCESS;
}

wiced_result_t cs43l22_battery_compensation_disable(void)
{
    /* this function will call generic cs43l22 register write and read functions */
    /* we will convert the percents to db internally */

    return WICED_SUCCESS;
}

/* status read */
wiced_result_t cs43l22_read_status(cs43l22_status_t* status)
{
    UNUSED_PARAMETER(status);
    /* this function will call generic cs43l22 register write and read functions */
    /* we will convert the percents to db internally */

    return WICED_SUCCESS;
}
#endif /* NOTYET */


/* Declare global audio device interface */

wiced_audio_device_interface_t wm8533_interface =
{
        .audio_device_init             = wm8533_init,
        .audio_device_deinit           = wm8533_deinit,
        .audio_device_configure        = wm8533_configure,
        .audio_device_start_streaming  = wm8533_start_play,
        .audio_device_stop_streaming   = wm8533_stop_play,
        .audio_device_set_volume       = wm8533_set_volume,
        .audio_device_set_treble       = NULL,
        .audio_device_set_bass         = NULL,
        .audio_device_get_volume_range = wm8533_get_volume_range,
        .audio_device_ioctl            = wm8533_ioctl,
};

