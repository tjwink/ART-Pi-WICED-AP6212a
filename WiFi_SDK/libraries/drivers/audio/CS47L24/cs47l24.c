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

#include <string.h>
#include <stdlib.h>

#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_audio.h"
#include "wiced_log.h"
#include "wwd_constants.h"
#include "wwd_assert.h"
#include "platform_i2s.h"

#include "cs47l24.h"
#include "cs47l24_private.h"
#include "cs47l24_register_map.h"


/******************************************************
 *                      Macros
 ******************************************************/

#define LOCK_RTD(rtd)   wiced_rtos_lock_mutex(&(rtd)->lock)
#define UNLOCK_RTD(rtd) wiced_rtos_unlock_mutex(&(rtd)->lock)

/******************************************************
 *                    Constants
 ******************************************************/

/* Pulse width for PDN reset. */
#define CS47L24_RESET_HOLD_TIME_IN_MILLIS    (1)

#define CS47L24_DUET_MCLK_1_HZ               (24576000)
#define CS47L24_DUET_MCLK_2_HZ               (32768)
#define CS47L24_SYSCLOCK_RATE_DEFAULT        (CS47L24_SYSCLOCK_147M456_135M4752)
#define CS47L24_ASYNC_CLOCK_RATE_DEFAULT     (CS47L24_SYSCLOCK_49M152_45M1584)

#define CS47L24_FRAC_SAMPLE_RATE_MIN         (44100 / 6)

#define CS47L24_ASRC_SAMPLE_RATE_MIN         (8000)
#define CS47L24_ASRC_SAMPLE_RATE_MAX         (48000)
#define CS47L24_ISRC_SAMPLE_RATE_MIN         (8000)
#define CS47L24_ISRC_SAMPLE_RATE_MAX         (192000)
#define CS47L24_ASRC_SAMPLE_RATE_MIN         (8000)
#define CS47L24_ASRC_SAMPLE_RATE_MAX         (48000)

#define CS47L24_DEVICE_SAMPLE_RATE_MAX       (5)

#define CS47L24_VOLUME_DB_MIN                (-64.0)
#define CS47L24_VOLUME_DB_MAX                (6.0)             /* CS47L24 can do +31.5dB but there's a limit set at 0.5 dbFS */
#define CS47L24_VOLUME_DB_STEP               (0.5)

#define CS47L24_MIXER_VOLUME_DB_MIN          (-32.0)
#define CS47L24_MIXER_VOLUME_DB_MAX          (16.0)
#define CS47L24_MIXER_VOLUME_DB_STEP         (1.0)
#define CS47L24_MIXER_VOLUME_INDEX_OFFSET    (0x20)

#define CS47L24_DMIC_HPF_CUTOFF_DEFAULT      (CS47L24_INPUT_HPF_CUTOFF_10_HZ)
#define CS47L24_DMIC_VOLUME_DEFAULT          (20.0)
#define CS47L24_DMIC_MIXER_VOLUME_DEFAULT    (0.0)
#define CS47L24_AIF_MIXER_VOLUME_DEFAULT     (0.0)
#define CS47L24_DMIC_CLOCK_DEFAULT           (CS47L24_DMIC_CLOCK_3M072)

/*
 * TODO: the 2 constants below are specific to 4390x audio engine
 * We need to make them part of the audio device
 */
#define CS47L24_4390X_BCLK_PER_LRCLK_FRAME   (64)
#define CS47L24_4390X_BCLK_PER_LRCLK_SLOT    (32)
#define CS47L24_4390X_AIF_FORMAT             (CS47L24_AIF_FORMAT_I2S)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct cs47l24_device_runtime_data   cs47l24_device_runtime_data_t;
typedef wiced_result_t (cs47l24_rfn_t)(cs47l24_device_cmn_data_t *, wiced_bool_t);

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CS47L24_DEVICE_TYPE_PLAYBACK      = 0,
    CS47L24_DEVICE_TYPE_CAPTURE,

    /* Not a device type! */
    CS47L24_DEVICE_TYPE_MAX,
} cs47l24_device_type_t;

typedef enum
{
    CS47L24_CLOCK_SOURCE_MCLK1        = 0x0,
    CS47L24_CLOCK_SOURCE_MCLK2        = 0x1,
    CS47L24_CLOCK_SOURCE_SYSCLK       = 0x2, /* only valid as a source for the 32 KHz CLOCK (CS47L24_CLOCK_32K_1 register) ! */
    CS47L24_CLOCK_SOURCE_FLL1         = 0x4,
    CS47L24_CLOCK_SOURCE_FLL2         = 0x5,
    CS47L24_CLOCK_SOURCE_AIF1BCLK     = 0x8,
    CS47L24_CLOCK_SOURCE_AIF2BCLK     = 0x9,
    CS47L24_CLOCK_SOURCE_AIF3BCLK     = 0xA,
    CS47L24_CLOCK_SOURCE_AIF1LRCLK    = 0xC,
    CS47L24_CLOCK_SOURCE_AIF2LRCLK    = 0xD,
    CS47L24_CLOCK_SOURCE_AIF3LRCLK    = 0xE,
} cs47l24_clock_source_t;

typedef enum
{
    CS47L24_SAMPLERATE_NONE           = 0x00,
    CS47L24_SAMPLERATE_12K            = 0x01,
    CS47L24_SAMPLERATE_24K            = 0x02,
    CS47L24_SAMPLERATE_48K            = 0x03,
    CS47L24_SAMPLERATE_96K            = 0x04,
    CS47L24_SAMPLERATE_192K           = 0x05,
    CS47L24_SAMPLERATE_11K025         = 0x09,
    CS47L24_SAMPLERATE_22K05          = 0x0A,
    CS47L24_SAMPLERATE_44K1           = 0x0B,
    CS47L24_SAMPLERATE_88K2           = 0x0C,
    CS47L24_SAMPLERATE_176K4          = 0x0D,
    CS47L24_SAMPLERATE_4K             = 0x10,
    CS47L24_SAMPLERATE_8K             = 0x11,
    CS47L24_SAMPLERATE_16K            = 0x12,
    CS47L24_SAMPLERATE_32K            = 0x13,
} cs47l24_samplerate_t;

typedef enum
{
    CS47L24_FREQ_FAMILY_48K           = 0x0,
    CS47L24_FREQ_FAMILY_44K1,
} cs47l24_frequency_family_t;

typedef enum
{
    CS47L24_FLL_OUTDIV_SYSCLK         = 0x2,
    CS47L24_FLL_OUTDIV_ASYNC_CLOCK    = 0x6,
} cs47l24_fll_outdiv_t;

typedef enum
{
    CS47L24_INPUT_HPF_CUTOFF_2_HZ_5   = 0x0,
    CS47L24_INPUT_HPF_CUTOFF_5_HZ,
    CS47L24_INPUT_HPF_CUTOFF_10_HZ,
    CS47L24_INPUT_HPF_CUTOFF_20_HZ,
    CS47L24_INPUT_HPF_CUTOFF_40_HZ,
} cs47l24_input_hpf_cutoff_t;

typedef enum
{
    CS47L24_VOL_RAMP_RATE_0_MSEC      = 0x0,
    CS47L24_VOL_RAMP_RATE_0_MSEC_5,
    CS47L24_VOL_RAMP_RATE_1_MSEC,
    CS47L24_VOL_RAMP_RATE_2_MSEC,
    CS47L24_VOL_RAMP_RATE_4_MSEC,
    CS47L24_VOL_RAMP_RATE_8_MSEC,
    CS47L24_VOL_RAMP_RATE_15_MSEC,
    CS47L24_VOL_RAMP_RATE_30_MSEC,
} cs47l24_volume_ramp_rate_t;

typedef enum
{
    CS47L24_DMIC_CLOCK_1M536          = 0x0, /* up to 20 KHz passband */
    CS47L24_DMIC_CLOCK_3M072,                /* up to 20 KHz passband */
    CS47L24_DMIC_CLOCK_6M144,                /* up to 96 KHz passband */
    CS47L24_DMIC_CLOCK_0M768,                /* up to  8 KHz passband */
} cs47l24_dmic_clock_frequency_t;

typedef enum
{
    CS47L24_DMIC_POWER_SUPPLY_MICVDD  = 0x0,
    CS47L24_DMIC_POWER_SUPPLY_MICBIAS1,
    CS47L24_DMIC_POWER_SUPPLY_MICBIAS2,
} cs47l24_dmic_power_supply_t;

typedef enum
{
    CS47L24_MIC_BIAS_1V5              = 0x0,
    CS47L24_MIC_BIAS_1V6,
    CS47L24_MIC_BIAS_1V7,
    CS47L24_MIC_BIAS_1V8,
    CS47L24_MIC_BIAS_1V9,
    CS47L24_MIC_BIAS_2V0,
    CS47L24_MIC_BIAS_2V1,
    CS47L24_MIC_BIAS_2V2,
    CS47L24_MIC_BIAS_2V3,
    CS47L24_MIC_BIAS_2V4,
    CS47L24_MIC_BIAS_2V5,
    CS47L24_MIC_BIAS_2V6,
    CS47L24_MIC_BIAS_2V7,
    CS47L24_MIC_BIAS_2V8,
} cs47l24_mic_bias_voltage_t;

typedef enum
{
    CS47L24_DCORE_INPUT_SILENCE       = 0x00,

    CS47L24_DCORE_INPUT_TONEGEN_1     = 0x04,
    CS47L24_DCORE_INPUT_TONEGEN_2     = 0x05,
    CS47L24_DCORE_INPUT_HAPTICGEN_1   = 0x06,

    CS47L24_DCORE_INPUT_AEC_LOOPBACK  = 0x08,

    CS47L24_DCORE_INPUT_NOISEGEN      = 0x0D,

    CS47L24_DCORE_INPUT_IN1L          = 0X10,
    CS47L24_DCORE_INPUT_IN1R          = 0X11,
    CS47L24_DCORE_INPUT_IN2L          = 0X12,
    CS47L24_DCORE_INPUT_IN2R          = 0X13,

    CS47L24_DCORE_INPUT_AIF1_RX1      = 0x20,
    CS47L24_DCORE_INPUT_AIF1_RX2      = 0x21,
    CS47L24_DCORE_INPUT_AIF1_RX3      = 0x22,
    CS47L24_DCORE_INPUT_AIF1_RX4      = 0x23,
    CS47L24_DCORE_INPUT_AIF1_RX5      = 0x24,
    CS47L24_DCORE_INPUT_AIF1_RX6      = 0x25,
    CS47L24_DCORE_INPUT_AIF1_RX7      = 0x26,
    CS47L24_DCORE_INPUT_AIF1_RX8      = 0x27,

    CS47L24_DCORE_INPUT_AIF2_RX1      = 0x28,
    CS47L24_DCORE_INPUT_AIF2_RX2      = 0x29,
    CS47L24_DCORE_INPUT_AIF2_RX3      = 0x2A,
    CS47L24_DCORE_INPUT_AIF2_RX4      = 0x2B,
    CS47L24_DCORE_INPUT_AIF2_RX5      = 0x2C,
    CS47L24_DCORE_INPUT_AIF2_RX6      = 0x2D,

    CS47L24_DCORE_INPUT_AIF3_RX1      = 0x30,
    CS47L24_DCORE_INPUT_AIF3_RX2      = 0x31,

    CS47L24_DCORE_INPUT_EQ1           = 0x50,
    CS47L24_DCORE_INPUT_EQ2           = 0x51,

    CS47L24_DCORE_INPUT_DRC1L         = 0x58,
    CS47L24_DCORE_INPUT_DRC1R         = 0x59,
    CS47L24_DCORE_INPUT_DRC2L         = 0x5A,
    CS47L24_DCORE_INPUT_DRC2R         = 0x5B,

    CS47L24_DCORE_INPUT_LHPF1         = 0x60,
    CS47L24_DCORE_INPUT_LHPF2,
    CS47L24_DCORE_INPUT_LHPF3,
    CS47L24_DCORE_INPUT_LHPF4,


    CS47L24_DCORE_INPUT_DSP2_CHANNEL1 = 0x70,
    CS47L24_DCORE_INPUT_DSP2_CHANNEL2,
    CS47L24_DCORE_INPUT_DSP2_CHANNEL3,
    CS47L24_DCORE_INPUT_DSP2_CHANNEL4,
    CS47L24_DCORE_INPUT_DSP2_CHANNEL5,
    CS47L24_DCORE_INPUT_DSP2_CHANNEL6,

    CS47L24_DCORE_INPUT_DSP3_CHANNEL1 = 0x78,
    CS47L24_DCORE_INPUT_DSP3_CHANNEL2,
    CS47L24_DCORE_INPUT_DSP3_CHANNEL3,
    CS47L24_DCORE_INPUT_DSP3_CHANNEL4,
    CS47L24_DCORE_INPUT_DSP3_CHANNEL5,
    CS47L24_DCORE_INPUT_DSP3_CHANNEL6,

    CS47L24_DCORE_INPUT_ASRC1L        = 0x90,
    CS47L24_DCORE_INPUT_ASRC1R,
    CS47L24_DCORE_INPUT_ASRC2L,
    CS47L24_DCORE_INPUT_ASRC2R,

    CS47L24_DCORE_INPUT_ISRC1_INT1    = 0xA0,
    CS47L24_DCORE_INPUT_ISRC1_INT2,
    CS47L24_DCORE_INPUT_ISRC1_INT3,
    CS47L24_DCORE_INPUT_ISRC1_INT4,
    CS47L24_DCORE_INPUT_ISRC1_DEC1,
    CS47L24_DCORE_INPUT_ISRC1_DEC2,
    CS47L24_DCORE_INPUT_ISRC1_DEC3,
    CS47L24_DCORE_INPUT_ISRC1_DEC4,

    CS47L24_DCORE_INPUT_ISRC2_INT1,
    CS47L24_DCORE_INPUT_ISRC2_INT2,
    CS47L24_DCORE_INPUT_ISRC2_INT3,
    CS47L24_DCORE_INPUT_ISRC2_INT4,
    CS47L24_DCORE_INPUT_ISRC2_DEC1,
    CS47L24_DCORE_INPUT_ISRC2_DEC2,
    CS47L24_DCORE_INPUT_ISRC2_DEC3,
    CS47L24_DCORE_INPUT_ISRC2_DEC4,

    CS47L24_DCORE_INPUT_ISRC3_INT1,
    CS47L24_DCORE_INPUT_ISRC3_INT2,
    CS47L24_DCORE_INPUT_ISRC3_INT3,
    CS47L24_DCORE_INPUT_ISRC3_INT4,
    CS47L24_DCORE_INPUT_ISRC3_DEC1,
    CS47L24_DCORE_INPUT_ISRC3_DEC2,
    CS47L24_DCORE_INPUT_ISRC3_DEC3,
    CS47L24_DCORE_INPUT_ISRC3_DEC4,

} cs47l24_digital_core_input_t;

typedef enum
{
    CS47L24_DCORE_SAMPLERATE_1        = 0x0,
    CS47L24_DCORE_SAMPLERATE_2        = 0x1,
    CS47L24_DCORE_SAMPLERATE_3        = 0x2,
    CS47L24_DCORE_ASYNC_SAMPLERATE_1  = 0x8,
    CS47L24_DCORE_ASYNC_SAMPLERATE_2  = 0x9,
} cs47l24_digital_core_samplerate_t;

typedef enum
{
    CS47L24_AIF_BCLK_64K_58K8         = 0x2,
    CS47L24_AIF_BCLK_96K_88K2,
    CS47L24_AIF_BCLK_128K_117K6,
    CS47L24_AIF_BCLK_192K_176K4,
    CS47L24_AIF_BCLK_256K_235K2,
    CS47L24_AIF_BCLK_384K_352K8,
    CS47L24_AIF_BCLK_512K_470K4,
    CS47L24_AIF_BCLK_768K_705K6,
    CS47L24_AIF_BCLK_1M024_940K8,
    CS47L24_AIF_BCLK_1M536_1M4112,
    CS47L24_AIF_BCLK_2M048_1M8816,
    CS47L24_AIF_BCLK_3M072_2M8824,
    CS47L24_AIF_BCLK_4M096_3M7632,
    CS47L24_AIF_BCLK_6M144_5M6448,
    CS47L24_AIF_BCLK_8M192_7M5264,
    CS47L24_AIF_BCLK_12M288_11M2896,
    CS47L24_AIF_BCLK_24M576_22M5792,
} cs47l24_aif_bitclock_frequency_t;

typedef enum
{
    CS47L24_AIF_FORMAT_DSP_MODE_A     = 0x0,
    CS47L24_AIF_FORMAT_DSP_MODE_B,
    CS47L24_AIF_FORMAT_I2S,
    CS47L24_AIF_FORMAT_LEFT_JUSTIFIED,
} cs47l24_aif_format_t;

typedef enum
{
    CS47L24_GPIO_NONE                 = 0x00,
    CS47L24_GPIO_CLASSIC              = 0x01,
    CS47L24_GPIO_IRQ1_OUT             = 0x02,
    CS47L24_GPIO_IRQ2_OUT             = 0x03,
    CS47L24_GPIO_SYSCLK_OUT           = 0x04,
    CS47L24_GPIO_FLL1_OUT             = 0x05,
    CS47L24_GPIO_FLL2_OUT             = 0x06,
    CS47L24_GPIO_PWM1_OUT             = 0x08,
    CS47L24_GPIO_PWM2_OUT             = 0x09,
    CS47L24_GPIO_ASYNC_SYSCLK_OUT     = 0x3D,
} cs47l24_gpio_function_t;

/******************************************************
 *                    Structures
 ******************************************************/

/* Codec device runtime information. */
struct cs47l24_device_runtime_data
{
    wiced_mutex_t   lock;

    /* This structure is initialized and ready for use. */
    uint8_t         rdy   : 1;

    uint8_t         dsp_prog : 1;

    uint8_t         need_isrc : 1;

    /* Playback/capture in-use flags. */
    uint8_t         init  : CS47L24_DEVICE_TYPE_MAX;
    uint8_t         cfg   : CS47L24_DEVICE_TYPE_MAX;

    /* Shared configuration (duplex). */
    uint8_t         bits_per_sample[CS47L24_DEVICE_TYPE_MAX];
    uint8_t         channels[CS47L24_DEVICE_TYPE_MAX];
    uint32_t        sample_rate[CS47L24_DEVICE_TYPE_MAX];
};

struct cs47l24_audio_device_interface
{
    cs47l24_device_type_t            type;
    wiced_audio_device_interface_t   adi;
};

struct cs47l24_device_route
{
    cs47l24_audio_device_interface_t *intf;
    cs47l24_rfn_t                    *fn;
};

typedef struct
{
    uint32_t addr;
    uint32_t data;
} cs47l24_spi_payload_t;

typedef struct
{
    uint32_t fsource;
    uint8_t  ref_clock_divider;
    uint8_t  fratio;
    uint8_t  gain;
    uint16_t n;
    uint16_t theta;
    uint16_t lambda;
} cs47l24_fll_register_settings_t;

typedef struct
{
    uint32_t             samplerate;
    cs47l24_samplerate_t sysclock_samplerate;
} cs47l24_sysclock_samplerate_t;

typedef struct
{
    uint32_t                         samplerate;
    cs47l24_aif_bitclock_frequency_t bitclock_rate;
} cs47l24_bitclock_rate_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static cs47l24_device_runtime_data_t device_runtime_table[CS47L24_MAX_DEVICES];

#ifdef CS47L24_ENABLE_FLL_SYNC_LOOP

static const cs47l24_fll_register_settings_t fll_main_48k_settings_table[] =
{
    {    32768, 0x0, 0xF, 0x0, /* 187.5    */ 0xBB, 0x8000, 0x002 },
};

static const cs47l24_fll_register_settings_t fll_sync_48k_settings_table[] =
{
    { 24576000, 0x1, 0x0, 0x4, /*  8.0000  */ 0x08,  0x000, 0x000 },
};

#else /* !CS47L24_ENABLE_FLL_SYNC_LOOP */

static const cs47l24_fll_register_settings_t fll_main_only_48k_settings_table[] =
{
    /*
     * Target FVC0 is 294.912MHz for 48KHz frequencies,
     * OUTDIV=2 for SYSCLOCK and OUTDIV=6 for ASYNC_SYSCLOCK,
     * FOUT = (FSOURCE / FREF Divider) * 3 * N.K * FRATIO / OUTDIV
     */
    {    32000, 0x0, 0xE, 0x0, /* 204.8    */ 0xCC, 0x004, 0x005 },
    {    32768, 0x0, 0xF, 0x0, /* 187.5    */ 0xBB, 0x001, 0x002 },
    {    48000, 0x0, 0xE, 0x0, /* 136.5333 */ 0x88, 0x008, 0x00F },
    {   128000, 0x0, 0x6, 0x0, /* 109.4173 */ 0x6D, 0x005, 0x007 },
    {   512000, 0x0, 0x4, 0x2, /* 38.4     */ 0x26, 0x002, 0x005 },
    {  1024000, 0x0, 0x4, 0x4, /* 19.2     */ 0x13, 0x001, 0x005 },
    {  1536000, 0x0, 0x2, 0x4, /* 21.3333  */ 0x15, 0x001, 0x003 },
    {  2048000, 0x0, 0x4, 0x4, /*  9.6     */ 0x09, 0x003, 0x005 },
    {  3072000, 0x0, 0x2, 0x4, /* 10.6667  */ 0x0A, 0x002, 0x003 },
    {  6144000, 0x0, 0x2, 0x4, /*  5.3333  */ 0x05, 0x001, 0x003 },
    { 11289600, 0x0, 0x0, 0x4, /*  8.7075  */ 0x08, 0x068, 0x093 },
    { 12000000, 0x0, 0x0, 0x4, /*  8.1920  */ 0x08, 0x018, 0x07D },
    { 12288000, 0x1, 0x2, 0x4, /*  5.3333  */ 0x05, 0x001, 0x003 },
    { 13000000, 0x0, 0x0, 0x4, /*  7.5618  */ 0x07, 0x391, 0x659 },
    { 19200000, 0x1, 0x0, 0x4, /* 10.24    */ 0x0A, 0x006, 0x019 },
    { 24000000, 0x1, 0x0, 0x4, /*  8.192   */ 0x08, 0x018, 0x07D },
    { 24576000, 0x2, 0x2, 0x4, /*  5.3333  */ 0x05, 0x001, 0x003 },
    { 26000000, 0x1, 0x0, 0x4, /*  7.5618  */ 0x07, 0x391, 0x659 },
    { 27000000, 0x1, 0x0, 0x4, /*  7.2818  */ 0x07, 0x13D, 0x465 },
};

static const cs47l24_fll_register_settings_t fll_main_only_44k1_settings_table[] =
{
    /*
     * Target FVC0 is 270.9504 MHz for 44.1KHz frequencies,
     * OUTDIV=2 for SYSCLOCK and OUTDIV=6 for ASYNC_SYSCLOCK,
     * FOUT = (FSOURCE / FREF Divider) * 3 * N.K * FRATIO / OUTDIV
     */
    {   705600, 0x0, 0x2, 0x2, /* 42.6667  */ 0x2A, 0x002, 0x003 },
    {  1411200, 0x0, 0x2, 0x4, /* 21.3333  */ 0x15, 0x001, 0x003 },
    {  2822400, 0x0, 0x2, 0x4, /* 10.6667  */ 0x0A, 0x002, 0x003 },
    {  5644800, 0x0, 0x2, 0x4, /*  5.3333  */ 0x05, 0x001, 0x003 },
    { 11289600, 0x1, 0x2, 0x4, /*  5.3333  */ 0x05, 0x001, 0x003 },
};

#endif /* CS47L24_ENABLE_FLL_SYNC_LOOP */

static const cs47l24_sysclock_samplerate_t sysclock_samplerate[] =
{
    {      0, CS47L24_SAMPLERATE_NONE      },
    {   4000, CS47L24_SAMPLERATE_4K        },
    {   8000, CS47L24_SAMPLERATE_8K        },
    {  11025, CS47L24_SAMPLERATE_11K025    },
    {  12000, CS47L24_SAMPLERATE_12K       },
    {  16000, CS47L24_SAMPLERATE_16K       },
    {  22050, CS47L24_SAMPLERATE_22K05     },
    {  24000, CS47L24_SAMPLERATE_24K       },
    {  32000, CS47L24_SAMPLERATE_32K       },
    {  44100, CS47L24_SAMPLERATE_44K1      },
    {  48000, CS47L24_SAMPLERATE_48K       },
    {  88200, CS47L24_SAMPLERATE_88K2      },
    {  96000, CS47L24_SAMPLERATE_96K       },
    { 176400, CS47L24_SAMPLERATE_176K4     },
    { 192000, CS47L24_SAMPLERATE_192K      },
};

static const cs47l24_bitclock_rate_t blck_rate_64fs[] =
{
    {   7350, CS47L24_AIF_BCLK_512K_470K4     },
    {   8000, CS47L24_AIF_BCLK_512K_470K4     },
    {  16000, CS47L24_AIF_BCLK_1M024_940K8    },
    {  22050, CS47L24_AIF_BCLK_1M536_1M4112   },
    {  24000, CS47L24_AIF_BCLK_1M536_1M4112   },
    {  32000, CS47L24_AIF_BCLK_2M048_1M8816   },
    {  44100, CS47L24_AIF_BCLK_3M072_2M8824   },
    {  48000, CS47L24_AIF_BCLK_3M072_2M8824   },
    {  88200, CS47L24_AIF_BCLK_6M144_5M6448   },
    {  96000, CS47L24_AIF_BCLK_6M144_5M6448   },
    { 176400, CS47L24_AIF_BCLK_12M288_11M2896 },
    { 192000, CS47L24_AIF_BCLK_12M288_11M2896 },
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t cs47l24_set_initial_clocking(cs47l24_device_data_t *dd);
static wiced_result_t cs47l24_enable_dsp(cs47l24_device_data_t *dd);
static wiced_result_t cs47l24_enable_dmic_route(cs47l24_device_data_t *dd);
static wiced_result_t cs47l24_disable_dmic_route(cs47l24_device_data_t *dd);
static wiced_result_t cs47l24_reset(cs47l24_device_cmn_data_t *cs47l24);

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t cs47l24_block_write(cs47l24_device_cmn_data_t* cs47l24, uint8_t* buffer, uint32_t buffer_length)
{
    wiced_result_t              result  = WICED_SUCCESS;
    wiced_spi_message_segment_t msg;

    if (buffer_length < 2)
    {
        result = WICED_ERROR;
        goto _exit;
    }

    msg.tx_buffer = buffer;
    msg.rx_buffer = NULL;
    msg.length    = buffer_length;

    result = wiced_spi_transfer( cs47l24->spi_data, &msg, 1 );
    if (result == WICED_SUCCESS)
    {
        // wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "write(0x%08lX,0x%08lX)=0x%08lX\n", ntohl(payload_send.addr), ntohl(payload_send.data), ntohl(payload_recv.data));
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_block_write(0x%02X%02X) fails !\n", buffer[0], buffer[1]);
    }
 _exit:
    return result;
}


wiced_result_t cs47l24_reg_write(cs47l24_device_cmn_data_t* cs47l24, uint32_t reg, uint16_t value)
{
    wiced_result_t              result  = WICED_SUCCESS;
    wiced_spi_message_segment_t msg;
    cs47l24_spi_payload_t       payload_send;
    cs47l24_spi_payload_t       payload_recv;

    memset(&payload_recv, 0, sizeof(payload_recv));

    payload_send.addr  = htonl(reg & 0x7FFFFFFF); /* Set MSB to 0 for write operations */
    payload_send.data  = value;
    payload_send.data =  htonl(payload_send.data);

    msg.tx_buffer = &payload_send;
    msg.rx_buffer = &payload_recv;
    msg.length    = sizeof(cs47l24_spi_payload_t);

    result = wiced_spi_transfer( cs47l24->spi_data, &msg, 1 );
    if (result == WICED_SUCCESS)
    {
        // wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "write(0x%08lX,0x%08lX)=0x%08lX\n", ntohl(payload_send.addr), ntohl(payload_send.data), ntohl(payload_recv.data));
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_reg_write(0x%08lX) fails !\n", ntohl(payload_send.addr));
    }
    return result;
}


wiced_result_t cs47l24_reg_read(cs47l24_device_cmn_data_t* cs47l24, uint32_t reg, uint16_t *value)
{
    wiced_result_t              result  = WICED_SUCCESS;
    wiced_spi_message_segment_t msg;
    cs47l24_spi_payload_t       payload_send;
    cs47l24_spi_payload_t       payload_recv;

    memset(&payload_recv, 0, sizeof(payload_recv));

    payload_send.addr  = htonl(reg | 0x80000000); /* Set MSB to 1 for read operations */
    payload_send.data  = 0;

    msg.tx_buffer = &payload_send;
    msg.rx_buffer = &payload_recv;
    msg.length    = sizeof(cs47l24_spi_payload_t);

    result = wiced_spi_transfer( cs47l24->spi_data, &msg, 1 );
    if (result == WICED_SUCCESS)
    {
        // wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "read(0x%08lX)=0x%08lX\n", 0x7FFFFFFF & ntohl(payload_send.addr), ntohl(payload_recv.data));
        *value = (uint16_t)(ntohl(payload_recv.data));
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_reg_read(0x%08lX) fails !\n", 0x7FFFFFFF & ntohl(payload_send.addr));
    }

    return result;
}


wiced_result_t cs47l24_upd_bits(cs47l24_device_cmn_data_t *cs47l24, uint32_t reg, uint16_t mask, uint16_t val)
{
    wiced_result_t result = WICED_SUCCESS;
    uint16_t       old    = 0;
    uint16_t       new    = 0;

    WICED_VERIFY_GOTO( cs47l24_reg_read( cs47l24, reg, &old ), result, _exit );

    new = ( old & ~mask ) | val;
    if ( new != old )
    {
        result = cs47l24_reg_write( cs47l24, reg, new );
    }
_exit:
    return result;
}


static inline cs47l24_device_type_t cs47l24_type(cs47l24_device_data_t *dd)
{
    return dd->route->intf->type;
}


static inline uint16_t cs47l24_percent_to_index(uint8_t percentage)
{
    uint16_t idx;

    if (percentage > 100)
    {
        percentage = 100;
    }

    idx = (uint16_t)((double)((CS47L24_VOLUME_DB_MAX - CS47L24_VOLUME_DB_MIN) / CS47L24_VOLUME_DB_STEP) * percentage);

    return idx;
}


/* Convert a dB range to a n-indexed integer range. */
static inline uint16_t cs47l24_double_range_with_offset_to_index(double volume_db, double volume_max, double volume_min, double volume_step, uint16_t index_offset)
{
    uint16_t idx;

    if (volume_db > volume_max)
    {
        volume_db = volume_max;
    }
    else if (volume_db < volume_min)
    {
        volume_db = volume_min;
    }

    idx  = (uint16_t)((double)((volume_db - volume_min) / volume_step));
    idx += index_offset;

    return idx;
}


static inline uint16_t cs47l24_dac_adc_volume_index(double volume_db)
{
    return cs47l24_double_range_with_offset_to_index(volume_db, CS47L24_VOLUME_DB_MAX, CS47L24_VOLUME_DB_MIN, CS47L24_VOLUME_DB_STEP, 0);
}


static inline uint16_t cs47l24_mixer_volume_index(double volume_db)
{
    return cs47l24_double_range_with_offset_to_index(volume_db, CS47L24_MIXER_VOLUME_DB_MAX, CS47L24_MIXER_VOLUME_DB_MIN, CS47L24_MIXER_VOLUME_DB_STEP, CS47L24_MIXER_VOLUME_INDEX_OFFSET);
}

/* System reset from datasheet. */
static wiced_result_t cs47l24_reset(cs47l24_device_cmn_data_t *cs47l24)
{
    wiced_result_t result           = WICED_SUCCESS;
    uint16_t       boot_done_status = 0;
    uint32_t       loop_count       = 0;

    if (cs47l24->pdn != WICED_GPIO_NONE)
    {
        /* Pulse PDN line. */
        WICED_VERIFY( wiced_gpio_output_low( cs47l24->pdn ) );
        WICED_VERIFY( wiced_rtos_delay_milliseconds( CS47L24_RESET_HOLD_TIME_IN_MILLIS ) );
        WICED_VERIFY( wiced_gpio_output_high( cs47l24->pdn ) );
    }

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &boot_done_status), result, _exit );
    loop_count = 0;
    while (((boot_done_status & CS47L24_BOOT_DONE_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &boot_done_status), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

    WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, CS47L24_SOFTWARE_RESET, 0x1), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &boot_done_status), result, _exit );
    loop_count = 0;
    while (((boot_done_status & CS47L24_BOOT_DONE_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &boot_done_status), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SOFTWARE_RESET, &boot_done_status), result, _exit );

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_reset: failed !");
    }
    return result;
}


static wiced_result_t cs47l24_set_initial_clocking(cs47l24_device_data_t *dd)
{
    wiced_result_t                   result              = WICED_SUCCESS;
    cs47l24_device_cmn_data_t*       cs47l24             = dd->cmn;
    uint32_t                         loop_count          = 0;
    uint16_t                         sysclock_disable;
    uint16_t                         input_disable;
    uint16_t                         output_disable;
    uint16_t                         fll_enable;
    uint16_t                         samplerate_status;
    cs47l24_fll_register_settings_t* settings_table;
    uint32_t                         settings_table_size;
    uint32_t                         i;
    uint32_t                         source_clock_rate;
    uint16_t                         fll_n;
    uint16_t                         fll_theta;
    uint16_t                         fll_lambda;
    uint16_t                         fll_fratio;
    uint16_t                         fll_outdiv;
    uint16_t                         fll_refclk_div;
    uint16_t                         fll_refclk_src;
    uint16_t                         fll_gain;
    uint32_t                         input_source_reg;

    /* Disable inputs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INPUT_ENABLES, CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK|CS47L24_IN2L_ENA_MASK|CS47L24_IN2R_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
    loop_count = 0;
    while (((input_disable & (CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK|CS47L24_IN2L_ENA_MASK|CS47L24_IN2R_ENA_MASK)) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: inputs disabled...\n");

    /* Disable outputs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_ENABLES_1, CS47L24_OUT4L_ENA_MASK|CS47L24_OUT1L_ENA_MASK|CS47L24_OUT1R_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_OUTPUT_STATUS_1, &output_disable), result, _exit );
    loop_count = 0;
    while (((output_disable & CS47L24_OUT4L_ENA_STS_MASK) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_OUTPUT_STATUS_1, &output_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: outputs disabled 1 ...\n");

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
    loop_count = 0;
    while (((output_disable & (CS47L24_OUT1L_ENA_STS_MASK|CS47L24_OUT1R_ENA_STS_MASK)) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: outputs disabled 2 ...\n");

    /* Silencing all input sources */
    for ( input_source_reg = CS47L24_AIF1TX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_AIF1TX8MIX_INPUT_4_SOURCE; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for ( input_source_reg = CS47L24_AIF2TX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_AIF2TX6MIX_INPUT_4_SOURCE; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for ( input_source_reg = CS47L24_AIF3TX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_AIF3TX2MIX_INPUT_4_SOURCE; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for ( input_source_reg = CS47L24_OUT1LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_OUT1RMIX_INPUT_4_SOURCE; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for ( input_source_reg = CS47L24_OUT4LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_OUT4LMIX_INPUT_4_SOURCE; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }

    /* Turn off 32K clock */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_CLOCK_32K_1, CS47L24_CLK_32K_ENA_MASK, 0), result, _exit);

    /* Turn off SYSCLOCK */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_SYSTEM_CLOCK, CS47L24_OPCLK_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SYSTEM_CLOCK_1, CS47L24_SYSCLK_ENA_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
    loop_count = 0;
    while (((sysclock_disable & CS47L24_SYSCLK_ENA_LOW_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: SYSCLOCK OFF ...\n");

    /* Turn off ASYNC_CLOCK */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_ASYNC_CLOCK, CS47L24_OPCLK_ASYNC_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_CLOCK_1, CS47L24_ASYNC_CLK_ENA_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
    loop_count = 0;
    while (((sysclock_disable & CS47L24_ASYNC_CLK_ENA_LOW_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: ASYNC_CLOCK OFF ...\n");

    /* Turn OFF both FLLs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_1, CS47L24_FLL1_ENA_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_1, CS47L24_FLL1_SYNC_ENA_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_1, CS47L24_FLL2_ENA_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_SYNCHRONISER_1, CS47L24_FLL2_SYNC_ENA_MASK, 0), result, _exit );

#ifdef CS47L24_ENABLE_FLL_SYNC_LOOP

    /* Program FLL1 settings for SYSCLK output of 48kHz frequency family */
    settings_table      = (cs47l24_fll_register_settings_t*) fll_sync_48k_settings_table;
    settings_table_size = ARRAY_SIZE(fll_sync_48k_settings_table);
    source_clock_rate   = CS47L24_DUET_MCLK_1_HZ;
    fll_refclk_src      = CS47L24_CLOCK_SOURCE_MCLK1;

    for (i = 0; i < settings_table_size; i++)
    {
        if (settings_table[i].fsource == source_clock_rate)
        {
            break;
        }
    }

    if (i >= settings_table_size)
    {
        result = WICED_ERROR;
        goto _exit;
    }

    fll_n          = settings_table[i].n;
    fll_theta      = settings_table[i].theta;
    fll_lambda     = settings_table[i].lambda;
    fll_fratio     = settings_table[i].fratio;
    fll_refclk_div = settings_table[i].ref_clock_divider;
    fll_gain       = settings_table[i].gain;

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_2, CS47L24_FLL1_SYNC_N_MASK,       fll_n          << CS47L24_FLL1_SYNC_N_SHIFT      ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_3, CS47L24_FLL1_SYNC_THETA_MASK,   fll_theta      << CS47L24_FLL1_SYNC_THETA_SHIFT  ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_4, CS47L24_FLL1_SYNC_LAMBDA_MASK,  fll_lambda     << CS47L24_FLL1_SYNC_LAMBDA_SHIFT ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_5, CS47L24_FLL1_SYNC_FRATIO_MASK,  fll_fratio     << CS47L24_FLL1_SYNC_FRATIO_SHIFT ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_6, CS47L24_FLL1_SYNC_CLK_DIV_MASK, fll_refclk_div << CS47L24_FLL1_SYNC_CLK_DIV_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_6, CS47L24_FLL1_SYNC_CLK_SRC_MASK, fll_refclk_src << CS47L24_FLL1_SYNC_CLK_SRC_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_7, CS47L24_FLL1_SYNC_GAIN_MASK,    fll_gain       << CS47L24_FLL1_SYNC_GAIN_SHIFT   ), result, _exit );
    /* wide bandwidth clocking */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_7, CS47L24_FLL1_SYNC_BW_MASK,      0x0                                              ), result, _exit );

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_SYNCHRONISER_1, CS47L24_FLL1_SYNC_ENA_MASK, CS47L24_FLL1_SYNC_ENA), result, _exit );

    settings_table      = (cs47l24_fll_register_settings_t*) fll_main_48k_settings_table;
    settings_table_size = ARRAY_SIZE(fll_main_48k_settings_table);
    source_clock_rate   = CS47L24_DUET_MCLK_2_HZ;
    fll_refclk_src      = CS47L24_CLOCK_SOURCE_MCLK2;

#else /* !CS47L24_ENABLE_FLL_SYNC_LOOP */

    settings_table      = (cs47l24_fll_register_settings_t*) fll_main_only_48k_settings_table;
    settings_table_size = ARRAY_SIZE(fll_main_only_48k_settings_table);
    source_clock_rate   = CS47L24_DUET_MCLK_1_HZ;
    fll_refclk_src      = CS47L24_CLOCK_SOURCE_MCLK1;

#endif /* CS47L24_ENABLE_FLL_SYNC_LOOP */

    for (i = 0; i < settings_table_size; i++)
    {
        if (settings_table[i].fsource == source_clock_rate)
        {
            break;
        }
    }

    if (i >= settings_table_size)
    {
        result = WICED_ERROR;
        goto _exit;
    }

    fll_n          = settings_table[i].n;
    fll_theta      = settings_table[i].theta;
    fll_lambda     = settings_table[i].lambda;
    fll_fratio     = settings_table[i].fratio;
    fll_outdiv     = CS47L24_FLL_OUTDIV_SYSCLK;
    fll_refclk_div = settings_table[i].ref_clock_divider;
    fll_gain       = settings_table[i].gain;

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_2, CS47L24_FLL1_N_MASK,           fll_n          << CS47L24_FLL1_N_SHIFT          ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_3, CS47L24_FLL1_THETA_MASK,       fll_theta      << CS47L24_FLL1_THETA_SHIFT      ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_4, CS47L24_FLL1_LAMBDA_MASK,      fll_lambda     << CS47L24_FLL1_LAMBDA_SHIFT     ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_5, CS47L24_FLL1_FRATIO_MASK,      fll_fratio     << CS47L24_FLL1_FRATIO_SHIFT     ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_5, CS47L24_FLL1_OUTDIV_MASK,      fll_outdiv     << CS47L24_FLL1_OUTDIV_SHIFT     ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_6, CS47L24_FLL1_CLK_REF_DIV_MASK, fll_refclk_div << CS47L24_FLL1_CLK_REF_DIV_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_6, CS47L24_FLL1_CLK_REF_SRC_MASK, fll_refclk_src << CS47L24_FLL1_CLK_REF_SRC_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_7, CS47L24_FLL1_GAIN_MASK,        fll_gain       << CS47L24_FLL1_GAIN_SHIFT       ), result, _exit );
    /* Disable free running mode ! */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_1, CS47L24_FLL1_FREERUN_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL1_CONTROL_1, CS47L24_FLL1_ENA_MASK, CS47L24_FLL1_ENA), result, _exit );

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &fll_enable), result, _exit );
    loop_count = 0;
    while (((fll_enable & CS47L24_FLL1_CLOCK_OK_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &fll_enable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: FLL1 is good to go ...\n");

    /*
     * Program SYSCLK settings
     * Hard-coded to 48K family of freq
     * For now, 16K, 24K and 48K
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SYSTEM_CLOCK_1, CS47L24_SYSCLK_FRAC_MASK, CS47L24_FREQ_FAMILY_48K           << CS47L24_SYSCLK_FRAC_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SYSTEM_CLOCK_1, CS47L24_SYSCLK_FREQ_MASK, CS47L24_SYSCLOCK_RATE_DEFAULT << CS47L24_SYSCLK_FREQ_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SYSTEM_CLOCK_1, CS47L24_SYSCLK_SRC_MASK,  CS47L24_CLOCK_SOURCE_FLL1         << CS47L24_SYSCLK_SRC_SHIFT ), result, _exit );

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SAMPLE_RATE_1, CS47L24_SAMPLE_RATE_1_MASK, CS47L24_SAMPLERATE_48K << CS47L24_SAMPLE_RATE_1_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SAMPLE_RATE_2, CS47L24_SAMPLE_RATE_2_MASK, CS47L24_SAMPLERATE_24K << CS47L24_SAMPLE_RATE_2_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SAMPLE_RATE_3, CS47L24_SAMPLE_RATE_3_MASK, CS47L24_SAMPLERATE_16K << CS47L24_SAMPLE_RATE_3_SHIFT), result, _exit );

    /* Turn SYSCLOCK ON */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_SYSTEM_CLOCK_1, CS47L24_SYSCLK_ENA_MASK, CS47L24_SYSCLK_ENA), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
    loop_count = 0;
    while (((sysclock_disable & CS47L24_SYSCLK_ENA_LOW_STS_MASK) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: SYSCLOCK is good to go...\n");

    /* Enable the 32K clock */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_CLOCK_32K_1, CS47L24_CLK_32K_SRC_MASK, CS47L24_CLOCK_SOURCE_SYSCLK << CS47L24_CLK_32K_SRC_SHIFT), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_CLOCK_32K_1, CS47L24_CLK_32K_ENA_MASK, CS47L24_CLK_32K_ENA                                     ), result, _exit);

#ifdef CS47L24_ENABLE_SYSCLK_OVER_GPIO

    /* Enable SYSCLK output over GPIO1 */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_DIR_MASK, CS47L24_GPIO_DIR_OUT), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_PUP_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_PDOWN_MASK, CS47L24_GPIO_PDOWN_ENA), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_POLARITY_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_OUT_CONF_MASK, CS47L24_GPIO_OUT_CONF_CMOS), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO1_CTRL, CS47L24_GPIO_FUNC_MASK, CS47L24_GPIO_SYSCLK_OUT << CS47L24_GPIO_FUNC_SHIFT), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_SYSTEM_CLOCK, CS47L24_OPCLK_DIV_MASK, 0x0 << CS47L24_OPCLK_DIV_SHIFT                            ), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_SYSTEM_CLOCK, CS47L24_OPCLK_SEL_MASK, CS47L24_SYSCLOCK_49M152_45M1584 << CS47L24_OPCLK_SEL_SHIFT), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_SYSTEM_CLOCK, CS47L24_OPCLK_ENA_MASK, CS47L24_OPCLK_ENA                                         ), result, _exit);

#endif /* CS47L24_ENABLE_SYSCLK_OVER_GPIO */

    /* Verify sample rate 1 through 3 */
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_1_STATUS, &samplerate_status), result, _exit );
    loop_count = 0;
    while ( ((samplerate_status & CS47L24_SAMPLE_RATE_1_STS_MASK) >> CS47L24_SAMPLE_RATE_1_STS_SHIFT) != CS47L24_SAMPLERATE_48K )
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_1_STATUS, &samplerate_status), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: SAMPLE_RATE 1 is OK !\n");

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_2_STATUS, &samplerate_status), result, _exit );
    loop_count = 0;
    while ( ((samplerate_status & CS47L24_SAMPLE_RATE_2_STS_MASK) >> CS47L24_SAMPLE_RATE_2_STS_SHIFT) != CS47L24_SAMPLERATE_24K )
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_2_STATUS, &samplerate_status), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: SAMPLE_RATE 2 is OK !\n");

    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_3_STATUS, &samplerate_status), result, _exit );
    loop_count = 0;
    while ( ((samplerate_status & CS47L24_SAMPLE_RATE_3_STS_MASK) >> CS47L24_SAMPLE_RATE_3_STS_SHIFT) != CS47L24_SAMPLERATE_16K )
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_SAMPLE_RATE_3_STATUS, &samplerate_status), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: SAMPLE_RATE 3 is OK !\n");

    /* Set input rate */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INPUT_RATE, CS47L24_IN_RATE_MASK, CS47L24_DCORE_SAMPLERATE_1 << CS47L24_IN_RATE_SHIFT), result, _exit);
    /* Set output rate */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_RATE_1, CS47L24_OUT_RATE_MASK, CS47L24_DCORE_SAMPLERATE_1 << CS47L24_OUT_RATE_SHIFT), result, _exit);

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_initial_clocking: IN and OUT rates are OK !\n");

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_set_initial_clocking: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_enable_dsp(cs47l24_device_data_t *dd)
{

    wiced_result_t                   result              = WICED_SUCCESS;
    cs47l24_device_cmn_data_t*       cs47l24             = dd->cmn;
    cs47l24_device_runtime_data_t*   rtd                 = &device_runtime_table[cs47l24->id];
    uint32_t                         input_source_reg;
    uint32_t                         i;

    /*
     * Setup GPIO2 for WUPD usage
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_DIR_MASK, CS47L24_GPIO_DIR_OUT), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_PUP_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_PDOWN_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_POLARITY_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_OUT_CONF_MASK, CS47L24_GPIO_OUT_CONF_CMOS), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_IN_DEBOUNCE_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_GPIO2_CTRL, CS47L24_GPIO_FUNC_MASK, CS47L24_GPIO_CLASSIC << CS47L24_GPIO_FUNC_SHIFT), result, _exit);

    /*
     * Make sure DSP inputs are all silenced
     */
    for (input_source_reg = CS47L24_DSP2LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_DSP2RMIX_INPUT_4_VOLUME; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for (input_source_reg = CS47L24_DSP2AUX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_DSP2AUX6MIX_INPUT_1_SOURCE; input_source_reg += 0x8)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for (input_source_reg = CS47L24_DSP3LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_DSP3RMIX_INPUT_4_VOLUME; input_source_reg += 0x2)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    for (input_source_reg = CS47L24_DSP3AUX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_DSP3AUX6MIX_INPUT_1_SOURCE; input_source_reg += 0x8)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }

    for (i = CS47L24_FIRMWARE_DSP2; i < CS47L24_FIRMWARE_DSP_MAX; i++)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[i].dsp_clocking1, CS47L24_DSP_CLK_SEL_MASK, g_cs47l24_dsp[i].dsp_clock_request << CS47L24_DSP_CLK_SEL_SHIFT), result, _exit);
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[i].dsp_control1, CS47L24_DSP_RATE_MASK, CS47L24_DCORE_SAMPLERATE_3 << CS47L24_DSP_RATE_SHIFT), result, _exit);
    }

    if (rtd->dsp_prog == 0)
    {
        WICED_VERIFY_GOTO( cs47l24_process_wmfw_blob(cs47l24), result, _exit);
        rtd->dsp_prog = 1;
    }

    /*
     * Connect DSP2 to DSP3 for WUPD
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP2AUX1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_DSP3_CHANNEL2 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_enable_dsp: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_disable_dmic_route(cs47l24_device_data_t *dd)
{
    wiced_result_t                   result              = WICED_SUCCESS;
    cs47l24_device_cmn_data_t*       cs47l24             = dd->cmn;
    uint32_t                         loop_count          = 0;
    uint16_t                         input_disable;
    /*
     * Mute DMIC first
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN1L_MUTE_MASK, CS47L24_IN1L_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN1R_MUTE_MASK, CS47L24_IN1R_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN_VU_MASK    , CS47L24_IN_VU    ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN_VU_MASK    , CS47L24_IN_VU    ), result, _exit );

    /* Disable inputs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INPUT_ENABLES, CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
    loop_count = 0;
    while (((input_disable & (CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK)) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

    /* Turn off MIC BIAS */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_ENA_MASK   , 0), result, _exit );

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_disable_dmic_route: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_enable_dmic_route(cs47l24_device_data_t *dd)
{
    wiced_result_t                   result              = WICED_SUCCESS;
    cs47l24_device_cmn_data_t*       cs47l24             = dd->cmn;
    cs47l24_device_runtime_data_t*   rtd                 = &device_runtime_table[cs47l24->id];
    uint32_t                         loop_count          = 0;
    uint16_t                         input_disable;
    uint32_t                         input_source_reg;
    uint16_t                         volume_default;
    uint16_t                         mixer_volume_dmic;
    uint16_t                         mixer_volume_aif2rx;

    /*
     * Setup high and low sample rates for ISRC 1 and ISRC 2
     */
    for (input_source_reg = CS47L24_ISRC1DEC1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_ISRC3INT4MIX_INPUT_1_SOURCE; input_source_reg += 0x8)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }

    /* ISRC1 for DMIC inputs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_1_CTRL_1, CS47L24_ISRC1_FSH_MASK, CS47L24_DCORE_SAMPLERATE_1 << CS47L24_ISRC1_FSH_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_1_CTRL_2, CS47L24_ISRC1_FSL_MASK, CS47L24_DCORE_SAMPLERATE_3 << CS47L24_ISRC1_FSL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_1_CTRL_3, CS47L24_ISRC1_NOTCH_ENA_MASK, CS47L24_ISRC1_NOTCH_ENA), result, _exit );
    /* ISRC2 for AIF2 RX */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_2_CTRL_1, CS47L24_ISRC2_FSH_MASK, CS47L24_DCORE_SAMPLERATE_1 << CS47L24_ISRC2_FSH_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_2_CTRL_2, CS47L24_ISRC2_FSL_MASK, CS47L24_DCORE_SAMPLERATE_3 << CS47L24_ISRC2_FSL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_2_CTRL_3, CS47L24_ISRC2_NOTCH_ENA_MASK, CS47L24_ISRC2_NOTCH_ENA), result, _exit );

    /*
     * Setup ASRC
     */
    for (input_source_reg = CS47L24_ASRC1LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_ASRC2RMIX_INPUT_1_SOURCE; input_source_reg += 0x8)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    }
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_RATE1, CS47L24_ASRC_RATE1_MASK, CS47L24_DCORE_SAMPLERATE_1 << CS47L24_ASRC_RATE1_SHIFT), result, _exit );

    /*
     * Mute DMIC first
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN1L_MUTE_MASK, CS47L24_IN1L_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN1R_MUTE_MASK, CS47L24_IN1R_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN_VU_MASK    , CS47L24_IN_VU    ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN_VU_MASK    , CS47L24_IN_VU    ), result, _exit );

    volume_default      = cs47l24_dac_adc_volume_index(CS47L24_DMIC_VOLUME_DEFAULT);
    mixer_volume_dmic   = cs47l24_mixer_volume_index(CS47L24_DMIC_MIXER_VOLUME_DEFAULT);
    mixer_volume_aif2rx = cs47l24_mixer_volume_index(-6.0); /* summing stereo signal into mono */

    /* MIC bias regulator / charge pump */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_ENA_MASK    , 0                                              ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_BYPASS_MASK , CS47L24_MICB1_BYPASS                           ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_DISCH_MASK  , CS47L24_MICB1_DISCH                            ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_RATE_MASK   , 0                                              ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_FAST_MASK   , 0                                              ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_LVL_MASK    , CS47L24_MIC_BIAS_2V2 << CS47L24_MICB1_LVL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_EXT_CAP_MASK, 0                                              ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_MIC_BIAS_CTRL_1, CS47L24_MICB1_ENA_MASK    , CS47L24_MICB1_ENA                              ), result, _exit );

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_HPF_CONTROL   , CS47L24_IN_HPF_CUT_MASK   , CS47L24_DMIC_HPF_CUTOFF_DEFAULT << CS47L24_IN_HPF_CUT_SHIFT     ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_IN1L_CONTROL  , CS47L24_IN1L_HPF_MASK     , CS47L24_IN1L_HPF_ENA                                            ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_IN1L_CONTROL  , CS47L24_IN1_OSR_MASK      , CS47L24_DMIC_CLOCK_DEFAULT << CS47L24_IN1_OSR_SHIFT             ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_IN1L_CONTROL  , CS47L24_IN1_DMIC_SUP_MASK , CS47L24_DMIC_POWER_SUPPLY_MICBIAS1 << CS47L24_IN1_DMIC_SUP_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DMIC1L_CONTROL, CS47L24_IN1_DMICL_DLY_MASK, 0 << CS47L24_IN1_DMICL_DLY_SHIFT                                ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_IN1R_CONTROL  , CS47L24_IN1R_HPF_MASK     , CS47L24_IN1R_HPF_ENA                                            ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DMIC1R_CONTROL, CS47L24_IN1_DMICR_DLY_MASK, 0 << CS47L24_IN1_DMICR_DLY_SHIFT                                ), result, _exit );

    /*
     * Set default input volume
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN1L_DIG_VOL_MASK, volume_default << CS47L24_IN1L_DIG_VOL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN1R_DIG_VOL_MASK, volume_default << CS47L24_IN1R_DIG_VOL_SHIFT), result, _exit );

    /*
     * Connect DMIC inputs to DSP3 through ISRC1
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC1DEC1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_IN1L << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC1DEC2MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_IN1R << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3RMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC1_DEC1 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3RMIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, mixer_volume_dmic << CS47L24_INPUT_VOLUME_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3AUX1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC1_DEC2 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );

    /*
     * Connect AIF2 RX to DSP3 through ISRC2 and ASRC
     * Connect HP output to ASRC SYSCLK output
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUT1LMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ASRC2L <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUT1RMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ASRC2R <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC2DEC1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ASRC2L << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC2DEC2MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ASRC2R << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3LMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC2_DEC1 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3LMIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, mixer_volume_aif2rx << CS47L24_INPUT_VOLUME_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3LMIX_INPUT_2_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC2_DEC2 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DSP3LMIX_INPUT_2_VOLUME, CS47L24_INPUT_VOLUME_MASK, mixer_volume_aif2rx << CS47L24_INPUT_VOLUME_SHIFT), result, _exit );

    /* Enable inputs */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INPUT_ENABLES, CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK, CS47L24_IN1L_ENA|CS47L24_IN1R_ENA), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
    loop_count = 0;
    while (((input_disable & (CS47L24_IN1L_ENA_MASK|CS47L24_IN1R_ENA_MASK)) != (CS47L24_IN1L_ENA|CS47L24_IN1R_ENA)) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INPUT_ENABLES_STATUS, &input_disable), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

    /* Enable ISRC1 DEC routes for WUPD */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_1_CTRL_3, CS47L24_ISRC1_DEC1_ENA_MASK|CS47L24_ISRC1_DEC2_ENA_MASK, CS47L24_ISRC1_DEC1_ENA|CS47L24_ISRC1_DEC2_ENA), result, _exit );
    /* Enable ISRC2 DEC routes for DSP3 AEC processing */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_2_CTRL_3, CS47L24_ISRC2_DEC1_ENA_MASK|CS47L24_ISRC2_DEC2_ENA_MASK, CS47L24_ISRC2_DEC1_ENA|CS47L24_ISRC2_DEC2_ENA), result, _exit );

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN1L_MUTE_MASK, 0            ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN1R_MUTE_MASK, 0            ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN_VU_MASK    , CS47L24_IN_VU), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN_VU_MASK    , CS47L24_IN_VU), result, _exit );

    /*
     * We want to flag the capture path as being initialized
     * But, not "configured" as of yet.
     */
    rtd->init |= (1 << CS47L24_DEVICE_TYPE_CAPTURE);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_enable_dmic_route: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_init(void *driver_data, wiced_audio_data_port_t* data_port)
{
    cs47l24_device_data_t         *dd      = ( cs47l24_device_data_t* )driver_data;
    cs47l24_device_cmn_data_t     *cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t *rtd     = &device_runtime_table[cs47l24->id];
    wiced_result_t                 result  = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Initialization is only required once. */
    if ( ((rtd->init & (1 << cs47l24_type(dd))) != 0) && (cs47l24_type(dd) != CS47L24_DEVICE_TYPE_CAPTURE) )
    {
        /* This device type is already initialized! */
        wiced_assert("already initialized", !(rtd->init & 1<<cs47l24_type(dd)));
        result = WICED_ERROR;
        goto cs47l24_init_unlock;
    }
    else if ( rtd->init != 0 )
    {
        goto already_initialized;
    }

    /* Initialize GPIOs. */
    if (cs47l24->pdn != WICED_GPIO_NONE)
    {
        WICED_VERIFY_GOTO( wiced_gpio_init( cs47l24->pdn, OUTPUT_PUSH_PULL ), result, cs47l24_init_unlock );
    }

    /* Enable I2C clocks, init I2C peripheral. */
    WICED_VERIFY_GOTO( wiced_spi_init( cs47l24->spi_data ), result, cs47l24_init_unlock );

    /* Reset to defaults. */
    WICED_VERIFY_GOTO( cs47l24_reset(cs47l24), result, cs47l24_init_unlock );

    /* Set initial clocking details */
    WICED_VERIFY_GOTO( cs47l24_set_initial_clocking(dd), result, cs47l24_init_unlock );
    /* Enable and program DSPs, if needed */
    WICED_VERIFY_GOTO( cs47l24_enable_dsp(dd), result, cs47l24_init_unlock );
    /* Enable DMIC input routing into DSPs for WUDP processing */
    WICED_VERIFY_GOTO( cs47l24_enable_dmic_route(dd), result, cs47l24_init_unlock );

    /* Reset DSP2 to ensure proper keyword trigger behavior */
    WICED_VERIFY_GOTO( cs47l24_dsp_core_reset(cs47l24, CS47L24_FIRMWARE_DSP2),  result, cs47l24_init_unlock );

already_initialized:
    /* Mark device as initialized. */
    rtd->init |= (1 << cs47l24_type(dd));

    data_port->type = PLATFORM_AUDIO_LINE;
    data_port->port = dd->data_port;
    switch (cs47l24_type(dd))
    {
        case CS47L24_DEVICE_TYPE_PLAYBACK:
            data_port->channel = WICED_PLAY_CHANNEL;
            break;

        case CS47L24_DEVICE_TYPE_CAPTURE:
            data_port->channel = WICED_RECORD_CHANNEL;
            break;

        default:
            result = WICED_ERROR;
            break;
    }

cs47l24_init_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


static wiced_result_t cs47l24_deinit(void *driver_data)
{
    cs47l24_device_data_t         *dd      = ( cs47l24_device_data_t* )driver_data;
    cs47l24_device_cmn_data_t     *cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t *rtd     = &device_runtime_table[cs47l24->id];
    wiced_result_t                 result  = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Power-down audio route. */
    if ( dd->route->fn != NULL )
    {
        WICED_VERIFY_GOTO( (*dd->route->fn)( cs47l24, WICED_FALSE ), result, cs47l24_deinit_unlock );
    }

    /* This audio route is no longer configured. */
    /* Remove in-use flags. */
    rtd->cfg  &= ~( 1 << cs47l24_type( dd ) );

    /*
     * We need to maintain the initialization of the capture route at ALL TIMES
     * Eventually, we would need to completely shut it down in order to save power
     * while we're not actually using WUPD
     */
    if (cs47l24_type( dd ) != CS47L24_DEVICE_TYPE_CAPTURE)
    {
        rtd->init &= ~( 1 << cs47l24_type( dd ) );
    }

    /*
     * No other initializations; clean-up.
     * However, we will never get there because we're maintaining the capture route at all times.
     * See comment above.
     */
    if ( ( rtd->init & ~( 1 << cs47l24_type( dd ) ) ) == 0 )
    {
        cs47l24_disable_dmic_route(dd);

        /* Power-down chip. */
        if (cs47l24->pdn != WICED_GPIO_NONE)
        {
            WICED_VERIFY_GOTO( wiced_gpio_output_low( cs47l24->pdn ), result, cs47l24_deinit_unlock );
        }
    }

    /* Don't deinitialize SPI since it might be used by other modules.
     * Diddo for IOE.
     */
 cs47l24_deinit_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


/* PLL Slave Mode. */
wiced_result_t cs47l24_spll(cs47l24_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t                   result               = WICED_SUCCESS;
    cs47l24_device_cmn_data_t*       cs47l24              = dd->cmn;
    cs47l24_device_runtime_data_t*   rtd                  = &device_runtime_table[cs47l24->id];
    uint32_t                         loop_count;
    uint16_t                         sysclock_disable;
    cs47l24_frequency_family_t       freq_family;
    cs47l24_fll_register_settings_t* settings_table;
    uint32_t                         settings_table_size;
    uint32_t                         i;
    uint32_t                         source_clock_rate;
    uint16_t                         fll_n;
    uint16_t                         fll_theta;
    uint16_t                         fll_lambda;
    uint16_t                         fll_fratio;
    uint16_t                         fll_outdiv;
    uint16_t                         fll_refclk_div;
    uint16_t                         fll_refclk_src;
    uint16_t                         fll_gain;
    uint16_t                         async_rate_1;
    uint16_t                         async_rate_2;
    uint32_t                         input_source_reg;

    UNUSED_PARAMETER(mclk);

    rtd->need_isrc = 0;

    freq_family = ((config->sample_rate % CS47L24_FRAC_SAMPLE_RATE_MIN) == 0) ? CS47L24_FREQ_FAMILY_44K1 : CS47L24_FREQ_FAMILY_48K;

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_spll: sample_rate=%lu, freq family=%d\n", config->sample_rate, freq_family);

    if (cs47l24_type(dd) == CS47L24_DEVICE_TYPE_PLAYBACK)
    {
        if ((config->sample_rate > CS47L24_ISRC_SAMPLE_RATE_MAX) || (config->sample_rate < CS47L24_ISRC_SAMPLE_RATE_MIN))
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_spll: sample rate %lu out of range [%lu-%lu]!\n",
                          config->sample_rate, CS47L24_ISRC_SAMPLE_RATE_MIN, CS47L24_ISRC_SAMPLE_RATE_MAX);
            goto _exit;
        }

        for (i = 0; i < ARRAY_SIZE(sysclock_samplerate); i++)
        {
            if (config->sample_rate == sysclock_samplerate[i].samplerate)
            {
                break;
            }
        }

        if (i >= ARRAY_SIZE(sysclock_samplerate))
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_spll: unsupported %lu sample rate !\n", config->sample_rate);
            goto _exit;
        }

        async_rate_2 = sysclock_samplerate[i].sysclock_samplerate;

        if (config->sample_rate > CS47L24_ASRC_SAMPLE_RATE_MAX)
        {
            rtd->need_isrc = 1;
        }

        /* Turn off ASYNC_CLOCK */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_ASYNC_CLOCK, CS47L24_OPCLK_ASYNC_ENA_MASK, 0), result, _exit);
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_CLOCK_1, CS47L24_ASYNC_CLK_ENA_MASK, 0), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );

        loop_count = 0;
        while (((sysclock_disable & CS47L24_ASYNC_CLK_ENA_LOW_STS_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
        {
            wiced_rtos_delay_milliseconds(1);
            WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_3, &sysclock_disable), result, _exit );
            loop_count++;
        }
        CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

        /* Turn off FLL2 */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_1, CS47L24_FLL2_ENA_MASK, 0), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_SYNCHRONISER_1, CS47L24_FLL2_SYNC_ENA_MASK, 0), result, _exit );

        /* Enable FLL2 based on incoming sample rate */
        if (freq_family == CS47L24_FREQ_FAMILY_48K)
        {
            settings_table      = (cs47l24_fll_register_settings_t*) fll_main_only_48k_settings_table;
            settings_table_size = ARRAY_SIZE(fll_main_only_48k_settings_table);
            async_rate_1        = CS47L24_SAMPLERATE_48K;
        }
        else
        {
            settings_table      = (cs47l24_fll_register_settings_t*) fll_main_only_44k1_settings_table;
            settings_table_size = ARRAY_SIZE(fll_main_only_44k1_settings_table);
            async_rate_1        = CS47L24_SAMPLERATE_44K1;
        }
        source_clock_rate = config->sample_rate * CS47L24_4390X_BCLK_PER_LRCLK_FRAME;
        fll_refclk_src    = CS47L24_CLOCK_SOURCE_AIF2BCLK;

        for (i = 0; i < settings_table_size; i++)
        {
            if (settings_table[i].fsource == source_clock_rate)
            {
                break;
            }
        }

        if (i >= settings_table_size)
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_spll: unsupported BCLK rate !\n");
            goto _exit;
        }

        fll_n          = settings_table[i].n;
        fll_theta      = settings_table[i].theta;
        fll_lambda     = settings_table[i].lambda;
        fll_fratio     = settings_table[i].fratio;
        fll_outdiv     = CS47L24_FLL_OUTDIV_ASYNC_CLOCK;
        fll_refclk_div = settings_table[i].ref_clock_divider;
        fll_gain       = settings_table[i].gain;

        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_2, CS47L24_FLL2_N_MASK,           fll_n          << CS47L24_FLL2_N_SHIFT          ), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_3, CS47L24_FLL2_THETA_MASK,       fll_theta      << CS47L24_FLL2_THETA_SHIFT      ), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_4, CS47L24_FLL2_LAMBDA_MASK,      fll_lambda     << CS47L24_FLL2_LAMBDA_SHIFT     ), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_5, CS47L24_FLL2_FRATIO_MASK,      fll_fratio     << CS47L24_FLL2_FRATIO_SHIFT     ), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_5, CS47L24_FLL2_OUTDIV_MASK,      fll_outdiv     << CS47L24_FLL2_OUTDIV_SHIFT     ), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_6, CS47L24_FLL2_CLK_REF_DIV_MASK, fll_refclk_div << CS47L24_FLL2_CLK_REF_DIV_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_6, CS47L24_FLL2_CLK_REF_SRC_MASK, fll_refclk_src << CS47L24_FLL2_CLK_REF_SRC_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_7, CS47L24_FLL2_GAIN_MASK,        fll_gain       << CS47L24_FLL2_GAIN_SHIFT       ), result, _exit );
        /* Disable free running mode ! */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_1, CS47L24_FLL2_FREERUN_MASK, 0 /*CS47L24_FLL2_FREERUN*/), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_FLL2_CONTROL_1, CS47L24_FLL2_ENA_MASK, CS47L24_FLL2_ENA), result, _exit );

        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_CLOCK_1, CS47L24_ASYNC_CLK_FREQ_MASK, CS47L24_ASYNC_CLOCK_RATE_DEFAULT << CS47L24_ASYNC_CLK_FREQ_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_CLOCK_1, CS47L24_ASYNC_CLK_SRC_MASK , CS47L24_CLOCK_SOURCE_FLL2        << CS47L24_ASYNC_CLK_SRC_SHIFT ), result, _exit );

        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_SAMPLE_RATE_1, CS47L24_ASYNC_SAMPLE_RATE_1_MASK, async_rate_1 << CS47L24_ASYNC_SAMPLE_RATE_1_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_SAMPLE_RATE_2, CS47L24_ASYNC_SAMPLE_RATE_2_MASK, async_rate_2 << CS47L24_ASYNC_SAMPLE_RATE_2_SHIFT), result, _exit );

        /* Turn ASYNC CLOCK ON */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASYNC_CLOCK_1, CS47L24_ASYNC_CLK_ENA_MASK, CS47L24_ASYNC_CLK_ENA), result, _exit );

        /* AIF2 uses ASYNC sample rate 2 */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RATE_CTRL, CS47L24_AIF2_RATE_MASK, CS47L24_DCORE_ASYNC_SAMPLERATE_2 << CS47L24_AIF2_RATE_SHIFT), result, _exit);

        /* Silence all ASRC inputs */
        for (input_source_reg = CS47L24_ASRC2LMIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_ASRC2RMIX_INPUT_1_SOURCE; input_source_reg += 0x8)
        {
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
        }

        /* Silence all ISRC3 inputs */
        for (input_source_reg = CS47L24_ISRC3DEC1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_ISRC3INT4MIX_INPUT_1_SOURCE; input_source_reg += 0x8)
        {
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
        }

        if (rtd->need_isrc)
        {
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_RATE2, CS47L24_ASRC_RATE2_MASK, CS47L24_DCORE_ASYNC_SAMPLERATE_1 << CS47L24_ASRC_RATE2_SHIFT), result, _exit );
            /* Enable ASRC routes */
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC2LMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC3_DEC1 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC2RMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_ISRC3_DEC2 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_ENABLE, CS47L24_ASRC2L_ENA_MASK|CS47L24_ASRC2R_ENA_MASK, CS47L24_ASRC2L_ENA|CS47L24_ASRC2R_ENA), result, _exit );

            /* ISRC3 decimates from ASYNC sample rate 2 (anything above 48KHz) to sample rate 1 (either 44.1KHz or 48KHz) */
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_1, CS47L24_ISRC3_FSH_MASK, CS47L24_DCORE_ASYNC_SAMPLERATE_2 << CS47L24_ISRC3_FSH_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_2, CS47L24_ISRC3_FSL_MASK, CS47L24_DCORE_ASYNC_SAMPLERATE_1 << CS47L24_ISRC3_FSL_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_3, CS47L24_ISRC3_NOTCH_ENA_MASK, CS47L24_ISRC3_NOTCH_ENA), result, _exit );
        }
        else
        {
            /* Make sure ISRC3 is disabled (from previous run) */
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_3, CS47L24_ISRC3_DEC1_ENA_MASK|CS47L24_ISRC3_DEC2_ENA_MASK, 0), result, _exit );

            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_RATE2, CS47L24_ASRC_RATE2_MASK, CS47L24_DCORE_ASYNC_SAMPLERATE_2 << CS47L24_ASRC_RATE2_SHIFT), result, _exit );
        }
    }
    else if (cs47l24_type(dd) == CS47L24_DEVICE_TYPE_CAPTURE)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RATE_CTRL, CS47L24_AIF1_RATE_MASK, CS47L24_DCORE_SAMPLERATE_3 << CS47L24_AIF1_RATE_SHIFT), result, _exit);
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_spll: failed !\n");
    }
    return result;
}


/* PLL Master Mode. */
wiced_result_t cs47l24_mpll(cs47l24_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t result = WICED_SUCCESS;
    cs47l24_device_cmn_data_t *cs47l24 = dd->cmn;

    UNUSED_PARAMETER(config);
    UNUSED_PARAMETER(mclk);
    UNUSED_VARIABLE(cs47l24);

    return result;
}


wiced_result_t cs47l24_ext(cs47l24_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t result = WICED_SUCCESS;
    cs47l24_device_cmn_data_t *cs47l24 = dd->cmn;

    UNUSED_PARAMETER(config);
    UNUSED_PARAMETER(mclk);
    UNUSED_VARIABLE(cs47l24);

    return result;
}




static wiced_result_t cs47l24_clock(cs47l24_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t             result           = WICED_SUCCESS;
    cs47l24_device_cmn_data_t* cs47l24          = dd->cmn;
    uint32_t                   input_source_reg;
    cs47l24_bitclock_rate_t*   bclk_rate_table;
    uint32_t                   bclk_rate_table_size;
    uint32_t                   fs_index;
    uint16_t                   bclk_rate        = 0;
    uint16_t                   word_length      = config->bits_per_sample;

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_clock: fs=%lu,sample=%hu,channels=%u\n", config->sample_rate, word_length, config->channels);

    /*
     * Pick BCLK samplerate; assuming here that it's 4390x I2S engine h/w with bclk_rate = 2 x 32-bit x sample_rate
     */
    bclk_rate_table      = (cs47l24_bitclock_rate_t*)blck_rate_64fs;
    bclk_rate_table_size = ARRAY_SIZE(blck_rate_64fs);

    for (fs_index = 0; fs_index < bclk_rate_table_size; fs_index++)
    {
        if (bclk_rate_table[fs_index].samplerate == config->sample_rate)
        {
            break;
        }
    }

    if (fs_index >= bclk_rate_table_size)
    {
        result = WICED_ERROR;
        goto _exit;
    }

    bclk_rate = bclk_rate_table[fs_index].bitclock_rate;

    /*
     * Make sure sources register are set to Mute/Silence as we're picking a sample rate
     */
    if (cs47l24_type(dd) == CS47L24_DEVICE_TYPE_PLAYBACK)
    {
        for ( input_source_reg = CS47L24_AIF2TX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_AIF2TX6MIX_INPUT_4_SOURCE; input_source_reg += 0x2)
        {
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
        }

        /* Disable AIF2 TX/RX before configuring */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_TX_ENABLES, CS47L24_AIF2TX_ALL_ENA_MASK, 0), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_ENABLES, CS47L24_AIF2RX_ALL_ENA_MASK, 0), result, _exit );

        /* BCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_FRC_MASK, 0), result, _exit );
        /* LRCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_FRC_MASK, 0), result, _exit );

        /* Using regular I2S format */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_FORMAT, CS47L24_AIF2_FMT_MASK, CS47L24_4390X_AIF_FORMAT << CS47L24_AIF2_FMT_SHIFT), result, _exit );

        /* Number of BCLK cycles per LRCLK frame (frame is comprised of multiple audio samples depending on number of channels) */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_BCLK_RATE, CS47L24_AIF2RX_BCPF_MASK, CS47L24_4390X_BCLK_PER_LRCLK_FRAME << CS47L24_AIF2RX_BCPF_SHIFT), result, _exit );

        /* Number of valid bits per LRCLK slot */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_FRAME_CTRL_1, CS47L24_AIF2TX_WL_MASK, word_length << CS47L24_AIF2TX_WL_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_FRAME_CTRL_2, CS47L24_AIF2RX_WL_MASK, word_length << CS47L24_AIF2RX_WL_SHIFT), result, _exit );

        /* Number of BCLK cycles per LRCLK slot (one slot represent one channel)*/
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_FRAME_CTRL_1, CS47L24_AIF2TX_SLOT_LEN_MASK, CS47L24_4390X_BCLK_PER_LRCLK_SLOT << CS47L24_AIF2TX_SLOT_LEN_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_FRAME_CTRL_2, CS47L24_AIF2RX_SLOT_LEN_MASK, CS47L24_4390X_BCLK_PER_LRCLK_SLOT << CS47L24_AIF2RX_SLOT_LEN_SHIFT), result, _exit );

        /* BCLK signal is NOT INVERTED */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_BCLK_CTRL, CS47L24_AIF2_BCLK_INV_MASK, 0), result, _exit );
        /* BCLK is only clocking with data */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_BCLK_CTRL, CS47L24_AIF2_BCLK_FRC_MASK, 0), result, _exit );
        /* CS47L24 will be I2S slave and WICED audio engine will be I2S master */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_BCLK_CTRL, CS47L24_AIF2_BCLK_MSTR_MASK, 0), result, _exit );

        /* LRCLK signal is NOT INVERTED */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_PIN_CTRL, CS47L24_AIF2RX_LRCLK_INV_MASK, 0), result, _exit );
        /* LRCLK is only clocking with data */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_PIN_CTRL, CS47L24_AIF2RX_LRCLK_FRC_MASK, 0), result, _exit );
        /* CS47L24 will be I2S slave and WICED audio engine will be I2S master */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_PIN_CTRL, CS47L24_AIF2RX_LRCLK_MSTR_MASK, 0), result, _exit );

        /* Program BCLK rate based on sample rate and I2S format */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_BCLK_CTRL, CS47L24_AIF2_BCLK_FREQ_MASK, bclk_rate << CS47L24_AIF2_BCLK_FREQ_SHIFT), result, _exit );
    }
    else if (cs47l24_type(dd) == CS47L24_DEVICE_TYPE_CAPTURE)
    {
        WICED_VERIFY_GOTO( cs47l24_dsp_wupd_irq_reset(cs47l24), result, _exit );

        for ( input_source_reg = CS47L24_AIF1TX1MIX_INPUT_1_SOURCE; input_source_reg <= CS47L24_AIF1TX8MIX_INPUT_4_SOURCE; input_source_reg += 0x2)
        {
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, input_source_reg, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_SILENCE <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
        }

        /* Disable AIF1 TX/RX before configuring */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_TX_ENABLES, CS47L24_AIF1TX_ALL_ENA_MASK, 0), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_ENABLES, CS47L24_AIF1RX_ALL_ENA_MASK, 0), result, _exit );

        /* BCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_FRC_MASK, 0), result, _exit );
        /* LRCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_FRC_MASK, 0), result, _exit );

        /* Using regular I2S format */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_FORMAT, CS47L24_AIF1_FMT_MASK, CS47L24_4390X_AIF_FORMAT << CS47L24_AIF1_FMT_SHIFT), result, _exit );

        /* Number of BCLK cycles per LRCLK frame (frame is comprised of multiple audio samples depending on number of channels) */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_BCLK_RATE, CS47L24_AIF1RX_BCPF_MASK, CS47L24_4390X_BCLK_PER_LRCLK_FRAME << CS47L24_AIF1RX_BCPF_SHIFT), result, _exit );
        /* Number of valid bits per LRCLK slot */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_FRAME_CTRL_1, CS47L24_AIF1TX_WL_MASK, word_length << CS47L24_AIF1TX_WL_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_FRAME_CTRL_2, CS47L24_AIF1RX_WL_MASK, word_length << CS47L24_AIF1RX_WL_SHIFT), result, _exit );
        /* Number of BCLK cycles per LRCLK slot (one slot represent one channel)*/
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_FRAME_CTRL_1, CS47L24_AIF1TX_SLOT_LEN_MASK, CS47L24_4390X_BCLK_PER_LRCLK_SLOT << CS47L24_AIF1TX_SLOT_LEN_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_FRAME_CTRL_2, CS47L24_AIF1RX_SLOT_LEN_MASK, CS47L24_4390X_BCLK_PER_LRCLK_SLOT << CS47L24_AIF1RX_SLOT_LEN_SHIFT), result, _exit );

        /* BCLK signal is NOT INVERTED */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_INV_MASK, 0), result, _exit );
        /* CS47L24 will be I2S master and WICED audio engine will be I2S slave */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_MSTR_MASK, CS47L24_AIF1_BCLK_MSTR), result, _exit );

        /* LRCLK signal is NOT INVERTED */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_INV_MASK, 0), result, _exit );
        /* CS47L24 will be I2S master and WICED audio engine will be I2S slave */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_MSTR_MASK, CS47L24_AIF1RX_LRCLK_MSTR), result, _exit );

        /* Program BCLK rate based on sample rate and I2S format */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_FREQ_MASK, bclk_rate << CS47L24_AIF1_BCLK_FREQ_SHIFT), result, _exit );

        /* BCLK is clocking ALL THE TIME even with data */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_FRC_MASK, CS47L24_AIF1_BCLK_FRC), result, _exit );
        /* LRCLK is clocking ALL THE TIME even with data */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_FRC_MASK, CS47L24_AIF1RX_LRCLK_FRC), result, _exit );
    }

    /* Clock configuration. */
    if ( cs47l24->ck.fn != NULL)
    {
        WICED_VERIFY_GOTO( (*cs47l24->ck.fn)(dd, config, mclk), result, _exit );
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_clock: failed !\n");
    }
    return result;
}


static inline wiced_result_t cs47l24_set_clock_config(cs47l24_device_data_t *dd, const wiced_audio_config_t *config)
{
    cs47l24_device_cmn_data_t*     cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t* rtd     = &device_runtime_table[cs47l24->id];

    rtd->bits_per_sample[cs47l24_type(dd)] = config->bits_per_sample;
    rtd->channels[cs47l24_type(dd)]        = config->channels;
    rtd->sample_rate[cs47l24_type(dd)]     = config->sample_rate;

    return WICED_SUCCESS;
}


wiced_result_t cs47l24_dmic(cs47l24_device_cmn_data_t *cs47l24, wiced_bool_t is_pwr_up)
{
    wiced_result_t result     = WICED_SUCCESS;

    /*
     * Mute AIF1 TX mixer inputs
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX1MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, 0), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX2MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, 0), result, _exit );

    if (is_pwr_up == WICED_TRUE)
    {
        /*
         * Connect DSP3 channel 2 output directly to AIF1 TX pins (dual mono signal)
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_DSP3_CHANNEL2 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX2MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_DSP3_CHANNEL2 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );

        /*
         * Enable AIF1 TX pins
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_TX_ENABLES, CS47L24_AIF1TX1_ENA_MASK|CS47L24_AIF1TX2_ENA_MASK, CS47L24_AIF1TX1_ENA|CS47L24_AIF1TX2_ENA), result, _exit );
    }
    else
    {
        /*
         * Disable AIF1 TX pins
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_TX_ENABLES, CS47L24_AIF1TX1_ENA_MASK|CS47L24_AIF1TX2_ENA_MASK, 0), result, _exit );
        /* BCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_BCLK_CTRL, CS47L24_AIF1_BCLK_FRC_MASK, 0), result, _exit );
        /* LRCLK should stop clocking */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1_RX_PIN_CTRL, CS47L24_AIF1RX_LRCLK_FRC_MASK, 0), result, _exit );
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dmic: failed !\n");
    }
    return result;
}


wiced_result_t cs47l24_hp(cs47l24_device_cmn_data_t *cs47l24, wiced_bool_t is_pwr_up)
{
    wiced_result_t                 result         = WICED_SUCCESS;
    cs47l24_device_runtime_data_t* rtd            = &device_runtime_table[cs47l24->id];
    uint32_t                       loop_count     = 0;
    uint16_t                       output_disable;

    /*
     * Mute first
     */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT1L_MUTE_MASK, CS47L24_OUT1L_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT1R_MUTE_MASK, CS47L24_OUT1R_MUTE), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU    ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU    ), result, _exit );

    if (is_pwr_up == WICED_TRUE)
    {
        /* Enable HP outputs */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_ENABLES_1, CS47L24_OUT1L_ENA_MASK|CS47L24_OUT1R_ENA_MASK, CS47L24_OUT1L_ENA|CS47L24_OUT1R_ENA), result, _exit);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
        loop_count = 0;
        while (((output_disable & (CS47L24_OUT1L_ENA_STS_MASK|CS47L24_OUT1R_ENA_STS_MASK)) != (CS47L24_OUT1L_ENA_STS|CS47L24_OUT1R_ENA_STS)) && (loop_count < CS47L24_READ_RETRY_COUNT))
        {
            wiced_rtos_delay_milliseconds(1);
            WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
            loop_count++;
        }
        CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

        /*
         * Enable AIF2 RX pins
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_ENABLES, CS47L24_AIF2RX1_ENA_MASK|CS47L24_AIF2RX2_ENA_MASK, CS47L24_AIF2RX1_ENA|CS47L24_AIF2RX2_ENA), result, _exit );

        if (rtd->need_isrc)
        {
            /* Enable ISRC3 DEC */
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC3DEC1MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_AIF2_RX1 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC3DEC2MIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_AIF2_RX2 << CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_3, CS47L24_ISRC3_DEC1_ENA_MASK|CS47L24_ISRC3_DEC2_ENA_MASK, CS47L24_ISRC3_DEC1_ENA|CS47L24_ISRC3_DEC2_ENA), result, _exit );
        }
        else
        {
            /* Enable ASRC routes */
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC2LMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_AIF2_RX1 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC2RMIX_INPUT_1_SOURCE, CS47L24_INPUT_SOURCE_MASK, CS47L24_DCORE_INPUT_AIF2_RX2 <<  CS47L24_INPUT_SOURCE_SHIFT), result, _exit );
            WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_ENABLE, CS47L24_ASRC2L_ENA_MASK|CS47L24_ASRC2R_ENA_MASK, CS47L24_ASRC2L_ENA|CS47L24_ASRC2R_ENA), result, _exit );
        }
    }
    else
    {
        /* Disable ISRC3 route */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ISRC_3_CTRL_3, CS47L24_ISRC3_DEC1_ENA_MASK|CS47L24_ISRC3_DEC2_ENA_MASK, 0), result, _exit );

        /* Disable ASRC route */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ASRC_ENABLE, CS47L24_ASRC2L_ENA_MASK|CS47L24_ASRC2R_ENA_MASK, 0), result, _exit );

        /*
         * Disable AIF2 RX pins
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF2_RX_ENABLES, CS47L24_AIF2RX1_ENA_MASK|CS47L24_AIF2RX2_ENA_MASK, 0), result, _exit );

        /* Disable HP outputs */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_OUTPUT_ENABLES_1, CS47L24_OUT1L_ENA_MASK|CS47L24_OUT1R_ENA_MASK, 0), result, _exit);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
        loop_count = 0;
        while (((output_disable & (CS47L24_OUT1L_ENA_STS_MASK|CS47L24_OUT1R_ENA_STS_MASK)) != 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
        {
            wiced_rtos_delay_milliseconds(1);
            WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_RAW_OUTPUT_STATUS_1, &output_disable), result, _exit );
            loop_count++;
        }
        CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_hp: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_configure( void *driver_data, wiced_audio_config_t *config, uint32_t* mclk)
{
    cs47l24_device_data_t*         dd      = ( cs47l24_device_data_t* )driver_data;
    cs47l24_device_cmn_data_t*     cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t* rtd     = &device_runtime_table[cs47l24->id];
    wiced_result_t                 result  = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Must be initialized first! */
    if ( ( rtd->init & ( 1 << cs47l24_type( dd ) ) ) == 0 )
    {
        result = WICED_ERROR;
        goto cs47l24_configure_unlock;
    }

    if ( ( rtd->cfg & ( 1 << cs47l24_type( dd ) ) ) != 0 )
    {
        /* Re-configuring is not permitted. */
        result = WICED_ERROR;
        goto cs47l24_configure_unlock;
    }

    /* First time. */
    /* Lock clock configuration. */
    WICED_VERIFY_GOTO( cs47l24_clock( dd, config, *mclk ), result, cs47l24_configure_unlock );

    /* Configure audio route. */
    WICED_VERIFY_GOTO( cs47l24_set_clock_config( dd, config ), result, cs47l24_configure_unlock );

    if ( dd->route->fn != NULL )
    {
        /* Power-up audio route. */
        WICED_VERIFY_GOTO( (*dd->route->fn)( cs47l24, WICED_TRUE ), result, cs47l24_configure_unlock );
    }

    /* The direction is now configured. */
    rtd->cfg |= ( 1 << cs47l24_type( dd ) );

cs47l24_configure_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


wiced_result_t cs47l24_start_play ( void* driver_data )
{
    cs47l24_device_data_t*         dd      = ( cs47l24_device_data_t* )driver_data;
    cs47l24_device_cmn_data_t*     cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t* rtd     = &device_runtime_table[cs47l24->id];
    wiced_result_t                 result  = WICED_SUCCESS;

    LOCK_RTD(rtd);

    if (cs47l24_type(dd) ==  CS47L24_DEVICE_TYPE_PLAYBACK)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT1L_MUTE_MASK, 0             ), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT1R_MUTE_MASK, 0             ), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU), result, _unlock );
    }
    else if (cs47l24_type(dd) ==  CS47L24_DEVICE_TYPE_CAPTURE)
    {
        uint16_t mixer_volume;

        mixer_volume = cs47l24_mixer_volume_index(CS47L24_AIF_MIXER_VOLUME_DEFAULT);

        /*
         * Unmute AIF1 TX mixer
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX1MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, mixer_volume << CS47L24_INPUT_VOLUME_SHIFT), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX2MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, mixer_volume << CS47L24_INPUT_VOLUME_SHIFT), result, _unlock );
    }

 _unlock:
    UNLOCK_RTD(rtd);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_start_play: failed !\n");
    }
    return result;
}


wiced_result_t cs47l24_stop_play ( void* driver_data )
{
    cs47l24_device_data_t*         dd      = ( cs47l24_device_data_t* )driver_data;
    cs47l24_device_cmn_data_t*     cs47l24 = dd->cmn;
    cs47l24_device_runtime_data_t* rtd     = &device_runtime_table[cs47l24->id];
    wiced_result_t                 result  = WICED_SUCCESS;

    LOCK_RTD(rtd);

    if (cs47l24_type(dd) ==  CS47L24_DEVICE_TYPE_PLAYBACK)
    {
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT1L_MUTE_MASK, CS47L24_OUT1L_MUTE), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT1R_MUTE_MASK, CS47L24_OUT1R_MUTE), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU    ), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT_VU_MASK    , CS47L24_OUT_VU    ), result, _unlock );
    }
    else if (cs47l24_type(dd) ==  CS47L24_DEVICE_TYPE_CAPTURE)
    {
        /*
         * Mute AIF1 TX mixer
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX1MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, 0), result, _unlock );
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_AIF1TX2MIX_INPUT_1_VOLUME, CS47L24_INPUT_VOLUME_MASK, 0), result, _unlock );
    }

 _unlock:
    UNLOCK_RTD(rtd);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_stop_play: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_set_playback_volume(void *driver_data, double decibels)
{
    wiced_result_t             result  = WICED_SUCCESS;
    cs47l24_device_data_t*     dd      = (cs47l24_device_data_t *)driver_data;
    cs47l24_device_cmn_data_t* cs47l24 = dd->cmn;
    uint16_t                   volume;

    volume = cs47l24_dac_adc_volume_index(decibels);

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT1L_VOL_MASK, volume << CS47L24_OUT1L_VOL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT1R_VOL_MASK, volume << CS47L24_OUT1R_VOL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1L, CS47L24_OUT_VU_MASK   , CS47L24_OUT_VU                    ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_DAC_DIGITAL_VOLUME_1R, CS47L24_OUT_VU_MASK   , CS47L24_OUT_VU                    ), result, _exit );
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_playback_volume: volume set to %fdB/0x%04hX\n", decibels, volume);

  _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_set_playback_volume: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_set_capture_volume(void *driver_data, double decibels)
{
    wiced_result_t             result  = WICED_SUCCESS;
    /*
     * Do not alter DMIC input volume for now
     * We'll have to use volume control on the AIF1 TX mixer if we have to
     */
#ifdef CS47L24_ENABLE_ADC_VOLUME_CONTROL

    cs47l24_device_data_t*     dd      = (cs47l24_device_data_t *)driver_data;
    cs47l24_device_cmn_data_t* cs47l24 = dd->cmn;
    uint16_t                   volume;

    volume = cs47l24_dac_adc_volume_index(decibels);

    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN1L_DIG_VOL_MASK, volume << CS47L24_IN1L_DIG_VOL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN1R_DIG_VOL_MASK, volume << CS47L24_IN1R_DIG_VOL_SHIFT), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1L, CS47L24_IN_VU_MASK       , CS47L24_IN_VU                       ), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_ADC_DIGITAL_VOLUME_1R, CS47L24_IN_VU_MASK       , CS47L24_IN_VU                       ), result, _exit );
    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_set_capture_volume: volume set to %fdB/0x%04hX\n", decibels, volume);
 _exit:
#endif

    return result;
}


static wiced_result_t cs47l24_get_playback_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    UNUSED_PARAMETER(driver_data);

    *min_volume_in_decibels = CS47L24_VOLUME_DB_MIN;
    *max_volume_in_decibels = CS47L24_VOLUME_DB_MAX;

    return WICED_SUCCESS;
}


static wiced_result_t cs47l24_get_capture_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    UNUSED_PARAMETER(driver_data);

    *min_volume_in_decibels = CS47L24_VOLUME_DB_MIN;
    *max_volume_in_decibels = CS47L24_VOLUME_DB_MAX;

    return WICED_SUCCESS;
}


static wiced_result_t cs47l24_ioctl(void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data)
{
    UNUSED_PARAMETER( driver_data );
    UNUSED_PARAMETER( cmd );
    UNUSED_PARAMETER( cmd_data );

    return WICED_UNSUPPORTED;
}


static wiced_result_t cs47l24_init_device_runtime_data( cs47l24_device_runtime_data_t *rtd )
{
    if ( rtd->rdy == 1 )
    {
        /* Already initialized */
        return WICED_SUCCESS;
    }

    WICED_VERIFY( wiced_rtos_init_mutex( &rtd->lock ) );

    rtd->init      = 0;
    rtd->cfg       = 0;
    rtd->rdy       = 1;
    rtd->dsp_prog  = 0;
    rtd->need_isrc = 0;

    return WICED_SUCCESS;
}


/* This function can only be called from the platform initialization routine */
wiced_result_t cs47l24_device_register( cs47l24_device_data_t *device_data, const platform_audio_device_id_t device_id )
{
    if ( device_data == NULL )
    {
        return WICED_BADARG;
    }

    /* Initialize private portion of device interface. */
    device_data->route->intf->adi.audio_device_driver_specific = device_data;
    device_data->route->intf->adi.device_id = device_id;

    /* Initialize runtime data. */
    WICED_VERIFY( cs47l24_init_device_runtime_data( &device_runtime_table[ device_data->cmn->id ] ) );

    /* Register a device to the audio device list and keep device data internally from this point */
    return wiced_register_audio_device( device_id, &device_data->route->intf->adi );
}


cs47l24_audio_device_interface_t cs47l24_playback =
{
    .type                               = CS47L24_DEVICE_TYPE_PLAYBACK,
    .adi                                = {
        .audio_device_init              = cs47l24_init,
        .audio_device_deinit            = cs47l24_deinit,
        .audio_device_configure         = cs47l24_configure,
        .audio_device_start_streaming   = cs47l24_start_play,
        .audio_device_stop_streaming    = cs47l24_stop_play,
        .audio_device_set_volume        = cs47l24_set_playback_volume,
        .audio_device_get_volume_range  = cs47l24_get_playback_volume_range,
        .audio_device_set_treble        = NULL,
        .audio_device_set_bass          = NULL,
        .audio_device_ioctl             = cs47l24_ioctl,
    },
};


cs47l24_audio_device_interface_t cs47l24_capture =
{
    .type                               = CS47L24_DEVICE_TYPE_CAPTURE,
    .adi                                = {
        .audio_device_init              = cs47l24_init,
        .audio_device_deinit            = cs47l24_deinit,
        .audio_device_configure         = cs47l24_configure,
        .audio_device_start_streaming   = cs47l24_start_play,
        .audio_device_stop_streaming    = cs47l24_stop_play,
        .audio_device_set_volume        = cs47l24_set_capture_volume,
        .audio_device_get_volume_range  = cs47l24_get_capture_volume_range,
        .audio_device_set_treble        = NULL,
        .audio_device_set_bass          = NULL,
        .audio_device_ioctl             = cs47l24_ioctl,
    },
};

const cs47l24_device_route_t cs47l24_route_dac_hp =
{
    .intf   = &cs47l24_playback,
    .fn     = cs47l24_hp,
};

const cs47l24_device_route_t cs47l24_route_adc_dmic =
{
    .intf   = &cs47l24_capture,
    .fn     = cs47l24_dmic,
};
