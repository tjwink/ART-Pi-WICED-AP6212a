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

//#include "wiced.h"
#include "wiced_platform.h"
#include "wiced_audio.h"
#include "ak4954.h"
#include "wwd_constants.h"
#include "wwd_assert.h"

#include "wiced_rtos.h"
#include "platform_i2s.h"
#include "wiced_audio.h"
#include "wwd_assert.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


/******************************************************
 *                      Macros
 ******************************************************/

#define LOCK_RTD(rtd)       wiced_rtos_lock_mutex(&(rtd)->lock)
#define UNLOCK_RTD(rtd)     wiced_rtos_unlock_mutex(&(rtd)->lock)

#if !defined(ARRAYSIZE)
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

/* FIXME */
wiced_result_t wm8533_platform_configure( void* device_data, uint32_t mclk, uint32_t fs, uint8_t width );
#define ak4954_platform_configure wm8533_platform_configure

/******************************************************
 *                    Constants
 ******************************************************/

#define AK4954_REG_PM1                      (0x00)
#define AK4954_REG_PM2                      (0x01)
#define AK4954_REG_SIG_SEL_1                (0x02)
#define AK4954_REG_SIG_SEL_2                (0x03)
#define AK4954_REG_SIG_SEL_3                (0x04)
#define AK4954_REG_MODE_CTRL_1              (0x05)
#define AK4954_REG_MODE_CTRL_2              (0x06)
#define AK4954_REG_MODE_CTRL_3              (0x07)
#define AK4954_REG_DIG_MIC                  (0x08)
#define AK4954_REG_TIMER_SEL                (0x09)
#define AK4954_REG_ALC_TIMER_SEL            (0x0A)
#define AK4954_REG_ALC_MODE_CTRL_1          (0x0B)
#define AK4954_REG_ALC_MODE_CTRL_2          (0x0C)
#define AK4954_REG_IVL_CTRL                 (0x0D)
#define AK4954_REG_IVR_CTRL                 (0x0E)
#define AK4954_REG_RESERVED_1               (0x0F)
#define AK4954_REG_RESERVED_2               (0x10)
#define AK4954_REG_RESERVED_3               (0x11)
#define AK4954_REG_HP_OUT_CTRL              (0x12)
#define AK4954_REG_DVL_CTRL                 (0x13)
#define AK4954_REG_DVR_CTRL                 (0x14)
#define AK4954_REG_BEEP_FREQ                (0x15)
#define AK4954_REG_BEEP_ON_TIME             (0x16)
#define AK4954_REG_BEEP_OFF_TIME            (0x17)
#define AK4954_REG_BEEP_REPEAT_CNT          (0x18)
#define AK4954_REG_BEEP_VOL_CTRL            (0x19)
#define AK4954_REG_RESERVED_4               (0x1A)
#define AK4954_REG_DIG_FILTER_SEL_1         (0x1B)
#define AK4954_REG_DIG_FILTER_SEL_2         (0x1C)
#define AK4954_REG_DIG_FILTER_MODE          (0x1D)
#define AK4954_REG_HPF2_COEF_0              (0x1E)
#define AK4954_REG_HPF2_COEF_1              (0x1F)
#define AK4954_REG_HPF2_COEF_2              (0x20)
#define AK4954_REG_HPF2_COEF_3              (0x21)
#define AK4954_REG_LPF_COEF_0               (0x22)
#define AK4954_REG_LPF_COEF_1               (0x23)
#define AK4954_REG_LPF_COEF_2               (0x24)
#define AK4954_REG_LPF_COEF_3               (0x25)
#define AK4954_REG_FIL3_COEF_0              (0x26)
#define AK4954_REG_FIL3_COEF_1              (0x27)
#define AK4954_REG_FIL3_COEF_2              (0x28)
#define AK4954_REG_FIL3_COEF_3              (0x29)
#define AK4954_REG_EQ_COEF_0                (0x2A)
#define AK4954_REG_EQ_COEF_1                (0x2B)
#define AK4954_REG_EQ_COEF_2                (0x2C)
#define AK4954_REG_EQ_COEF_3                (0x2D)
#define AK4954_REG_EQ_COEF_4                (0x2E)
#define AK4954_REG_EQ_COEF_5                (0x2F)
#define AK4954_REG_DIG_FILTER_SEL_3         (0x30)
#define AK4954_REG_RESERVED_5               (0x31)
#define AK4954_REG_E1_COEF_0                (0x32)
#define AK4954_REG_E1_COEF_1                (0x33)
#define AK4954_REG_E1_COEF_2                (0x34)
#define AK4954_REG_E1_COEF_3                (0x35)
#define AK4954_REG_E1_COEF_4                (0x36)
#define AK4954_REG_E1_COEF_5                (0x37)
#define AK4954_REG_E2_COEF_0                (0x38)
#define AK4954_REG_E2_COEF_1                (0x39)
#define AK4954_REG_E2_COEF_2                (0x3A)
#define AK4954_REG_E2_COEF_3                (0x3B)
#define AK4954_REG_E2_COEF_4                (0x3C)
#define AK4954_REG_E2_COEF_5                (0x3D)
#define AK4954_REG_E3_COEF_0                (0x3E)
#define AK4954_REG_E3_COEF_1                (0x3F)
#define AK4954_REG_E3_COEF_2                (0x40)
#define AK4954_REG_E3_COEF_3                (0x41)
#define AK4954_REG_E3_COEF_4                (0x42)
#define AK4954_REG_E3_COEF_5                (0x43)
#define AK4954_REG_E4_COEF_0                (0x44)
#define AK4954_REG_E4_COEF_1                (0x45)
#define AK4954_REG_E4_COEF_2                (0x46)
#define AK4954_REG_E4_COEF_3                (0x47)
#define AK4954_REG_E4_COEF_4                (0x48)
#define AK4954_REG_E4_COEF_5                (0x49)
#define AK4954_REG_E5_COEF_0                (0x4A)
#define AK4954_REG_E5_COEF_1                (0x4B)
#define AK4954_REG_E5_COEF_2                (0x4C)
#define AK4954_REG_E5_COEF_3                (0x4D)
#define AK4954_REG_E5_COEF_4                (0x4E)
#define AK4954_REG_E5_COEF_5                (0x4F)
#define AK4954_REG_DRC_MODE_CTRL            (0x50)
#define AK4954_REG_NS_CTRL                  (0x51)
#define AK4954_REG_NS_GAIN_ATT_CTRL         (0x52)
#define AK4954_REG_NS_ON_LVL                (0x53)
#define AK4954_REG_NS_OFF_LVL               (0x54)
#define AK4954_REG_NS_REF_SEL               (0x55)
#define AK4954_REG_NS_LPF_COEF_0            (0x56)
#define AK4954_REG_NS_LPF_COEF_1            (0x57)
#define AK4954_REG_NS_LPF_COEF_2            (0x58)
#define AK4954_REG_NS_LPF_COEF_3            (0x59)
#define AK4954_REG_NS_HPF_COEF_0            (0x5A)
#define AK4954_REG_NS_HPF_COEF_1            (0x5B)
#define AK4954_REG_NS_HPF_COEF_2            (0x5C)
#define AK4954_REG_NS_HPF_COEF_3            (0x5D)
#define AK4954_REG_RESERVED_6               (0x5E)
#define AK4954_REG_RESERVED_7               (0x5F)
#define AK4954_REG_DVLC_FILTER_SEL          (0x60)
#define AK4954_REG_DVLC_MODE_CTRL           (0x61)
#define AK4954_REG_DVLCL_CURVE_X1           (0x62)
#define AK4954_REG_DVLCL_CURVE_Y1           (0x63)
#define AK4954_REG_DVLCL_CURVE_X2           (0x64)
#define AK4954_REG_DVLCL_CURVE_Y2           (0x65)
#define AK4954_REG_DVLCL_CURVE_X3           (0x66)
#define AK4954_REG_DVLCL_CURVE_Y3           (0x67)
#define AK4954_REG_DVLCL_SLOPE_1            (0x68)
#define AK4954_REG_DVLCL_SLOPE_2            (0x69)
#define AK4954_REG_DVLCL_SLOPE_3            (0x6A)
#define AK4954_REG_DVLCL_SLOPE_4            (0x6B)
#define AK4954_REG_DVLCM_CURVE_X1           (0x6C)
#define AK4954_REG_DVLCM_CURVE_Y1           (0x6D)
#define AK4954_REG_DVLCM_CURVE_X2           (0x6E)
#define AK4954_REG_DVLCM_CURVE_Y2           (0x6F)
#define AK4954_REG_DVLCM_CURVE_X3           (0x70)
#define AK4954_REG_DVLCM_CURVE_Y3           (0x71)
#define AK4954_REG_DVLCM_SLOPE_1            (0x72)
#define AK4954_REG_DVLCM_SLOPE_2            (0x73)
#define AK4954_REG_DVLCM_SLOPE_3            (0x74)
#define AK4954_REG_DVLCM_SLOPE_4            (0x75)
#define AK4954_REG_DVLCH_CURVE_X1           (0x76)
#define AK4954_REG_DVLCH_CURVE_Y1           (0x77)
#define AK4954_REG_DVLCH_CURVE_X2           (0x78)
#define AK4954_REG_DVLCH_CURVE_Y2           (0x79)
#define AK4954_REG_DVLCH_CURVE_X3           (0x7A)
#define AK4954_REG_DVLCH_CURVE_Y3           (0x7B)
#define AK4954_REG_DVLCH_SLOPE_1            (0x7C)
#define AK4954_REG_DVLCH_SLOPE_2            (0x7D)
#define AK4954_REG_DVLCH_SLOPE_3            (0x7E)
#define AK4954_REG_DVLCH_SLOPE_4            (0x7F)
#define AK4954_REG_DVLCL_LPF_COEF_0         (0x80)
#define AK4954_REG_DVLCL_LPF_COEF_1         (0x81)
#define AK4954_REG_DVLCL_LPF_COEF_2         (0x82)
#define AK4954_REG_DVLCL_LPF_COEF_3         (0x83)
#define AK4954_REG_DVLCM_HPF_COEF_0         (0x84)
#define AK4954_REG_DVLCM_HPF_COEF_1         (0x85)
#define AK4954_REG_DVLCM_HPF_COEF_2         (0x86)
#define AK4954_REG_DVLCM_HPF_COEF_3         (0x87)
#define AK4954_REG_DVLCM_LPF_COEF_0         (0x88)
#define AK4954_REG_DVLCM_LPF_COEF_1         (0x89)
#define AK4954_REG_DVLCM_LPF_COEF_2         (0x8A)
#define AK4954_REG_DVLCM_LPF_COEF_3         (0x8B)
#define AK4954_REG_DVLCH_HPF_COEF_0         (0x8C)
#define AK4954_REG_DVLCH_HPF_COEF_1         (0x8D)
#define AK4954_REG_DVLCH_HPF_COEF_2         (0x8E)
#define AK4954_REG_DVLCH_HPF_COEF_3         (0x8F)

/* 00H Power Management 1 */
#define AK4954_PMADL                        (0x01)
#define AK4954_PMADL_MASK                   (0x01)
#define AK4954_PMADL_SHIFT                  (0)
#define AK4954_PMADL_BITS                   (1)
#define AK4954_PMADR                        (0x02)
#define AK4954_PMADR_MASK                   (0x02)
#define AK4954_PMADR_SHIFT                  (1)
#define AK4954_PMADR_BITS                   (1)
#define AK4954_PMDAC                        (0x04)
#define AK4954_PMDAC_MASK                   (0x04)
#define AK4954_PMDAC_SHIFT                  (2)
#define AK4954_PMDAC_BITS                   (1)
#define AK4954_PMVCM                        (0x40)
#define AK4954_PMVCM_MASK                   (0x40)
#define AK4954_PMVCM_SHIFT                  (6)
#define AK4954_PMVCM_BITS                   (1)
#define AK4954_PMPFIL                       (0x80)
#define AK4954_PMPFIL_MASK                  (0x80)
#define AK4954_PMPFIL_SHIFT                 (7)
#define AK4954_PMPFIL_BITS                  (1)

/* 01H Power Management 2 */
#define AK4954_PMSL                         (0x2)
#define AK4954_PMSL_MASK                    (0x2)
#define AK4954_PMSL_SHIFT                   (1)
#define AK4954_PMPLL                        (0x04)
#define AK4954_PMPLL_MASK                   (0x04)
#define AK4954_PMPLL_SHIFT                  (2)
#define AK4954_PMPLL_BITS                   (1)
#define AK4954_MS                           (0x08)
#define AK4954_MS_MASK                      (0x08)
#define AK4954_MS_SHIFT                     (3)
#define AK4954_MS_BITS                      (1)
#define AK4954_PMHPL                        (0x10)
#define AK4954_PMHPL_MASK                   (0x10)
#define AK4954_PMHPL_SHIFT                  (4)
#define AK4954_PMHPR                        (0x20)
#define AK4954_PMHPR_MASK                   (0x20)
#define AK4954_PMHPR_SHIFT                  (5)

/* 02H Signal Select 1 */
#define AK4954_MGAIN_0DB                    (0x04)
#define AK4954_MGAIN_6DB                    (0x00)
#define AK4954_MGAIN_13DB                   (0x01)
#define AK4954_MGAIN_20DB                   (0x02)
#define AK4954_MGAIN_26DB                   (0x03)
#define AK4954_MGAIN_MASK                   (0x07)
#define AK4954_MGAIN_SHIFT                  (0)
#define AK4954_MGAIN_BITS                   (3)
#define AK4954_PMMP                         (0x08)
#define AK4954_PMMP_MASK                    (0x08)
#define AK4954_PMMP_SHIFT                   (3)
#define AK4954_PMMP_BITS                    (1)
#define AK4954_DACSL                        (0x20)
#define AK4954_DACSL_MASK                   (0x20)
#define AK4954_DACSL_SHIFT                  (5)
#define AK4954_DACSL_BITS                   (1)
#define AK4954_SLPSN                        (0x80)
#define AK4954_SLPSN_MASK                   (0x80)
#define AK4954_SLPSN_SHIFT                  (7)
#define AK4954_SLPSN_BITS                   (1)

/* 05H Mode Control 1 */
#define AK4954_DIF_MODE_0                   (0x00)
#define AK4954_DIF_MODE_1                   (0x01)
#define AK4954_DIF_MODE_2                   (0x02)
#define AK4954_DIF_MODE_3                   (0x03)
#define AK4954_DIF_MODE_4                   (0x06)
#define AK4954_DIF_MODE_5                   (0x07)
#define AK4954_DIF_MASK                     (0x07)      /* [2:0] */
#define AK4954_DIF_SHIFT                    (0)
#define AK4954_DIF_BITS                     (3)
#define AK4954_BCKO_32FS                    (0x00)
#define AK4954_BCKO_64FS                    (0x08)
#define AK4954_BCKO_MASK                    (0x08)
#define AK4954_BCKO_SHIFT                   (3)
#define AK4954_BCKO_BITS                    (1)
#define AK4954_PLL_32FS                     (0x00)
#define AK4954_PLL_64FS                     (0x10)
#define AK4954_PLL_11P2896MHZ               (0x20)
#define AK4954_PLL_12P288MHZ                (0x30)
#define AK4954_PLL_12MHZ                    (0x40)
#define AK4954_PLL_24MHZ                    (0x50)
#define AK4954_PLL_13P5MHZ                  (0x60)
#define AK4954_PLL_27MHZ                    (0x70)
#define AK4954_PLL_MASK                     (0x70)      /* [6:4] */
#define AK4954_PLL_SHIFT                    (4)
#define AK4954_PLL_BITS                     (3)

/* 06H Mode Control 2 */
#define AK4954_CM_256FS                     (0x00)
#define AK4954_CM_384FS                     (0x40)
#define AK4954_CM_512FS                     (0x80)
#define AK4954_CM_1024FS                    (0xC0)
#define AK4954_CM_MASK                      (0xC0)
#define AK4954_CM_SHIFT                     (6)
#define AK4954_CM_BITS                      (2)
#define AK4954_FS_8KHZ                      (0x00)
#define AK4954_FS_11P025KHZ                 (0x01)
#define AK4954_FS_12KHZ                     (0x02)
#define AK4954_FS_16KHZ                     (0x04)
#define AK4954_FS_22P05KHZ                  (0x05)
#define AK4954_FS_24KHZ                     (0x06)
#define AK4954_FS_32KHZ                     (0x08)
#define AK4954_FS_44P1KHZ                   (0x09)
#define AK4954_FS_48KHZ                     (0x0A)
#define AK4954_FS_64KHZ                     (0x0C)
#define AK4954_FS_88P2KHZ                   (0x0D)
#define AK4954_FS_96KHZ                     (0x0E)
#define AK4954_FS_MASK                      (0x0F)      /* [3:0] */
#define AK4954_FS_SHIFT                     (0)
#define AK4954_FS_BITS                      (4)

/* 0BH ALC Mode Control 1 */
#define AK4954_ALC                          (0x20)
#define AK4954_ALC_MASK                     (0x20)
#define AK4954_ALC_SHIFT                    (5)
#define AK4954_ALC_BITS                     (1)

/* 0DH Lch Input Volume Control */
#define AK4954_IVL_MASK                     (0xFF)
#define AK4954_IVL_SHIFT                    (0)

/* 0EH Rch Input Volume Control */
#define AK4954_IVR_MASK                     (0xFF)
#define AK4954_IVR_SHIFT                    (0)

/* 13H Lch Digital Volume Control */
#define AK4954_DVL_MASK                     (0xFF)
#define AK4954_DVL_SHIFT                    (0)
#define AK4954_DVL_BITS                     (8)

/* 14H Rch Digital Volume Control */
#define AK4954_DVR_MASK                     (0xFF)
#define AK4954_DVR_SHIFT                    (0)
#define AK4954_DVR_BITS                     (8)

/* 1DH Digital Filter Mode */
#define AK4954_PFSDO                        (0x01)
#define AK4954_PFSDO_MASK                   (0x01)
#define AK4954_PFSDO_SHIFT                  (0)
#define AK4954_PFSDO_BITS                   (1)
#define AK4954_ADCPF                        (0x02)
#define AK4954_ADCPF_MASK                   (0x02)
#define AK4954_ADCPF_SHIFT                  (1)
#define AK4954_ADCPF_BITS                   (1)
#define AK4954_PFDAC_SDTI                   (0)
#define AK4954_PFDAC                        (0x04)
#define AK4954_PFDAC_MASK                   (0x04)
#define AK4954_PFDAC_SHIFT                  (2)
#define AK4954_PFDAC_BITS                   (1)
#define AK4954_PMDRC                        (0x80)
#define AK4954_PMDRC_MASK                   (0x80)
#define AK4954_PMDRC_SHIFT                  (7)
#define AK4954_PMDRC_BITS                   (1)

/* 0x0C is 0dB. */
#define AK4954_DVL_DEFAULT                  (0x0C)
//#define AK4954_DVL_DEFAULT                  (0x0C + 30)
#define AK4954_DVL_MUTE                     (0xFF)

/* From AK4954_Configuration_4_21_14.doc, AK4954 Full-duplex Audio Configuration. */
#define AK4954_MGAIN_DEFAULT                (AK4954_MGAIN_20DB)

/* 0x91 is 0dB. */
#define AK4954_IVL_DEFAULT                  (0x91)

/* Input volume gain settings (Table 40). */
#define AK4954_IVL_GAIN_IN_DB_MIN           (-52.5 + (5 * -0.375))
#define AK4954_IVL_GAIN_IN_DB_MAX           (+36.0)
#define AK4954_IVL_GAIN_IN_DB_STEP          (+0.375)

/* Output digital volume gain settings (Table 69). */
#define AK4954_DVL_GAIN_IN_DB_MIN           (-65.5)
#define AK4954_DVL_GAIN_IN_DB_MAX           (+6.0)
#define AK4954_DVL_GAIN_IN_DB_STEP          (+0.5)

/* Rise-up time of the VCOM pin is 1.5ms (max)
 * See Control Sequence, Clock Set up.
 */
#define AK4954_PMVCM_RISE_UP_TIME_IN_MILLIS (2)

/* Pulse width for PDN reset. */
#define AK4954_RESET_HOLD_TIME_IN_MILLIS    (1)

#define I2C_XFER_RETRY_COUNT                (3)
#define I2C_DISABLE_DMA                     WICED_TRUE

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct ak4954_device_runtime_data   ak4954_device_runtime_data_t;
typedef struct ak4954_mcki_fs_map           ak4954_mcki_fs_map_t;
typedef struct ak4954_fs_map                ak4954_fs_map_t;
typedef struct ak4954_mcki_freq_map         ak4954_mcki_freq_map_t;
typedef struct ak4954_dbl_range             ak4954_dbl_range_t;
typedef struct i2c_ak4954_payload           i2c_ak4954_payload_t;

typedef enum ak4954_device_type             ak4954_device_type_t;

typedef wiced_result_t (ak4954_rfn_t)(ak4954_device_cmn_data_t *, wiced_bool_t);

/******************************************************
 *                   Enumerations
 ******************************************************/

enum ak4954_device_type
{
    AK4954_DEVICE_TYPE_PLAYBACK     = 0,
    AK4954_DEVICE_TYPE_CAPTURE,

    /* Not a device type! */
    AK4954_DEVICE_TYPE_MAX,
};

/******************************************************
 *                    Structures
 ******************************************************/

struct ak4954_mcki_fs_map
{
    uint32_t        fs1, fs2;
    uint16_t        mult;
    uint8_t         cm;
};

struct ak4954_fs_map
{
    uint32_t        fs;
    uint8_t         mc2fs;
};

struct ak4954_mcki_freq_map
{
    uint32_t        mcki;
    uint8_t         pll;
    size_t          sz;
    const uint32_t  *fs;
};

/* Codec device runtime information. */
struct ak4954_device_runtime_data
{
    wiced_mutex_t   lock;

    /* This structure is initialized and ready for use. */
    uint8_t         rdy   : 1;

    /* Playback/capture in-use flags. */
    uint8_t         init  : AK4954_DEVICE_TYPE_MAX;
    uint8_t         cfg   : AK4954_DEVICE_TYPE_MAX;

    /* Shared configuration (duplex). */
    uint8_t         bits_per_sample;
    uint8_t         channels;
    uint32_t        sample_rate;
};

struct ak4954_audio_device_interface
{
    ak4954_device_type_t            type;
    wiced_audio_device_interface_t  adi;
};

struct ak4954_device_route
{
    ak4954_audio_device_interface_t *intf;
    ak4954_rfn_t                    *fn;
};

struct ak4954_dbl_range
{
    double min;
    double max;
    double step;

    /* The range is inverted: descending from position 0. */
    wiced_bool_t inv;
};

struct i2c_ak4954_payload
{
    uint8_t addr;
    uint8_t data;
};

/******************************************************
 *               Variables Definitions
 ******************************************************/

static ak4954_device_runtime_data_t device_runtime_table[AK4954_MAX_DEVICES];

static const ak4954_mcki_fs_map_t mcki_fs_map[] =
{
    {  8000, 96000,  256, AK4954_CM_256FS  },
    { 11025, 48000,  384, AK4954_CM_384FS  },
    { 11025, 48000,  512, AK4954_CM_512FS  },
    {  8000, 24000, 1024, AK4954_CM_1024FS },
};

static const ak4954_fs_map_t fs_map[] =
{
    {  8000, AK4954_FS_8KHZ      },
    { 11025, AK4954_FS_11P025KHZ },
    { 12000, AK4954_FS_12KHZ     },
    { 16000, AK4954_FS_16KHZ     },
    { 22050, AK4954_FS_22P05KHZ  },
    { 24000, AK4954_FS_24KHZ     },
    { 32000, AK4954_FS_32KHZ     },
    { 44100, AK4954_FS_44P1KHZ   },
    { 48000, AK4954_FS_48KHZ     },
    { 64000, AK4954_FS_64KHZ     },
    { 88200, AK4954_FS_88P2KHZ   },
    { 96000, AK4954_FS_96KHZ     },
};

static const uint32_t mcki_8[8] =
{
    8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000
};
static const uint32_t mcki_12[12] =
{
    8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000, 11025, 22050, 44100, 88200
};

/* Removed values that don't round. */
static const ak4954_mcki_freq_map_t mcki_freq_map[] =
{
    { 11289600, AK4954_PLL_11P2896MHZ, 12, mcki_12 },
    { 12288000, AK4954_PLL_12P288MHZ,  12, mcki_12 },
    { 12000000, AK4954_PLL_12MHZ,       8, mcki_8  },
    { 24000000, AK4954_PLL_24MHZ,       8, mcki_8  },
    { 13500000, AK4954_PLL_13P5MHZ,     0, 0       },
    { 27000000, AK4954_PLL_27MHZ,       0, 0       },
};

const static ak4954_dbl_range_t ak4954_dbl_range_ivl =
{
    .min    = AK4954_IVL_GAIN_IN_DB_MIN,
    .max    = AK4954_IVL_GAIN_IN_DB_MAX,
    .step   = AK4954_IVL_GAIN_IN_DB_STEP,
    .inv    = WICED_FALSE,
};

const static ak4954_dbl_range_t ak4954_dbl_range_dvl =
{
    .min    = AK4954_DVL_GAIN_IN_DB_MIN,
    .max    = AK4954_DVL_GAIN_IN_DB_MAX,
    .step   = AK4954_DVL_GAIN_IN_DB_STEP,
    .inv    = WICED_TRUE,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/* XXX The AK4954 can perform more than one byte write operation per sequence, but
 * for now just implement one byte per register write.
 */
static wiced_result_t ak4954_i2c_reg_write(wiced_i2c_device_t *device, uint8_t reg, uint8_t value)
{
    static wiced_i2c_message_t msg[1];
    i2c_ak4954_payload_t payload;

    payload.addr = reg;
    payload.data = value;

    WICED_VERIFY( wiced_i2c_init_tx_message( msg, &payload, 2, I2C_XFER_RETRY_COUNT, I2C_DISABLE_DMA ) );

    return wiced_i2c_transfer( device, msg, 1 );
}


/* AK4954 Random Address Read. */
static wiced_result_t ak4954_i2c_reg_read( wiced_i2c_device_t *device, uint8_t reg, uint8_t *value )
{
    static wiced_i2c_message_t msg[2];

    /* Some I2C masters might not support combined messages. */

    /* Reset device's register counter by issuing a TX. */
    WICED_VERIFY( wiced_i2c_init_tx_message( &msg[0], &reg, sizeof(reg), I2C_XFER_RETRY_COUNT, I2C_DISABLE_DMA ) );

    /* Initialize RX message. */
    WICED_VERIFY( wiced_i2c_init_rx_message( &msg[1], value, sizeof(*value), I2C_XFER_RETRY_COUNT, I2C_DISABLE_DMA ) );

    /* Transfer. */
    return wiced_i2c_transfer( device, msg, ARRAYSIZE( msg ) );
}


static wiced_result_t ak4954_reg_write( ak4954_device_cmn_data_t* ak4954, uint8_t address, uint8_t reg_data )
{
    return ak4954_i2c_reg_write(ak4954->i2c_data, address, reg_data);
}


static wiced_result_t ak4954_reg_read(ak4954_device_cmn_data_t* ak4954, uint8_t address, uint8_t* reg_data)
{
    return ak4954_i2c_reg_read(ak4954->i2c_data, address, reg_data);
}


static wiced_result_t ak4954_upd_bits(ak4954_device_cmn_data_t *ak4954, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t old, new;

    WICED_VERIFY( ak4954_reg_read( ak4954, reg, &old ) );

    new = ( old & ~mask ) | val;
    if ( new != old )
    {
        return ak4954_reg_write( ak4954, reg, new );
    }

    return WICED_SUCCESS;
}


static inline ak4954_device_type_t ak4954_type(ak4954_device_data_t *dd)
{
    return dd->route->intf->type;
}


/* Convert an ascending or descending range to a 0-indexed integer range. */
static uint8_t ak4954_double_range_to_index( ak4954_device_cmn_data_t *ak4954, double val, const ak4954_dbl_range_t *r )
{
    uint8_t idx;

    if ( val > r->max )
    {
        val = r->max;
    }
    else if ( val < r->min )
    {
        val = r->min;
    }

    if ( r->inv == WICED_TRUE )
    {
        idx = ( r->max - val ) / r->step;
    }
    else
    {
        idx = ( val - r->min ) / r->step;
    }

    return idx;
}


/* System reset from datasheet. */
static wiced_result_t ak4954_reset(ak4954_device_cmn_data_t *ak4954)
{
    if (ak4954->pdn != WICED_GPIO_NONE)
    {
        /* Pulse PDN line. */
        WICED_VERIFY( wiced_gpio_output_low( ak4954->pdn ) );
        WICED_VERIFY( wiced_rtos_delay_milliseconds( AK4954_RESET_HOLD_TIME_IN_MILLIS ) );
        WICED_VERIFY( wiced_gpio_output_high( ak4954->pdn ) );
    }

    /* Issue dummy command. */
    return ak4954_reg_write( ak4954, AK4954_REG_PM1, 0 );
}


static wiced_result_t ak4954_init(void *driver_data, wiced_audio_data_port_t* data_port)
{
    ak4954_device_data_t *dd = ( ak4954_device_data_t* )driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    ak4954_device_runtime_data_t *rtd = &device_runtime_table[ak4954->id];
    uint8_t pm2 = 0;
    wiced_result_t result;

    result = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Initialization is only required once. */
    if ( ( rtd->init & ( 1 << ak4954_type( dd ) ) ) != 0 )
    {
        /* This device type is already initialized! */
        wiced_assert("already initialized", !(rtd->init & 1<<ak4954_type(dd)));
        result = WICED_ERROR;
        goto ak4954_init_unlock;
    }
    else if ( rtd->init != 0 )
    {
        WPRINT_LIB_INFO(("ak4954 reusing first initialization"));
        goto already_initialized;
    }

    /* Initialize GPIOs. */
    if (ak4954->pdn != WICED_GPIO_NONE)
    {
        WICED_VERIFY_GOTO( wiced_gpio_init( ak4954->pdn, OUTPUT_PUSH_PULL ), result, ak4954_init_unlock );
    }

    /* Enable I2C clocks, init I2C peripheral. */
    WICED_VERIFY_GOTO( wiced_i2c_init( ak4954->i2c_data ), result, ak4954_init_unlock );

    /* Reset to defaults. */
    WICED_VERIFY_GOTO( ak4954_reset(ak4954), result, ak4954_init_unlock );

    /* Power-up. */
    WICED_VERIFY_GOTO( ak4954_reg_write( ak4954, AK4954_REG_PM1, AK4954_PMVCM ), result, ak4954_init_unlock );

    wiced_rtos_delay_milliseconds( AK4954_PMVCM_RISE_UP_TIME_IN_MILLIS );

    if ( ak4954->ck.pll_enab )
    {
        pm2 |= AK4954_PMPLL;
    }
    if ( ak4954->ck.is_frame_master )
    {
        pm2 |= AK4954_MS;
    }

    /* Enable PLL and M/S mode. */
    WICED_VERIFY_GOTO( ak4954_reg_write( ak4954, AK4954_REG_PM2, pm2 ), result, ak4954_init_unlock );

already_initialized:
    /* Mark device as initialized. */
    rtd->init |= 1<<ak4954_type(dd);

    data_port->type = PLATFORM_AUDIO_LINE;
    data_port->port = dd->data_port;
    switch(dd->route->intf->type)
    {
        case AK4954_DEVICE_TYPE_PLAYBACK:
            data_port->channel = WICED_PLAY_CHANNEL;
            break;

        case AK4954_DEVICE_TYPE_CAPTURE:
            data_port->channel = WICED_RECORD_CHANNEL;
            break;

        default:
            result = WICED_ERROR;
            break;
    }

ak4954_init_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


static wiced_result_t ak4954_deinit(void *driver_data)
{
    ak4954_device_data_t *dd = ( ak4954_device_data_t* )driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    ak4954_device_runtime_data_t *rtd = &device_runtime_table[ak4954->id];
    wiced_result_t result = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Power-down audio route. */
    if ( dd->route->fn != NULL )
    {
        WICED_VERIFY_GOTO( (*dd->route->fn)( ak4954, WICED_FALSE ), result, ak4954_deinit_unlock );
    }

    /* This audio route is no longer configured. */
    /* Remove in-use flags. */
    rtd->cfg  &= ~( 1 << ak4954_type( dd ) );
    rtd->init &= ~( 1 << ak4954_type( dd ) );

    /* No other initializations; clean-up. */
    if ( ( rtd->init & ~( 1 << ak4954_type( dd ) ) ) == 0 )
    {
        /* Disable PLL. */
        if (ak4954->ck.pll_enab)
        {
            WICED_VERIFY_GOTO( ak4954_upd_bits( ak4954, AK4954_REG_PM2, AK4954_PMPLL_MASK, 0 ), result, ak4954_deinit_unlock );
        }

        /* Disable VCM. */
        WICED_VERIFY_GOTO( ak4954_upd_bits( ak4954, AK4954_REG_PM1, AK4954_PMVCM_MASK, 0 ), result, ak4954_deinit_unlock );

        /* Power-down chip. */
        if (ak4954->pdn != WICED_GPIO_NONE)
        {
            WICED_VERIFY_GOTO( wiced_gpio_output_low( ak4954->pdn ), result, ak4954_deinit_unlock );
        }
    }

    /* Don't deinitialize I2C since it might be used by other modules.
     * Diddo for IOE.
     */
ak4954_deinit_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


/* PLL Slave Mode. */
wiced_result_t ak4954_spll(ak4954_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;

    UNUSED_PARAMETER(config);
    UNUSED_PARAMETER(mclk);

    /* FIXME Hard-coded to Broadcom I2S.ClockDivider BCLK. */
    return ak4954_upd_bits( ak4954, AK4954_REG_MODE_CTRL_1, AK4954_PLL_MASK, AK4954_PLL_64FS );
}


/* PLL Master Mode. */
wiced_result_t ak4954_mpll(ak4954_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t result;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    size_t i, j;

    result = WICED_SUCCESS;

    /* Traverse the supported PLL frequencies. */
    for (i=0; i<ARRAYSIZE(mcki_freq_map); i++)
    {
        /* ... for the selected sample rate. */
        for (j=0; j<mcki_freq_map[i].sz; j++)
        {
            if (mcki_freq_map[i].fs[j] == config->sample_rate)
            {
                if (mclk == 0)
                {
                    /* Configure clock. */
                    result = ak4954_platform_configure(dd, mcki_freq_map[i].mcki, config->sample_rate, config->bits_per_sample);
                }
                else if (mclk != mcki_freq_map[i].mcki)
                {
                    /* Supplied mclk doesn't satisfy this frequency. */
                    continue;
                }
                if (result == WICED_SUCCESS)
                {
                    result = ak4954_upd_bits(ak4954, AK4954_REG_MODE_CTRL_1, AK4954_CM_MASK, mcki_freq_map[i].pll);
                    goto done;
                }
            }
        }
    }
    if (i == ARRAYSIZE(mcki_freq_map) || result != WICED_SUCCESS)
        result = WICED_UNSUPPORTED;

done:
    return result;
}


wiced_result_t ak4954_ext(ak4954_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    wiced_result_t result = WICED_SUCCESS;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    size_t i;

    /* Traverse supported l-r ratios. */
    for (i=0; i<ARRAYSIZE(mcki_fs_map); i++)
    {
        /* Find a sample rate range that is supported. */
        if (config->sample_rate >= mcki_fs_map[i].fs1 &&
            config->sample_rate <= mcki_fs_map[i].fs2)
        {
            if (mclk == 0)
            {
                /* Find an mclk satisfying the Codec l-r ratio and the platform's supported clock frequencies. */
                result = ak4954_platform_configure(dd, mcki_fs_map[i].mult*config->sample_rate, config->sample_rate, config->bits_per_sample);
            }
            else if (mclk != mcki_fs_map[i].mult*config->sample_rate)
            {
                /* Supplied mclk doesn't satisfy this l-r ratios. */
                continue;
            }
            if (result == WICED_SUCCESS)
            {
                result = ak4954_upd_bits(ak4954, AK4954_REG_MODE_CTRL_2, AK4954_CM_MASK, mcki_fs_map[i].cm);
                break;
            }
        }
    }
    if (i == ARRAYSIZE(mcki_fs_map) || result != WICED_SUCCESS)
        result = WICED_UNSUPPORTED;

    return result;
}


static wiced_result_t ak4954_clock(ak4954_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    size_t i;
    uint8_t mc1 = 0;

    /* Sample frequency. */
    for ( i = 0; i < ARRAYSIZE( fs_map ); i++ )
    {
        if ( config->sample_rate == fs_map[i].fs )
        {
            WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_MODE_CTRL_2, AK4954_FS_MASK, fs_map[i].mc2fs) );
            break;
        }
    }

    if ( i == ARRAYSIZE( fs_map ) )
    {
        /* Requested sample rate cannot be supported */
        return WICED_UNSUPPORTED;
    }

    /* Clock configuration. */
    if ( ak4954->ck.fn != NULL)
    {
        WICED_VERIFY( (*ak4954->ck.fn)(dd, config, mclk) );
    }

    /* Bit clock, xfer format. */

    switch ( config->bits_per_sample )
    {
        case 16:
            mc1 |= AK4954_BCKO_32FS | AK4954_DIF_MODE_3;
            break;
        case 24:
            mc1 |= AK4954_BCKO_64FS | AK4954_DIF_MODE_3;
            break;
        case 32:
            mc1 |= AK4954_BCKO_64FS | AK4954_DIF_MODE_5;
            break;
        default:
            /* Invalid number of bits-per-sample */
            return WICED_UNSUPPORTED;
    }

    return ak4954_upd_bits(ak4954, AK4954_REG_MODE_CTRL_1, AK4954_BCKO_MASK | AK4954_DIF_MASK, mc1);
}


static inline wiced_bool_t ak4954_clock_config_eq(ak4954_device_runtime_data_t *rtd, const wiced_audio_config_t *config)
{
    if ( config->bits_per_sample == rtd->bits_per_sample &&
         config->channels        == rtd->channels        &&
         config->sample_rate     == rtd->sample_rate )
    {
        return WICED_TRUE;
    }
    return WICED_FALSE;
}


static inline wiced_result_t ak4954_set_clock_config(ak4954_device_runtime_data_t *rtd, const wiced_audio_config_t *config)
{
    rtd->bits_per_sample = config->bits_per_sample;
    rtd->channels = config->channels;
    rtd->sample_rate = config->sample_rate;

    return WICED_SUCCESS;
}


wiced_result_t ak4954_mic(ak4954_device_cmn_data_t *ak4954, wiced_bool_t is_pwr_up)
{
    if (is_pwr_up == WICED_TRUE)
    {
        /* Set-up mic gain and power-up mic pwr supply.
        */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_SIG_SEL_1, AK4954_PMMP_MASK | AK4954_MGAIN_MASK, AK4954_PMMP | AK4954_MGAIN_DEFAULT) );

        /* Set-up input signal. */
        /* Set-up timer. */
//

        /* Set-up ALC. */
//        if (result == WICED_SUCCESS)
//            ; result = ak4954_upd_bits(ak4954, AK4954_REG_ALC_MODE_CTRL_1, AK4954_ALC_MASK, AK4954_ALC);

        /* Set-up REF value of ALC. */
        /* Set-up IVOL value of ALC. */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_IVL_CTRL, AK4954_IVL_MASK, AK4954_IVL_DEFAULT) );

        /* Set-up programmable filter on/off. */
        /* Set-up programmable filter path. */
        /* Set-up coefficient of the programmable filter. */
//

        /* Power-up mic. */
        return ak4954_upd_bits(ak4954,
                                      AK4954_REG_PM1,
                                      AK4954_PMADL_MASK | AK4954_PMADR_MASK | AK4954_PMPFIL_MASK,
                                      AK4954_PMADL | AK4954_PMADR | AK4954_PMPFIL);
    }
    else
    {
        /* Power-down mic, ADC, programmable filter. */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_PM1, AK4954_PMADL_MASK | AK4954_PMADR_MASK | AK4954_PMPFIL_MASK, 0) );
        /* Disable ALC. */
        return ak4954_upd_bits(ak4954, AK4954_REG_ALC_MODE_CTRL_1, AK4954_ALC_MASK, 0);
    }
}


wiced_result_t ak4954_hp(ak4954_device_cmn_data_t *ak4954, wiced_bool_t is_pwr_up)
{
    if (is_pwr_up == WICED_TRUE)
    {
        /* Reset l/r volume to default. */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_DVL_CTRL, AK4954_DVL_MASK, AK4954_DVL_DEFAULT) );

        /* Power-up DAC. */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_PM1, AK4954_PMDAC_MASK, AK4954_PMDAC) );

        /* Power-up hp amp. */
        return ak4954_upd_bits(ak4954, AK4954_REG_PM2, AK4954_PMHPR_MASK | AK4954_PMHPL_MASK, AK4954_PMHPR | AK4954_PMHPL);
    }
    else
    {
        /* Power-down HP amp. */
        WICED_VERIFY( ak4954_upd_bits(ak4954, AK4954_REG_PM2, AK4954_PMHPR_MASK | AK4954_PMHPL_MASK, 0) );
        /* Power-down DAC. */
        return ak4954_upd_bits(ak4954, AK4954_REG_PM1, AK4954_PMDAC_MASK, 0);
    }
}


wiced_result_t ak4954_spkr(ak4954_device_cmn_data_t *ak4954, wiced_bool_t is_pwr_up)
{
    if (is_pwr_up == WICED_TRUE)
    {
        /* Set-up path: DAC->SPK amp. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_SIG_SEL_1, AK4954_DACSL_MASK, AK4954_DACSL ) );

        /* Set-up SPK amp gain. */
        /* Set-up digital output volume control. */
        /* Set-up DRC control. */
//

        /* Power-up DRC. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_DIG_FILTER_MODE, AK4954_PMDRC_MASK, AK4954_PMDRC ) );

        /* Power-up DAC and SPK amp. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_PM1, AK4954_PMDAC_MASK, AK4954_PMDAC ) );
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_PM2, AK4954_PMSL_MASK,  AK4954_PMSL  ) );

        /* Exit SPK amp power save mode. */
        return ak4954_upd_bits( ak4954, AK4954_REG_SIG_SEL_1, AK4954_SLPSN_MASK, AK4954_SLPSN );
    }
    else
    {
        /* Enter SPKR amp power save mode. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_SIG_SEL_1, AK4954_SLPSN_MASK, 0 ) );

        /* Disable DAC to output amp. */
        /* Set separately from SLPSN because datasheet shows it that way. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_SIG_SEL_1, AK4954_DACSL_MASK, 0 ) );

        /* Power-down DAC and SPK amp. */
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_PM2, AK4954_PMSL_MASK,  0 ) );
        WICED_VERIFY( ak4954_upd_bits( ak4954, AK4954_REG_PM1, AK4954_PMDAC_MASK, 0 ) );

        /* Power-down DRC. */
        return ak4954_upd_bits( ak4954, AK4954_REG_DIG_FILTER_MODE, AK4954_PMDRC_MASK, 0 );
    }
}


static wiced_result_t ak4954_configure( void *driver_data, wiced_audio_config_t *config, uint32_t* mclk)
{
    ak4954_device_data_t *dd = ( ak4954_device_data_t* )driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    ak4954_device_runtime_data_t *rtd = &device_runtime_table[ak4954->id];
    wiced_result_t result;

    LOCK_RTD(rtd);

    /* Must be initialized first! */
    if ( ( rtd->init & ( 1 << ak4954_type( dd ) ) ) == 0 )
    {
        result = WICED_ERROR;
        goto ak4954_configure_unlock;
    }

    /*
     * Configure clocking.
     * Shared between playback/capture, so configs need to match.
     */
    if ( ( rtd->cfg & ~( 1 << ak4954_type(dd) ) ) != 0 )
    {
        /* Device configured in opposite direction.
         * Clocking arguments need to match to satisfy duplex requirements.
         */
        if ( ak4954_clock_config_eq(rtd, config) == WICED_FALSE )
        {
            result = WICED_UNSUPPORTED;
            goto ak4954_configure_unlock;
        }
    }
    else if ( ( rtd->cfg & ( 1 << ak4954_type( dd ) ) ) != 0 )
    {
        /* Re-configuring is not permitted. */
        result = WICED_ERROR;
        goto ak4954_configure_unlock;
    }

    /* First time. */
    /* Lock clock configuration. */
    WICED_VERIFY_GOTO( ak4954_clock( dd, config, *mclk ), result, ak4954_configure_unlock );

    /* Configure audio route. */
    WICED_VERIFY_GOTO( ak4954_set_clock_config( rtd, config ), result, ak4954_configure_unlock );

    if ( dd->route->fn != NULL )
    {
        /* Power-up audio route. */
        WICED_VERIFY_GOTO( (*dd->route->fn)( ak4954, WICED_TRUE ), result, ak4954_configure_unlock );
    }

    /* The direction is now configured. */
    rtd->cfg |= ( 1 << ak4954_type( dd ) );

ak4954_configure_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


wiced_result_t ak4954_start_play ( void* driver_data )
{
    ak4954_device_data_t *dd = ( ak4954_device_data_t* )driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    ak4954_device_runtime_data_t *rtd = &device_runtime_table[ak4954->id];
    wiced_result_t result = WICED_SUCCESS;

    LOCK_RTD(rtd);

    if (dd->route->intf->type ==  AK4954_DEVICE_TYPE_PLAYBACK)
    {
        /* Power-up hp amp. */
        ak4954_upd_bits(ak4954, AK4954_REG_PM2, AK4954_PMHPR_MASK | AK4954_PMHPL_MASK, AK4954_PMHPR | AK4954_PMHPL);
    }

    UNLOCK_RTD(rtd);

    return result;
}


wiced_result_t ak4954_stop_play ( void* driver_data )
{
    ak4954_device_data_t *dd = ( ak4954_device_data_t* )driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    ak4954_device_runtime_data_t *rtd = &device_runtime_table[ak4954->id];
    wiced_result_t result = WICED_SUCCESS;

    LOCK_RTD(rtd);

    if (dd->route->intf->type ==  AK4954_DEVICE_TYPE_PLAYBACK)
    {
        /* Power-down HP amp. */
        ak4954_upd_bits(ak4954, AK4954_REG_PM2, AK4954_PMHPR_MASK | AK4954_PMHPL_MASK, 0);
    }

    UNLOCK_RTD(rtd);

    return result;
}


static wiced_result_t ak4954_set_playback_volume(void *driver_data, double decibels)
{
    ak4954_device_data_t *dd = (ak4954_device_data_t *)driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    uint8_t val;

    val = ak4954_double_range_to_index(ak4954, decibels, &ak4954_dbl_range_dvl);

    /* Volume is full width of register. */
    /* Setting left volume also sets right as long as defaults are maintained. */
    return ak4954_reg_write(ak4954, AK4954_REG_DVL_CTRL, val);
}


static wiced_result_t ak4954_set_capture_volume(void *driver_data, double decibels)
{
    ak4954_device_data_t *dd = (ak4954_device_data_t *)driver_data;
    ak4954_device_cmn_data_t *ak4954 = dd->cmn;
    uint8_t val;

    val = ak4954_double_range_to_index(ak4954, decibels, &ak4954_dbl_range_ivl);

    /* Volume is full width of register. */
    /* Setting left volume also sets right as long as defaults are maintained. */
    return ak4954_reg_write(ak4954, AK4954_REG_IVL_CTRL, val);
}


static wiced_result_t ak4954_get_playback_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    UNUSED_PARAMETER( driver_data );

    *min_volume_in_decibels = ak4954_dbl_range_dvl.min;
    *max_volume_in_decibels = ak4954_dbl_range_dvl.max;

    return WICED_SUCCESS;
}

static wiced_result_t ak4954_get_capture_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    UNUSED_PARAMETER( driver_data );

    *min_volume_in_decibels = ak4954_dbl_range_ivl.min;
    *max_volume_in_decibels = ak4954_dbl_range_ivl.max;

    return WICED_SUCCESS;
}

static wiced_result_t ak4954_ioctl(void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data)
{
    UNUSED_PARAMETER( driver_data );
    UNUSED_PARAMETER( cmd );
    UNUSED_PARAMETER( cmd_data );

    return WICED_UNSUPPORTED;
}

static wiced_result_t ak4954_init_device_runtime_data( ak4954_device_runtime_data_t *rtd )
{
    if ( rtd->rdy == 1 )
    {
        /* Already initialized */
        return WICED_SUCCESS;
    }

    WICED_VERIFY( wiced_rtos_init_mutex( &rtd->lock ) );

    rtd->init = 0;
    rtd->cfg  = 0;
    rtd->rdy  = 1;

    return WICED_SUCCESS;
}


/* This function can only be called from the platform initialization routine */
wiced_result_t ak4954_device_register( ak4954_device_data_t *device_data, const platform_audio_device_id_t device_id )
{
    if ( device_data == NULL )
    {
        return WICED_BADARG;
    }

    /* Initialize private portion of device interface. */
    device_data->route->intf->adi.audio_device_driver_specific = device_data;
    device_data->route->intf->adi.device_id = device_id;

    /* Initialize runtime data. */
    WICED_VERIFY( ak4954_init_device_runtime_data( &device_runtime_table[ device_data->cmn->id ] ) );

    /* Register a device to the audio device list and keep device data internally from this point */
    return wiced_register_audio_device( device_id, &device_data->route->intf->adi );
}


ak4954_audio_device_interface_t ak4954_playback =
{
    .type                               = AK4954_DEVICE_TYPE_PLAYBACK,
    .adi                                = {
        .audio_device_init              = ak4954_init,
        .audio_device_deinit            = ak4954_deinit,
        .audio_device_configure         = ak4954_configure,
        .audio_device_start_streaming   = ak4954_start_play,
        .audio_device_stop_streaming    = ak4954_stop_play,
        .audio_device_set_volume        = ak4954_set_playback_volume,
        .audio_device_get_volume_range  = ak4954_get_playback_volume_range,
        .audio_device_set_treble        = NULL,
        .audio_device_set_bass          = NULL,
        .audio_device_ioctl             = ak4954_ioctl,
    },
};


ak4954_audio_device_interface_t ak4954_capture =
{
    .type                               = AK4954_DEVICE_TYPE_CAPTURE,
    .adi                                = {
        .audio_device_init              = ak4954_init,
        .audio_device_deinit            = ak4954_deinit,
        .audio_device_configure         = ak4954_configure,
        .audio_device_start_streaming   = ak4954_start_play,
        .audio_device_stop_streaming    = ak4954_stop_play,
        .audio_device_set_volume        = ak4954_set_capture_volume,
        .audio_device_get_volume_range  = ak4954_get_capture_volume_range,
        .audio_device_set_treble        = NULL,
        .audio_device_set_bass          = NULL,
        .audio_device_ioctl             = ak4954_ioctl,
    },
};

const ak4954_device_route_t ak4954_dac_hp =
{
    .intf   = &ak4954_playback,
    .fn     = ak4954_hp,
};

const ak4954_device_route_t ak4954_dac_spkr =
{
    .intf   = &ak4954_playback,
    .fn     = ak4954_spkr,
};

const ak4954_device_route_t ak4954_adc_mic =
{
    .intf   = &ak4954_capture,
    .fn     = ak4954_mic,
};
