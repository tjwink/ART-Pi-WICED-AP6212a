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
#include <stdio.h>

#include "wiced_platform.h"
#include "wiced_audio.h"
#include "wiced_rtos.h"
#include "ak4961.h"
#include "wwd_constants.h"
#include "wwd_assert.h"
#include "platform_i2s.h"
#include "wiced_audio.h"
#include "wwd_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define LOCK_RTD(rtd)       wiced_rtos_lock_mutex(&(rtd)->lock)
#define UNLOCK_RTD(rtd)     wiced_rtos_unlock_mutex(&(rtd)->lock)

#if !defined(ARRAYSIZE)
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))
#endif
#define AK4961_VERIFY_GOTO(x, res, here) {res = (x); if (res != WICED_SUCCESS){wiced_assert(#x, 0==1); goto here;}}

#define AK4961_VERIFY(x) {wiced_result_t res = (x); if (res != WICED_SUCCESS){wiced_assert(#x, 0==1); return res;}}


/******************************************************
 *                    Constants
 ******************************************************/

#define AK4961_REG_FC1                      (0x0000)
#define AK4961_REG_FC2                      (0x0001)
#define AK4961_REG_FC3                      (0x0002)

#define AK4961_REG_PM1                      (0x0003)
#define AK4961_REG_PM2                      (0x0004)
#define AK4961_REG_PM3                      (0x0005)
#define AK4961_REG_PM4                      (0x0006)
#define AK4961_REG_PM5                      (0x0007)
#define AK4961_REG_PM6                      (0x0008)
#define AK4961_REG_PM8                      (0x000A)
#define AK4961_REG_PM9                      (0x000B)
#define AK4961_REG_PM10                     (0x000C)
#define AK4961_REG_LOUT2                    (0x000D)
#define AK4961_REG_MIC_PWR_LVL              (0x000F)

#define AK4961_REG_MIC_AMP_1_LCH_GAIN       (0x0010)
#define AK4961_REG_MIC_AMP_1_RCH_GAIN       (0x0011)
#define AK4961_REG_MIC_AMP_2_GAIN           (0x0012)
#define AK4961_REG_MIC_AMP_3_GAIN           (0x0013)
#define AK4961_REG_CM_SEL                   (0x001A)
#define AK4961_REG_DMIC_1                   (0x001E)
#define AK4961_REG_DMIC_2                   (0x001F)
#define AK4961_REG_DAC1_MONO_MIX            (0x0020)
#define AK4961_REG_DAC2_MONO_MIX            (0x0021)
#define AK4961_REG_MC                       (0x0026)
#define AK4961_REG_MIC_INPUT_SEL_1          (0x0034)
#define AK4961_REG_MIC_INPUT_SEL_2          (0x0035)
#define AK4961_REG_LCH_OUT_VOL_1            (0x0036)
#define AK4961_REG_RCH_OUT_VOL_1            (0x0037)
#define AK4961_REG_LCH_OUT_VOL_2            (0x0038)
#define AK4961_REG_RCH_OUT_VOL_2            (0x0039)
#define AK4961_REG_HP_VOL_CTRL              (0x003A)
#define AK4961_REG_LOUT1_VOL_CTRL           (0x003B)
#define AK4961_REG_LOUT2_VOL_CTRL           (0x003C)

#define AK4961_REG_PLL1_CLK_SRC_SEL         (0x003E)
#define AK4961_REG_PLL1_REF_CLK_DIV1        (0x003F)
#define AK4961_REG_PLL1_REF_CLK_DIV2        (0x0040)
#define AK4961_REG_PLL1_FB_CLK_DIV1         (0x0041)
#define AK4961_REG_PLL1_FB_CLK_DIV2         (0x0042)

#define AK4961_REG_MCLK1_SRC_SEL            (0x0048)
#define AK4961_REG_MCLK2_SRC_SEL            (0x004B)
#define AK4961_REG_MCLK3_SRC_SEL            (0x004E)

#define AK4961_REG_CODEC_CLK_SRC_SEL        (0x0064)
#define AK4961_REG_CODEC_CLK_DIV            (0x0065)
#define AK4961_REG_DSP_MCLK_SRC_SEL         (0x0066)
#define AK4961_REG_BUS_CLK_DIV              (0x0067)
#define AK4961_REG_AIN12_SYNC_DOMAIN_SET    (0x0068)
#define AK4961_REG_AIN34_SYNC_DOMAIN_SET    (0x0069)
#define AK4961_REG_ADC_SYNC_DOMAIN_SET      (0x006A)
#define AK4961_REG_SB3_DSPO_SYNC_DOMAIN_SET (0x006C)
#define AK4961_REG_DSPO24_SYNC_DOMAIN_SET   (0x006D)
#define AK4961_REG_SDTO1A_SRC_SEL           (0x0072)
#define AK4961_REG_SDTO1B_SRC_SEL           (0x0073)
#define AK4961_REG_SDTO2_SRC_SEL            (0x0074)
#define AK4961_REG_SDTO3_SRC_SEL            (0x0075)
#define AK4961_REG_SDTO4_SRC_SEL            (0x0076)
#define AK4961_REG_DAC1_SRC_SEL             (0x0079)
#define AK4961_REG_DAC2_SRC_SEL             (0x007A)
#define AK4961_REG_DSPI1_SRC_SEL            (0x007B)
#define AK4961_REG_DSPI2_SRC_SEL            (0x007C)
#define AK4961_REG_DSPI3_SRC_SEL            (0x007D)
#define AK4961_REG_DSPI4_SRC_SEL            (0x007E)
#define AK4961_REG_DSPI5_SRC_SEL            (0x007F)

#define AK4961_REG_SDTIO1_IF_FORMAT         (0x008B)
#define AK4961_REG_SDTIO2_IF_FORMAT         (0x008C)
#define AK4961_REG_SDTIO3_IF_FORMAT         (0x008D)
#define AK4961_REG_SDTIO4_IF_FORMAT         (0x008E)
#define AK4961_REG_CODEC_IF_FORMAT          (0x008F)

#define AK4961_REG_DSP_SETTING1             (0x0094)
#define AK4961_REG_DSP_SETTING2             (0x0096)
#define AK4961_REG_DSP_SETTING3             (0x0097)
#define AK4961_REG_DSP_SETTING4             (0x0098)

#define AK4961_REG_CRAM_OP_RUN_STATE        (0x00CF)

#define AK4961_REG_PRAM_READY               (0x00D1)

#define AK4961_REG_CRC_RESULT1              (0x00D3)
#define AK4961_REG_CRC_RESULT2              (0x00D4)

/* 00H Flow Control 1 */
#define AK4961_FC1_PMSW                     (0x1)
#define AK4961_FC1_PMSW_MASK                (0x01)
#define AK4961_FC1_PMSW_SHIFT               (0)
#define AK4961_FC1_PMSW_BITS                (1)
#define AK4961_FC1_SWRSTN                   (0x2)
#define AK4961_FC1_SWRSTN_MASK              (0x02)
#define AK4961_FC1_SWRSTN_SHIFT             (1)
#define AK4961_FC1_SWRSTN_BITS              (1)

/* 01H Flow Control 2 */
#define AK4961_FC2_DLRDY                    (0x1)
#define AK4961_FC2_DLRDY_MASK               (0x01)
#define AK4961_FC2_DLRDY_SHIFT              (0)
#define AK4961_FC2_DLRDY_BITS               (1)

/* 02H Flow Control 3 */
#define AK4961_FC3_DSPRSTN                  (0x1)
#define AK4961_FC3_DSPRSTN_MASK             (0x01)
#define AK4961_FC3_DSPRSTN_SHIFT            (0)
#define AK4961_FC3_DSPRSTN_BITS             (1)

/* 03H Power Management 1 */
#define AK4961_PM1_PMPLL1                   (0x1)
#define AK4961_PM1_PMPLL1_MASK              (0x01)
#define AK4961_PM1_PMPLL1_SHIFT             (0)
#define AK4961_PM1_PMPLL11_BITS             (1)
#define AK4961_PM1_PMPLL2                   (0x2)
#define AK4961_PM1_PMPLL2_MASK              (0x02)
#define AK4961_PM1_PMPLL2_SHIFT             (1)
#define AK4961_PM1_PMPLL2_BITS              (1)

/* 04H Power Management 2 */
#define AK4961_PM2_PMAIF                    (0x10)
#define AK4961_PM2_PMAIF_MASK               (0x10)
#define AK4961_PM2_PMAIF_SHIFT              (4)
#define AK4961_PM2_PMAIF_BITS               (1)

/* 05H Power Management 3 */
#define AK4961_PM3_PMCP1_SET                (0x01)
#define AK4961_PM3_PMCP1_MASK               (0x01)
#define AK4961_PM3_PMCP1_SHIFT              (0)
#define AK4961_PM3_PMCP1_BITS               (1)
#define AK4961_PM3_PMCP2_SET                (0x02)
#define AK4961_PM3_PMCP2_MASK               (0x02)
#define AK4961_PM3_PMCP2_SHIFT              (1)
#define AK4961_PM3_PMCP2_BITS               (1)
#define AK4961_PM3_PMCP3_SET                (0x04)
#define AK4961_PM3_PMCP3_MASK               (0x04)
#define AK4961_PM3_PMCP3_SHIFT              (2)
#define AK4961_PM3_PMCP3_BITS               (1)
#define AK4961_PM3_PMLDO1_SET               (0x10)
#define AK4961_PM3_PMLDO1_MASK              (0x10)
#define AK4961_PM3_PMLDO1_SHIFT             (4)
#define AK4961_PM3_PMLDO1_BITS              (1)
#define AK4961_PM3_PMLDO3P_SET              (0x40)
#define AK4961_PM3_PMLDO3P_MASK             (0x40)
#define AK4961_PM3_PMLDO3P_SHIFT            (6)
#define AK4961_PM3_PMLDO3P_BITS             (1)
#define AK4961_PM3_PMLDO3N_SET              (0x80)
#define AK4961_PM3_PMLDO3N_MASK             (0x80)
#define AK4961_PM3_PMLDO3N_SHIFT            (7)
#define AK4961_PM3_PMLDO3N_BITS             (1)

/* 06H Power Management 4 */
#define AK4961_PM4_PMMP1A                   (0x1)
#define AK4961_PM4_PMMP1A_MASK              (0x01)

/* 07H Power Management 5 */
#define AK4961_PM5_PMAIN1                   (0x1)
#define AK4961_PM5_PMAIN1_MASK              (0x01)

/* 08H Power Management 6 */
#define AK4961_PM6_PMAD1L                   (0x1)
#define AK4961_PM6_PMAD1L_MASK              (0x01)
#define AK4961_PM6_PMAD1L_SHIFT             (0)
#define AK4961_PM6_PMAD1L_BITS              (1)
#define AK4961_PM6_PMAD1R_MASK              (0x02)
#define AK4961_PM6_PMAD1R_SHIFT             (1)
#define AK4961_PM6_PMAD1R_BITS              (1)
#define AK4961_PM6_PMAD2M_MASK              (0x04)
#define AK4961_PM6_PMAD2M_SHIFT             (2)
#define AK4961_PM6_PMAD2M_BITS              (1)
#define AK4961_PM6_PMAD2V_MASK              (0x08)
#define AK4961_PM6_PMAD2V_SHIFT             (3)
#define AK4961_PM6_PMAD2V_BITS              (1)

/* 0AH Power Management 8 */
#define AK4961_PM8_PMDA1_MASK               (0x01)
#define AK4961_PM8_PMDA1_SHIFT              (0)
#define AK4961_PM8_PMDA1_BITS               (1)
#define AK4961_PM8_PMDA2_MASK               (0x04)
#define AK4961_PM8_PMDA2_SHIFT              (2)
#define AK4961_PM8_PMDA2_BITS               (1)

/* 0BH Power Management 9 */
#define AK4961_PM9_PMHP_LR                  (0x03)
#define AK4961_PM9_PMHPL_MASK               (0x01)
#define AK4961_PM9_PMHPL_SHIFT              (0)
#define AK4961_PM9_PMHPR_MASK               (0x02)
#define AK4961_PM9_PMHPR_SHIFT              (1)
#define AK4961_PM9_PMLO1L_MASK              (0x10)
#define AK4961_PM9_PMLO1L_SHIFT             (4)
#define AK4961_PM9_PMLO1R_MASK              (0x20)
#define AK4961_PM9_PMLO1R_SHIFT             (5)

/* 0CH Power Management 10 */
#define AK4961_PM10_PMLO2LP_MASK            (0x01)
#define AK4961_PM10_PMLO2LP_SHIFT           (0)
#define AK4961_PM10_PMLO2LP_BITS            (1)
#define AK4961_PM10_PMLO2LN_MASK            (0x02)
#define AK4961_PM10_PMLO2LN_SHIFT           (1)
#define AK4961_PM10_PMLO2LN_BITS            (1)
#define AK4961_PM10_PMLO2RP_MASK            (0x10)
#define AK4961_PM10_PMLO2RP_SHIFT           (4)
#define AK4961_PM10_PMLO2RP_BITS            (1)
#define AK4961_PM10_PMLO2RN_MASK            (0x20)
#define AK4961_PM10_PMLO2RN_SHIFT           (5)
#define AK4961_PM10_PMLO2RN_BITS            (1)

/* 0DH Lineout2 Setting */
#define AK4961_LOUT2_LO2DIF_MASK            (0x01)
#define AK4961_LOUT2_LO2DIF_SHIFT           (0)
#define AK4961_LOUT2_LO2DIF_BITS            (1)

/* 0FH MIC Power Level */
#define AK4961_MIC_PWR_LVL_MICL1_MASK       (0x03)
#define AK4961_MIC_PWR_LVL_MICL1_SHIFT      (0)
#define AK4961_MIC_PWR_LVL_MICL1_BITS       (2)
#define AK4961_MIC_PWR_LVL_MICL2_MASK       (0x0C)
#define AK4961_MIC_PWR_LVL_MICL2_SHIFT      (2)
#define AK4961_MIC_PWR_LVL_MICL2_BITS       (2)

/* 10H-13H MIC-Amp Gain */
#define AK4961_MIC_AMP_GAIN_MASK            (0x0F)
#define AK4961_MIC_AMP_GAIN_SHIFT           (0)
#define AK4961_MIC_AMP_GAIN_BITS            (4)

#define AK4961_MIC_AMP_1_LCH_GAIN_DEFAULT (0x00) //0dB

/* 1AH Clock Mode Select */
#define AK4961_CM_SEL_CM_256FS              (0x00)
#define AK4961_CM_SEL_CM_512FS              (0x20)
#define AK4961_CM_SEL_CM_1024FS             (0x40)
#define AK4961_CM_SEL_CM_128FS              (0x60)
#define AK4961_CM_SEL_CM_MASK               (0x60)      /* [6:5] */
#define AK4961_CM_SEL_CM_SHIFT              (5)
#define AK4961_CM_SEL_CM_BITS               (2)
#define AK4961_CM_SEL_FS_8KHZ               (0x00)
#define AK4961_CM_SEL_FS_11P025KHZ          (0x01)
#define AK4961_CM_SEL_FS_12KHZ              (0x02)
#define AK4961_CM_SEL_FS_16KHZ              (0x04)
#define AK4961_CM_SEL_FS_22P05KHZ           (0x05)
#define AK4961_CM_SEL_FS_24KHZ              (0x06)
#define AK4961_CM_SEL_FS_32KHZ              (0x08)
#define AK4961_CM_SEL_FS_44P1KHZ            (0x09)
#define AK4961_CM_SEL_FS_48KHZ              (0x0A)
#define AK4961_CM_SEL_FS_64KHZ              (0x0C)
#define AK4961_CM_SEL_FS_88P2KHZ            (0x0D)
#define AK4961_CM_SEL_FS_96KHZ              (0x0E)
#define AK4961_CM_SEL_FS_128KHZ             (0x10)
#define AK4961_CM_SEL_FS_176P4KHZ           (0x11)
#define AK4961_CM_SEL_FS_192KHZ             (0x12)
#define AK4961_CM_SEL_FS_MASK               (0x1F)      /* [4:0] */
#define AK4961_CM_SEL_FS_SHIFT              (0)
#define AK4961_CM_SEL_FS_BITS               (5)

/* 1EH-1FH Digital MIC 1/2 */
#define AK4961_DMIC_DMIC_SEL                (0x01)
#define AK4961_DMIC_DMIC_MASK               (0x01)
#define AK4961_DMIC_DMIC_SHIFT              (0)
#define AK4961_DMIC_DMIC_BITS               (1)
#define AK4961_DMIC_DCLKP_MASK              (0x02)
#define AK4961_DMIC_DCLKP_SHIFT             (1)
#define AK4961_DMIC_DCLKP_BITS              (1)
#define AK4961_DMIC_DCLKE_MASK              (0x08)
#define AK4961_DMIC_DCLKE_SHIFT             (3)
#define AK4961_DMIC_DCLKE_BITS              (1)
#define AK4961_DMIC_PMDML_MASK              (0x10)
#define AK4961_DMIC_PMDML_SHIFT             (4)
#define AK4961_DMIC_PMDML_BITS              (1)
#define AK4961_DMIC_PMDMR_MASK              (0x20)
#define AK4961_DMIC_PMDMR_SHIFT             (5)
#define AK4961_DMIC_PMDMR_BITS              (1)

/* 20H-21H DAC Mono Mixing */
#define AK4961_DAC_MIX_LEFT_MASK            (0x07)
#define AK4961_DAC_MIX_LEFT_SHIFT           (0)
#define AK4961_DAC_MIX_LEFT_BITS            (3)
#define AK4961_DAC_MIX_RIGHT_MASK           (0x70)
#define AK4961_DAC_MIX_RIGHT_SHIFT          (4)
#define AK4961_DAC_MIX_RIGHT_BITS           (3)

/* 26H Mode Control */
#define AK4961_MC_DSMLP1_LOW_SPEED          (0x40)
#define AK4961_MC_DSMLP1_STD_SPEED          (0x00)
#define AK4961_MC_DSMLP1_MASK               (0x40)
#define AK4961_MC_DSMLP1_SHIFT              (6)
#define AK4961_MC_DSMLP1_BITS               (1)
#define AK4961_MC_DSMLP2_LOW_SPEED          (0x80)
#define AK4961_MC_DSMLP2_STD_SPEED          (0x00)
#define AK4961_MC_DSMLP2_MASK               (0x80)
#define AK4961_MC_DSMLP2_SHIFT              (7)
#define AK4961_MC_DSMLP2_BITS               (1)

/* 34H-35H MIC Input Selector */
#define AK4961_MIC_INPUT_SEL_RIGHT_MASK     (0x70)
#define AK4961_MIC_INPUT_SEL_RIGHT_SHIFT    (4)
#define AK4961_MIC_INPUT_SEL_RIGHT_BITS     (3)
#define AK4961_MIC_INPUT_SEL_LEFT_MASK      (0x07)
#define AK4961_MIC_INPUT_SEL_LEFT_SHIFT     (0)
#define AK4961_MIC_INPUT_SEL_LEFT_BITS      (3)

/* 36H-39H LCH/RCH Output Volume 1/2 */
#define AK4961_OUT_VOL_MASK                 (0x1F)
#define AK4961_OUT_VOL_SHIFT                (0)
#define AK4961_OUT_VOL_BITS                 (5)

/* 3AH HP Volume Control */
#define AK4961_HP_VOL_CTRL_HPG_0DB_DEFAULT  ((0x15) << AK4961_HP_VOL_CTRL_HPG_SHIFT)
#define AK4961_HP_VOL_CTRL_HPG_MASK         (0x1F)
#define AK4961_HP_VOL_CTRL_HPG_SHIFT        (0)
#define AK4961_HP_VOL_CTRL_HPG_BITS         (5)

/* 3BH LOUT1 Volume Control */
#define AK4961_LOUT1_VOL_CTRL_LO1G_0DB_DEFAULT  ((0x05) << AK4961_LOUT1_VOL_CTRL_LO1G_SHIFT)
#define AK4961_LOUT1_VOL_CTRL_LO1G_MASK     (0x07)
#define AK4961_LOUT1_VOL_CTRL_LO1G_SHIFT    (0)
#define AK4961_LOUT1_VOL_CTRL_LO1G_BITS     (3)

/* 3CH LOUT2 Volume Control */
#define AK4961_LOUT2_VOL_CTRL_LO2G_0DB_DEFAULT  ((0x05) << AK4961_LOUT2_VOL_CTRL_LO2G_SHIFT)
#define AK4961_LOUT2_VOL_CTRL_LO2G_MASK     (0x70)
#define AK4961_LOUT2_VOL_CTRL_LO2G_SHIFT    (4)
#define AK4961_LOUT2_VOL_CTRL_LO2G_BITS     (3)

/* 3EH PLL1 CLK Source Select */
#define AK4961_PLL1_CLK_SRC_SEL_PLS1_MASK   (0x1F)
#define AK4961_PLL1_CLK_SRC_SEL_PLS1_SHIFT  (0)
#define AK4961_PLL1_CLK_SRC_SEL_PLS1_BITS   (5)

/* 3FH-42H PLL1 reference clock and feedback divider */
#define AK4961_PLL1_CLK_DIV_PL1_MASK        (0xFF)
#define AK4961_PLL1_CLK_DIV_PL1_SHIFT       (0)
#define AK4961_PLL1_CLK_DIV_PL1_BITS        (8)

/* 48H-5AH MCLKx Source Select */
#define AK4961_MCLK_SRC_SEL_CKS_MASK        (0x1F)
#define AK4961_MCLK_SRC_SEL_CKS_SHIFT       (0)
#define AK4961_MCLK_SRC_SEL_CKS_BITS        (5)
#define AK4961_MCLK_SRC_SEL_MSN             (0x20)
#define AK4961_MCLK_SRC_SEL_MSN_MASK        (0x20)
#define AK4961_MCLK_SRC_SEL_MSN_SHIFT       (5)
#define AK4961_MCLK_SRC_SEL_MSN_BITS        (1)

/* 66H DSP MCLK Source Select */
#define AK4961_DSP_MCLK_SRC_SEL_CKSD_MASK   (0x1F)
#define AK4961_DSP_MCLK_SRC_SEL_CKSD_SHIFT  (0)
#define AK4961_DSP_MCLK_SRC_SEL_CKSD_BITS   (5)

/* 67H BUS CLK Divider */
#define AK4961_BUS_CLK_DIV_MDIVD_MASK       (0xFF)
#define AK4961_BUS_CLK_DIV_MDIVD_SHIFT      (0)
#define AK4961_BUS_CLK_DIV_MDIVD_BITS       (8)

/* 64H Codec clock Source Select */
#define AK4961_CODEC_CLK_SRC_SEL_MCKS2_MASK (0x1F)

/* 65H Codec clock divider */
#define AK4961_CODEC_CLK_DIV_MDIV2_MASK     (0xFF)
#define AK4961_CODEC_CLK_DIV_MDIV2_SHIFT    (0)
#define AK4961_CODEC_CLK_DIV_MDIV2_BITS     (8)

/* 68H AIN1/2 sync domain setting */
#define AK4961_AIN12_SYNC_SET_SDAIF1_MASK   (0x70)
#define AK4961_AIN12_SYNC_SET_SDAIF1_SHIFT  (4)
#define AK4961_AIN12_SYNC_SET_SDAIF1_BITS   (3)
#define AK4961_AIN12_SYNC_SET_SDAIF2_MASK   (0x07)
#define AK4961_AIN12_SYNC_SET_SDAIF2_SHIFT  (0)
#define AK4961_AIN12_SYNC_SET_SDAIF2_BITS   (3)

/* 69H AIN3/4 sync domain setting */
#define AK4961_AIN34_SYNC_SET_SDAIF3_MASK   (0x70)
#define AK4961_AIN34_SYNC_SET_SDAIF3_SHIFT  (4)
#define AK4961_AIN34_SYNC_SET_SDAIF3_BITS   (3)
#define AK4961_AIN34_SYNC_SET_SDAIF4_MASK   (0x07)
#define AK4961_AIN34_SYNC_SET_SDAIF4_SHIFT  (0)
#define AK4961_AIN34_SYNC_SET_SDAIF4_BITS   (3)

/* 6AH ADC sync domain setting */
#define AK4961_ADC_SYNC_SET_SDCDC_MASK      (0x07)
#define AK4961_ADC_SYNC_SET_SDCDC_SHIFT     (0)
#define AK4961_ADC_SYNC_SET_SDCDC_BITS      (3)

/* 6CH SB3/DSPO sync domain setting */
#define AK4961_SB3_SYNC_SET_SDSB3_MASK      (0x70)
#define AK4961_SB3_SYNC_SET_SDSB3_SHIFT     (4)
#define AK4961_SB3_SYNC_SET_SDSB3_BITS      (3)
#define AK4961_DSPO_SYNC_SET_SDDSP_MASK     (0x07)
#define AK4961_DSPO_SYNC_SET_SDDSP_SHIFT    (0)
#define AK4961_DSPO_SYNC_SET_SDDSP_BITS     (3)

/* 6DH DSPO2/4 sync domain setting */
#define AK4961_DSPO2_SYNC_SET_SDDSPO2_MASK  (0x70)
#define AK4961_DSPO2_SYNC_SET_SDDSPO2_SHIFT (4)
#define AK4961_DSPO2_SYNC_SET_SDDSPO2_BITS  (3)
#define AK4961_DSPO4_SYNC_SET_SDDSPO4_MASK  (0x07)
#define AK4961_DSPO4_SYNC_SET_SDDSPO4_SHIFT (0)
#define AK4961_DSPO4_SYNC_SET_SDDSPO4_BITS  (3)

/* 72H-88H Source Select */
#define AK4961_SRC_SEL_MASK                 (0x1F)
#define AK4961_SRC_SEL_SHIFT                (0)
#define AK4961_SRC_SEL_BITS                 (5)

/* 8BH SDTIOA/B/C/D I/F format */
#define AK4961_SDTIO1_IF_FMT_DLC1_24BIT     (0x00)
#define AK4961_SDTIO1_IF_FMT_DLC1_16BIT     (0x01)
#define AK4961_SDTIO1_IF_FMT_DLC1_32BIT     (0x04)
#define AK4961_SDTIO1_IF_FMT_DLC1_MASK      (0x07)

/* 94H DSP Setting 1 */
#define AK4961_DSP_SET1_DRMS_MASK           (0xC0)
#define AK4961_DSP_SET1_DRMS_SHIFT          (6)
#define AK4961_DSP_SET1_DRMS_BITS           (2)
#define AK4961_DSP_SET1_DRAD_MASK           (0x30)
#define AK4961_DSP_SET1_DRAD_SHIFT          (4)
#define AK4961_DSP_SET1_DRAD_BITS           (2)
#define AK4961_DSP_SET1_BANK_MASK           (0x0F)
#define AK4961_DSP_SET1_BANK_SHIFT          (0)
#define AK4961_DSP_SET1_BANK_BITS           (4)

/* 96H DSP Setting 2 */
#define AK4961_DSP_SET2_POMODE_MASK         (0x08)
#define AK4961_DSP_SET2_POMODE_SHIFT        (3)
#define AK4961_DSP_SET2_POMODE_BITS         (1)
#define AK4961_DSP_SET2_WAVP_MASK           (0x03)
#define AK4961_DSP_SET2_WAVP_SHIFT          (0)
#define AK4961_DSP_SET2_WAVP_BITS           (2)

/* 97H DSP Setting 3 */
#define AK4961_DSP_SET3_EDNOPDIS_MASK       (0x10)
#define AK4961_DSP_SET3_EDNOPDIS_SHIFT      (4)
#define AK4961_DSP_SET3_EDNOPDIS_BITS       (1)

/* 98H DSP Setting 4 */
#define AK4961_DSP_SET4_DSPCKADJ_MASK       (0xFF)
#define AK4961_DSP_SET4_DSPCKADJ_SHIFT      (0)
#define AK4961_DSP_SET4_DSPCKADJ_BITS       (8)

/* CFH CRAM Operation during Run State */
#define AK4961_CRAM_OP_RUNC                 (0x1)
#define AK4961_CRAM_OP_MASK                 (0x01)
#define AK4961_CRAM_OP_SHIFT                (0)
#define AK4961_CRAM_OP_BITS                 (1)

/* D1H PRAM Ready */
#define AK4961_PRAM_READY_PRIF              (0x1)
#define AK4961_PRAM_READY_MASK              (0x01)
#define AK4961_PRAM_READY_SHIFT             (0)
#define AK4961_PRAM_READY_BITS              (1)

/* D3H-D4H CRC Result 1&2 */
#define AK4961_CRC_RESULT1_MASK             (0xff)
#define AK4961_CRC_RESULT1_SHIFT            (0)
#define AK4961_CRC_RESULT1_BITS             (8)
#define AK4961_CRC_RESULT2_MASK             (0xff)
#define AK4961_CRC_RESULT2_SHIFT            (0)
#define AK4961_CRC_RESULT2_BITS             (8)
#define AK4961_CRC_RESULT_BYTES             (2)

/* Default input volume gain settings. */
#define AK4961_IVL_0DB_DEFAULT              (0x00)

/* MIC-Amp gain settings (Table 18). */
#define AK4961_IVL_GAIN_IN_DB_MIN           (+0.0)
#define AK4961_IVL_GAIN_IN_DB_MAX           (+30.0)
#define AK4961_IVL_GAIN_IN_DB_STEP          (+3.0)

/* Output digital volume gain settings (Table 69). */
#if FIXME
#define AK4961_DVL_GAIN_IN_DB_MIN           (-65.5)
#define AK4961_DVL_GAIN_IN_DB_MAX           (+6.0)
#define AK4961_DVL_GAIN_IN_DB_STEP          (+0.5)
#endif

/* HP Analog volume gain settings (Table 36). */
#define AK4961_HP_GAIN_IN_DB_MIN            (-40.0 + (-2))
#define AK4961_HP_GAIN_IN_DB_MAX            (+6.0)
#define AK4961_HP_GAIN_IN_DB_STEP           (+2.0)

/* Line-out Analog volume gain settings (Table 40/44). */
#define AK4961_LOUT_GAIN_IN_DB_MIN          (-7.5)
#define AK4961_LOUT_GAIN_IN_DB_MAX          (+3.0)
#define AK4961_LOUT_GAIN_IN_DB_STEP         (+1.5)

/* Pulse width for PDN reset. */
#define AK4961_RESET_HOLD_TIME_IN_MILLIS    (1)

/* CP1 power-up wait time (6.5 ms). */
#define AK4961_PMCP1_WAIT_IN_MILLIS         (7)

/* CP2 power-up wait time (4.5 ms). */
#define AK4961_PMCP2_WAIT_IN_MILLIS         (5)

/* CP3 power-up wait time (6.5 ms). */
#define AK4961_PMCP3_WAIT_IN_MILLIS         (7)

/* LDO1 power-up wait time (0.3 ms). */
#define AK4961_PMLDO1_WAIT_IN_MILLIS        (1)

/* LDO3N/P power-up wait time. */
#define AK4961_PMLDO3NP_WAIT_IN_MILLIS      (1)

/* HP-AMP power-up wait time (26 ms). */
#define AK4961_PM9_PMHP_WAIT_IN_MILLIS      (26)

/* LOUT1-AMP power-up wait time (4.4ms). */
#define AK4961_PM9_PMLO1_WAIT_IN_MILLIS     (5)

/* LOUT2-AMP power-up wait time (4.4ms). */
#define AK4961_PM10_PMLO2_WAIT_IN_MILLIS    (5)

/* Clock source address (Table 88). */
#define AK4961_CLK_SRC_ADDR_TIE_LOW         (0x00)
#define AK4961_CLK_SRC_ADDR_PLLCLK1         (0x01)
#define AK4961_CLK_SRC_ADDR_MCKI1           (0x04)
#define AK4961_CLK_SRC_ADDR_BCLK3           (0x0A)
#define AK4961_CLK_SRC_ADDR_SYNC3           (0x11)

#define AK4961_CP_LDO_CP1                   (1U << 0)
#define AK4961_CP_LDO_CP2                   (1U << 1)
#define AK4961_CP_LDO_CP3                   (1U << 2)
#define AK4961_CP_LDO_LDO1                  (1U << 3)
#define AK4961_CP_LDO_LDO2                  (1U << 4)
#define AK4961_CP_LDO_LDO3PN                (1U << 5)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct ak4961_adc_fs_map            ak4961_adc_fs_map_t;
typedef struct ak4961_dac_fs_map            ak4961_dac_fs_map_t;
typedef struct ak4961_fs_map                ak4961_fs_map_t;
typedef struct ak4961_fs_mcki_map           ak4961_fs_mcki_map_t;
typedef struct ak4961_fs_bitclock_map       ak4961_fs_bitclock_map_t;
typedef struct ak4961_fs2cm_map             ak4961_fs2cm_map_t;
typedef struct ak4961_dbl_range             ak4961_dbl_range_t;
typedef struct ak4961_sync_domain           ak4961_sync_domain_t;
typedef struct ak4961_amp                   ak4961_amp_t;
typedef struct ak4961_dac                   ak4961_dac_t;
typedef struct ak4961_adc                   ak4961_adc_t;

typedef enum ak4961_source_address_select   ak4961_source_address_select_t;


/******************************************************
 *                   Enumerations
 ******************************************************/

enum ak4961_source_address_select
{
    AK4961_SOURCE_ADDRESS_SELECT_ALL0   = 0x00,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI1A = 0x01,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI1B = 0x02,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI1C = 0x03,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI1D = 0x04,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI2  = 0x05,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI3  = 0x06,
    AK4961_SOURCE_ADDRESS_SELECT_SDTI4  = 0x07,
    AK4961_SOURCE_ADDRESS_SELECT_ADC1   = 0x09,
    AK4961_SOURCE_ADDRESS_SELECT_ADC2   = 0x0A,
    AK4961_SOURCE_ADDRESS_SELECT_DSPO1  = 0x0E,
    AK4961_SOURCE_ADDRESS_SELECT_DSPO2  = 0x0F,
    AK4961_SOURCE_ADDRESS_SELECT_DSPO3  = 0x10,
    AK4961_SOURCE_ADDRESS_SELECT_DSPO4  = 0x11,
    AK4961_SOURCE_ADDRESS_SELECT_DSPO5  = 0x12,
};


/******************************************************
 *                    Structures
 ******************************************************/

struct ak4961_adc_fs_map
{
    uint32_t        fs1;
    uint32_t        fs2;
    uint16_t        mck_div_fs;
    uint8_t         cm;
};

struct ak4961_dac_fs_map
{
    uint32_t        fs1;
    uint32_t        fs2;
    uint16_t        mck_div_fs;
    uint8_t         speed;
    uint8_t         cm;
};

struct ak4961_fs_map
{
    uint32_t        fs;
    uint8_t         fsbits;
};

struct ak4961_fs_mcki_map
{
    uint32_t        fs;
    uint32_t        mcki;
    uint16_t        pll1_ref_div;
    uint16_t        pll1_fb_div;
    uint16_t        mdiv2;
    uint16_t        mdivd;
};

struct ak4961_fs_bitclock_map
{
    uint32_t        fs;
    uint32_t        bitclock;
    uint16_t        pll1_ref_div;
    uint16_t        pll1_fb_div;
    uint16_t        mdiv2;
    uint16_t        mdivd;
};

struct ak4961_fs2cm_map
{
    uint32_t        fs1;
    uint32_t        fs2;
    uint16_t        mck_div_fs;
    uint8_t         speed;
    uint8_t         cm;
};

struct ak4961_dac
{
    uint8_t                         pm8_pmda_mask;
    uint16_t                        reg_dac_mono_mix;
    uint16_t                        reg_dac_src_sel;
    uint16_t                        reg_lch_out_vol;
};

struct ak4961_dsp
{
    uint16_t                        reg_dsp_src_sel;
};

struct ak4961_adc
{
    ak4961_source_address_select_t  source_address_select;
    uint8_t                         pm6_pmad_mask;
    uint8_t                         pm6_pmad_shift;
    uint16_t                        reg_dmic;
    uint16_t                        reg_mic_input_select;
    uint16_t                        reg_mic_amp_lch_gain;
    uint16_t                        reg_mic_amp_rch_gain;
};

struct ak4961_sync_domain
{
    uint16_t                        reg;
    uint8_t                         shift;
    uint8_t                         mask;
};

struct ak4961_amp
{
    /* 0=Single-ended, 1=Differential. */
    uint8_t                         is_differential : 1;
    uint16_t                        reg_pmamp;
    uint16_t                        reg_vol_ctrl;
    uint8_t                         pmamp_left_mask;
    uint8_t                         pmamp_right_mask;
    uint8_t                         default_gain;
    uint8_t                         vol_ctrl_gain_mask;
    uint8_t                         vol_ctrl_gain_shift;
    const ak4961_dbl_range_t        *gain_range;
    uint32_t                        pwr_up_time_in_millis;
};

struct ak4961_dac_route
{
    const ak4961_dac_t              *dac;
    const ak4961_amp_t              *amp;
};

struct ak4961_adc_route
{
    uint8_t                         is_dmic : 1;
    const ak4961_adc_t              *adc;

    /* NULL for DMIC. */
    const ak4961_dbl_range_t        *mic_amp_gain_range;
};

struct ak4961_source_port
{
    ak4961_source_address_select_t  source_address_select;
    const ak4961_sync_domain_t      *sync_domain;
};

struct ak4961_sink_port
{
    uint16_t                        reg_source_address;
    const ak4961_sync_domain_t      *sync_domain;
};

struct ak4961_dbl_range
{
    double min;
    double max;
    double step;

    /* The range is inverted: descending from position 0. */
    wiced_bool_t inv;
};


/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef USE_AK4961_MAP
static const ak4961_adc_fs_map_t ak4961_adc_fs_map[] =
{
    {   8000,  96000,  256, AK4961_CM_SEL_CM_256FS  },
    {   8000,  48000,  512, AK4961_CM_SEL_CM_512FS  },
    {   8000,  24000, 1024, AK4961_CM_SEL_CM_1024FS },
};

static const ak4961_dac_fs_map_t ak4961_dac1_fs_map[] =
{
    {   8000,  12000,  256, AK4961_MC_DSMLP1_LOW_SPEED, AK4961_CM_SEL_CM_256FS  },
    {  16000,  96000,  256, AK4961_MC_DSMLP1_STD_SPEED, AK4961_CM_SEL_CM_256FS  },
    {   8000,  48000,  512, AK4961_MC_DSMLP1_STD_SPEED, AK4961_CM_SEL_CM_512FS  },
    {   8000,  24000, 1024, AK4961_MC_DSMLP1_STD_SPEED, AK4961_CM_SEL_CM_1024FS },
    { 128000, 192000,  128, AK4961_MC_DSMLP1_STD_SPEED, AK4961_CM_SEL_CM_128FS  },
};

static const ak4961_dac_fs_map_t ak4961_dac2_fs_map[] =
{
    {  16000,  24000,  256, AK4961_MC_DSMLP2_LOW_SPEED, AK4961_CM_SEL_CM_256FS  },
    {  32000,  96000,  256, AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_256FS  },
    {   8000,  12000,  512, AK4961_MC_DSMLP2_LOW_SPEED, AK4961_CM_SEL_CM_512FS  },
    {  16000,  48000,  512, AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_512FS  },
    {   8000,  24000, 1024, AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_1024FS },
    { 128000, 192000,  128, AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_128FS  },
};
#endif /* #ifdef USE_AK4961_MAP */

static const ak4961_fs_map_t fs_map[] =
{
    {   8000, AK4961_CM_SEL_FS_8KHZ      },
    {  11025, AK4961_CM_SEL_FS_11P025KHZ },
    {  12000, AK4961_CM_SEL_FS_12KHZ     },
    {  16000, AK4961_CM_SEL_FS_16KHZ     },
    {  22050, AK4961_CM_SEL_FS_22P05KHZ  },
    {  24000, AK4961_CM_SEL_FS_24KHZ     },
    {  32000, AK4961_CM_SEL_FS_32KHZ     },
    {  44100, AK4961_CM_SEL_FS_44P1KHZ   },
    {  48000, AK4961_CM_SEL_FS_48KHZ     },
    {  64000, AK4961_CM_SEL_FS_64KHZ     },
    {  88200, AK4961_CM_SEL_FS_88P2KHZ   },
    {  96000, AK4961_CM_SEL_FS_96KHZ     },
    { 128000, AK4961_CM_SEL_FS_128KHZ    },
    { 176400, AK4961_CM_SEL_FS_176P4KHZ  },
    { 192000, AK4961_CM_SEL_FS_192KHZ    },
};

/* XXX We don't need to use the PLL if I2S and the CODEC can negotiate
 * a MCLK/fs ratio.
 */

/* Updated to correspond to the MCLK set by 4390x (i2s_core.h, i2s_clkdiv_coeffs).
     However, this differs from table 5 of BCM43907 Preliminary Data Sheet */

/*
 *  pll_out = mclk * ( (fb_div + 1) / (ref_div + 1) )
 */

/* The divider values are chosen such that the output freq is fs*(cm value) */
ak4961_fs_mcki_map_t ak4961_fs2mcki_map[] =
{
/*       fs            mclk           Ref div        Pll Ref Clk    fb div      mdiv2      codec mclk    mdivd      dsp mclk     */
    {    8000,        12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    11025,       11289600,       0x0003,        /*2822.4*/     0x0027,     0x0004,     /*512*/      0x0004,   /*112.896*/ },
    {    12000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    16000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    22050,       11289600,       0x0003,        /*2822.4*/     0x0027,     0x0004,     /*512*/      0x0004,   /*112.896*/ },
    {    24000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    32000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    44100,       11289600,       0x0003,        /*2822.4*/     0x0027,     0x0004,     /*512*/      0x0004,   /*112.896*/ },
    {    48000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*512*/      0x0004,   /*122.88*/  },
    {    64000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*256*/      0x0004,   /*122.88*/  },
    {    88200,       11289600,       0x0003,        /*2822.4*/     0x0027,     0x0004,     /*256*/      0x0004,   /*112.896*/ },
    {    96000,       12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*256*/      0x0004,   /*122.88*/  },
    {    128000,      12288000,       0x0003,        /*3072*/       0x0027,     0x0004,     /*128*/      0x0004,   /*122.88*/  },
    {    176400,      22579200,       0x0007,        /*2822.4*/     0x0027,     0x0004,     /*128*/      0x0004,   /*112.896*/ },
    {    192000,      24576000,       0x0007,        /*3072*/       0x0027,     0x0004,     /*128*/      0x0004,   /*122.88*/  },
};

ak4961_fs_bitclock_map_t ak4961_fs2bitclock_map[] =
{
/* 32*fs */
/*       fs            bclk           Ref div        Pll Ref Clk    fb div      mdiv2      codec mclk    mdivd      dsp mclk     */
    {    8000,        512000,         0x0000,        /*512*/        0x01DF,     0x000E,     /*512*/      0x0077,   /*122.88*/  },
    {    11025,       705600,         0x0000,        /*705.6*/      0x013F,     0x000E,     /*512*/      0x0077,   /*122.896*/ },
    {    16000,       1024000,        0x0000,        /*1024*/       0x00EF,     0x000E,     /*512*/      0x003B,   /*122.88*/  },
    {    22050,       1411200,        0x0000,        /*1411.2*/     0x009F,     0x000E,     /*512*/      0x003B,   /*112.896*/ },
    {    24000,       1536000,        0x0000,        /*1536*/       0x009F,     0x000E,     /*512*/      0x003B,   /*122.88*/  },
    {    32000,       2048000,        0x0000,        /*2048*/       0x0077,     0x000E,     /*512*/      0x001D,   /*122.88*/  },
    {    44100,       2822400,        0x0000,        /*2822.4*/     0x004F,     0x0009,     /*512*/      0x0013,   /*112.896*/ },
    {    48000,       3072000,        0x0000,        /*3072*/       0x004F,     0x0009,     /*512*/      0x0013,   /*122.88*/  },
};

static const ak4961_fs2cm_map_t ak4961_fs_cm_dsmlp_map[] =
{
    {   8000,  12000,  512, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_LOW_SPEED, AK4961_CM_SEL_CM_512FS },
    {  16000,  48000,  512, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_512FS },
    {  64000,  96000,  256, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_256FS },
    { 128000, 192000,  128, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_128FS },
};

static const ak4961_fs2cm_map_t ak4961_fs_cm_dsmlp_alternative_map[] =
{
    {   8000,  12000,  512, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_LOW_SPEED, AK4961_CM_SEL_CM_512FS },
    {  16000,  24000,  256, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_LOW_SPEED, AK4961_CM_SEL_CM_256FS },
    {  32000,  96000,  256, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_256FS },
    { 128000, 192000,  128, AK4961_MC_DSMLP1_STD_SPEED | AK4961_MC_DSMLP2_STD_SPEED, AK4961_CM_SEL_CM_128FS },
};

const ak4961_sync_domain_t ak4961_sync_domain_sdaif1 =
{
    .reg    = AK4961_REG_AIN12_SYNC_DOMAIN_SET,
    .shift  = AK4961_AIN12_SYNC_SET_SDAIF1_SHIFT,
    .mask   = AK4961_AIN12_SYNC_SET_SDAIF1_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sdaif2 =
{
    .reg    = AK4961_REG_AIN12_SYNC_DOMAIN_SET,
    .shift  = AK4961_AIN12_SYNC_SET_SDAIF2_SHIFT,
    .mask   = AK4961_AIN12_SYNC_SET_SDAIF2_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sdaif3 =
{
    .reg    = AK4961_REG_AIN34_SYNC_DOMAIN_SET,
    .shift  = AK4961_AIN34_SYNC_SET_SDAIF3_SHIFT,
    .mask   = AK4961_AIN34_SYNC_SET_SDAIF3_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sdaif4 =
{
    .reg    = AK4961_REG_AIN34_SYNC_DOMAIN_SET,
    .shift  = AK4961_AIN34_SYNC_SET_SDAIF4_SHIFT,
    .mask   = AK4961_AIN34_SYNC_SET_SDAIF4_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sdcdc =
{
    .reg    = AK4961_REG_ADC_SYNC_DOMAIN_SET,
    .shift  = AK4961_ADC_SYNC_SET_SDCDC_SHIFT,
    .mask   = AK4961_ADC_SYNC_SET_SDCDC_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sddsp =
{
    .reg    = AK4961_REG_SB3_DSPO_SYNC_DOMAIN_SET,
    .shift  = AK4961_DSPO_SYNC_SET_SDDSP_SHIFT,
    .mask   = AK4961_DSPO_SYNC_SET_SDDSP_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sddspo2 =
{
    .reg    = AK4961_REG_DSPO24_SYNC_DOMAIN_SET,
    .shift  = AK4961_DSPO2_SYNC_SET_SDDSPO2_SHIFT,
    .mask   = AK4961_DSPO2_SYNC_SET_SDDSPO2_MASK,
};

const ak4961_sync_domain_t ak4961_sync_domain_sddspo4 =
{
    .reg    = AK4961_REG_DSPO24_SYNC_DOMAIN_SET,
    .shift  = AK4961_DSPO4_SYNC_SET_SDDSPO4_SHIFT,
    .mask   = AK4961_DSPO4_SYNC_SET_SDDSPO4_MASK,
};

const ak4961_source_port_t ak4961_source_port_sdti1a =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI1A,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_source_port_t ak4961_source_port_sdti1b =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI1B,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_source_port_t ak4961_source_port_sdti1c =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI1C,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_source_port_t ak4961_source_port_sdti1d =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI1D,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_source_port_t ak4961_source_port_sdti2 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI2,
    .sync_domain            = &ak4961_sync_domain_sdaif2,
};

const ak4961_source_port_t ak4961_source_port_sdti3 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI3,
    .sync_domain            = &ak4961_sync_domain_sdaif3,
};

const ak4961_source_port_t ak4961_source_port_sdti4 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_SDTI4,
    .sync_domain            = &ak4961_sync_domain_sdaif4,
};

#if 0
const ak4961_source_port_t ak4961_source_port_adc1 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_ADC1,
    .sync_domain            = &ak4961_sync_domain_sdcdc,
};

const ak4961_source_port_t ak4961_source_port_adc2 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_ADC1,
    .sync_domain            = &ak4961_sync_domain_sdcdc,
};
#endif

const ak4961_source_port_t ak4961_source_port_dspo1 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_DSPO1,
    .sync_domain            = &ak4961_sync_domain_sddsp,
};

const ak4961_source_port_t ak4961_source_port_dspo2 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_DSPO2,
    .sync_domain            = &ak4961_sync_domain_sddspo2,
};

const ak4961_source_port_t ak4961_source_port_dspo3 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_DSPO3,
    .sync_domain            = &ak4961_sync_domain_sddsp,
};

const ak4961_source_port_t ak4961_source_port_dspo4 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_DSPO4,
    .sync_domain            = &ak4961_sync_domain_sddspo4,
};

const ak4961_source_port_t ak4961_source_port_dspo5 =
{
    .source_address_select  = AK4961_SOURCE_ADDRESS_SELECT_DSPO5,
    .sync_domain            = &ak4961_sync_domain_sddsp,
};

const ak4961_sink_port_t ak4961_sink_port_sdto1a =
{
    .reg_source_address     = AK4961_REG_SDTO1A_SRC_SEL,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_sink_port_t ak4961_sink_port_sdto1b =
{
    .reg_source_address     = AK4961_REG_SDTO1B_SRC_SEL,
    .sync_domain            = &ak4961_sync_domain_sdaif1,
};

const ak4961_sink_port_t ak4961_sink_port_sdto2 =
{
    .reg_source_address     = AK4961_REG_SDTO2_SRC_SEL,
    .sync_domain            = &ak4961_sync_domain_sdaif2,
};

const ak4961_sink_port_t ak4961_sink_port_sdto3 =
{
    .reg_source_address     = AK4961_REG_SDTO3_SRC_SEL,
    .sync_domain            = &ak4961_sync_domain_sdaif3,
};

const ak4961_sink_port_t ak4961_sink_port_sdto4 =
{
    .reg_source_address     = AK4961_REG_SDTO4_SRC_SEL,
    .sync_domain            = &ak4961_sync_domain_sdaif4,
};

const ak4961_sink_port_t ak4961_sink_port_dac1 =
{
    .reg_source_address     = AK4961_REG_DAC1_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dac2 =
{
    .reg_source_address     = AK4961_REG_DAC2_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dspi1 =
{
    .reg_source_address     = AK4961_REG_DSPI1_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dspi2 =
{
    .reg_source_address     = AK4961_REG_DSPI2_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dspi3 =
{
    .reg_source_address     = AK4961_REG_DSPI3_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dspi4 =
{
    .reg_source_address     = AK4961_REG_DSPI4_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const ak4961_sink_port_t ak4961_sink_port_dspi5 =
{
    .reg_source_address     = AK4961_REG_DSPI5_SRC_SEL,
    .sync_domain            = NULL /* auto */,
};

const static ak4961_dbl_range_t ak4961_dbl_range_ivl =
{
    .min    = AK4961_IVL_GAIN_IN_DB_MIN,
    .max    = AK4961_IVL_GAIN_IN_DB_MAX,
    .step   = AK4961_IVL_GAIN_IN_DB_STEP,
    .inv    = WICED_FALSE,
};

#if NOTYET
const static ak4961_dbl_range_t ak4961_dbl_range_dvl =
{
    .min    = AK4961_DVL_GAIN_IN_DB_MIN,
    .max    = AK4961_DVL_GAIN_IN_DB_MAX,
    .step   = AK4961_DVL_GAIN_IN_DB_STEP,
    .inv    = WICED_TRUE,
};
#endif

const static ak4961_dbl_range_t ak4961_dbl_range_hpg =
{
    .min    = AK4961_HP_GAIN_IN_DB_MIN,
    .max    = AK4961_HP_GAIN_IN_DB_MAX,
    .step   = AK4961_HP_GAIN_IN_DB_STEP,
    .inv    = WICED_FALSE,
};

const static ak4961_dbl_range_t ak4961_dbl_range_lout =
{
    .min    = AK4961_LOUT_GAIN_IN_DB_MIN,
    .max    = AK4961_LOUT_GAIN_IN_DB_MAX,
    .step   = AK4961_LOUT_GAIN_IN_DB_STEP,
    .inv    = WICED_FALSE,
};

static const ak4961_amp_t ak4961_amp_hp =
{
    .is_differential            = 0,
    .reg_pmamp                  = AK4961_REG_PM9,
    .pmamp_left_mask            = AK4961_PM9_PMHPL_MASK,
    .pmamp_right_mask           = AK4961_PM9_PMHPR_MASK,
    .reg_vol_ctrl               = AK4961_REG_HP_VOL_CTRL,
    .vol_ctrl_gain_mask         = AK4961_HP_VOL_CTRL_HPG_MASK,
    .vol_ctrl_gain_shift        = AK4961_HP_VOL_CTRL_HPG_SHIFT,
    .default_gain               = AK4961_HP_VOL_CTRL_HPG_0DB_DEFAULT,
    .gain_range                 = &ak4961_dbl_range_hpg,
    .pwr_up_time_in_millis      = AK4961_PM9_PMHP_WAIT_IN_MILLIS,
};

#ifdef AK4961_AMP_LOUT1
static const ak4961_amp_t ak4961_amp_lout1 =
{
    .is_differential            = 0,
    .reg_pmamp                  = AK4961_REG_PM9,
    .pmamp_left_mask            = AK4961_PM9_PMLO1L_MASK,
    .pmamp_right_mask           = AK4961_PM9_PMLO1R_MASK,
    .reg_vol_ctrl               = AK4961_REG_LOUT1_VOL_CTRL,
    .vol_ctrl_gain_mask         = AK4961_LOUT1_VOL_CTRL_LO1G_MASK,
    .vol_ctrl_gain_shift        = AK4961_LOUT1_VOL_CTRL_LO1G_SHIFT,
    .default_gain               = AK4961_LOUT1_VOL_CTRL_LO1G_0DB_DEFAULT,
    .gain_range                 = &ak4961_dbl_range_lout,
    .pwr_up_time_in_millis      = AK4961_PM9_PMLO1_WAIT_IN_MILLIS,
};
#endif

static const ak4961_amp_t ak4961_amp_lout2 =
{
    .is_differential            = 0,
    .reg_pmamp                  = AK4961_REG_PM10,
    .pmamp_left_mask            = AK4961_PM10_PMLO2LP_MASK,
    .pmamp_right_mask           = AK4961_PM10_PMLO2RP_MASK,
    .reg_vol_ctrl               = AK4961_REG_LOUT2_VOL_CTRL,
    .vol_ctrl_gain_mask         = AK4961_LOUT2_VOL_CTRL_LO2G_MASK,
    .vol_ctrl_gain_shift        = AK4961_LOUT2_VOL_CTRL_LO2G_SHIFT,
    .default_gain               = AK4961_LOUT2_VOL_CTRL_LO2G_0DB_DEFAULT,
    .gain_range                 = &ak4961_dbl_range_lout,
    .pwr_up_time_in_millis      = AK4961_PM10_PMLO2_WAIT_IN_MILLIS,
};

static const ak4961_amp_t ak4961_amp_lout2_diff =
{
    .is_differential            = 1,
    .reg_pmamp                  = AK4961_REG_PM10,
    .pmamp_left_mask            = AK4961_PM10_PMLO2LP_MASK | AK4961_PM10_PMLO2LN_MASK,
    .pmamp_right_mask           = AK4961_PM10_PMLO2RP_MASK | AK4961_PM10_PMLO2RN_MASK,
    .reg_vol_ctrl               = AK4961_REG_LOUT2_VOL_CTRL,
    .vol_ctrl_gain_mask         = AK4961_LOUT2_VOL_CTRL_LO2G_MASK,
    .vol_ctrl_gain_shift        = AK4961_LOUT2_VOL_CTRL_LO2G_SHIFT,
    .default_gain               = AK4961_LOUT2_VOL_CTRL_LO2G_0DB_DEFAULT,
    .gain_range                 = &ak4961_dbl_range_lout,
    .pwr_up_time_in_millis      = AK4961_PM10_PMLO2_WAIT_IN_MILLIS,
};

static const ak4961_dac_t ak4961_dac1 =
{
    .reg_dac_mono_mix           = AK4961_REG_DAC1_MONO_MIX,
    .reg_dac_src_sel            = AK4961_REG_DAC1_SRC_SEL,
    .reg_lch_out_vol            = AK4961_REG_LCH_OUT_VOL_1,
    .pm8_pmda_mask              = AK4961_PM8_PMDA1_MASK,
};

static const ak4961_dac_t ak4961_dac2 =
{
    .reg_dac_mono_mix           = AK4961_REG_DAC2_MONO_MIX,
    .reg_dac_src_sel            = AK4961_REG_DAC2_SRC_SEL,
    .reg_lch_out_vol            = AK4961_REG_LCH_OUT_VOL_2,
    .pm8_pmda_mask              = AK4961_PM8_PMDA2_MASK,
};

static const struct ak4961_adc ak4961_adc1 =
{
    .source_address_select      = AK4961_SOURCE_ADDRESS_SELECT_ADC1,
    .reg_mic_input_select       = AK4961_REG_MIC_INPUT_SEL_1,
    .reg_dmic                   = AK4961_REG_DMIC_1,
    .reg_mic_amp_lch_gain       = AK4961_REG_MIC_AMP_1_LCH_GAIN,
    .reg_mic_amp_rch_gain       = AK4961_REG_MIC_AMP_1_RCH_GAIN,
    .pm6_pmad_mask              = AK4961_PM6_PMAD1L_MASK | AK4961_PM6_PMAD1R_MASK,
    .pm6_pmad_shift             = AK4961_PM6_PMAD1L_SHIFT,
};

static const struct ak4961_adc ak4961_adc2 =
{
    .source_address_select      = AK4961_SOURCE_ADDRESS_SELECT_ADC2,
    .reg_mic_input_select       = AK4961_REG_MIC_INPUT_SEL_2,
    .reg_dmic                   = AK4961_REG_DMIC_2,
    .reg_mic_amp_lch_gain       = AK4961_REG_MIC_AMP_2_GAIN,
    .reg_mic_amp_rch_gain       = AK4961_REG_MIC_AMP_3_GAIN,
    .pm6_pmad_mask              = AK4961_PM6_PMAD2M_MASK | AK4961_PM6_PMAD2V_MASK,
    .pm6_pmad_shift             = AK4961_PM6_PMAD2M_SHIFT,
};

const ak4961_dsp_t ak4961_dsp1 =
{
    .reg_dsp_src_sel            = AK4961_REG_DSPI1_SRC_SEL,
};

const ak4961_dac_route_t ak4961_dac1_hp_route =
{
    .dac                        = &ak4961_dac1,
    .amp                        = &ak4961_amp_hp,
};

const ak4961_dac_route_t ak4961_dac2_lout2_route =
{
    .dac                        = &ak4961_dac2,
    .amp                        = &ak4961_amp_lout2,
};

const ak4961_dac_route_t ak4961_dac2_lout2_diff_route =
{
    .dac                        = &ak4961_dac2,
    .amp                        = &ak4961_amp_lout2_diff,
};

const ak4961_adc_route_t ak4961_adc1_mic_route =
{
    .is_dmic                    = 0,
    .adc                        = &ak4961_adc1,
    .mic_amp_gain_range         = &ak4961_dbl_range_ivl,
};

const ak4961_adc_route_t ak4961_adc1_dmic1_route =
{
    .is_dmic                    = 1,
    .adc                        = &ak4961_adc1,
    .mic_amp_gain_range         = &ak4961_dbl_range_ivl,
};

const ak4961_adc_route_t ak4961_adc2_dmic2_route =
{
    .is_dmic                    = 1,
    .adc                        = &ak4961_adc2,
    .mic_amp_gain_range         = &ak4961_dbl_range_ivl,
};


/******************************************************
 *              Function Declarations
 ******************************************************/

extern wiced_result_t ak4961_reg_init( ak4961_device_cmn_data_t *ak4961 );
extern wiced_result_t ak4961_reg_reset( ak4961_device_cmn_data_t *ak4961 );
extern wiced_result_t ak4961_reg_read( ak4961_device_cmn_data_t *ak4961, uint16_t register_address, uint8_t *reg_data );
extern wiced_result_t ak4961_reg_write( ak4961_device_cmn_data_t *ak4961, uint16_t register_address, uint8_t reg_data );
extern wiced_result_t ak4961_ram_write( ak4961_device_cmn_data_t *ak4961, const uint8_t *data, uint32_t data_length );

static wiced_result_t ak4961_set_effect(void *driver_data, uint8_t mode);

/******************************************************
 *               Function Definitions
 ******************************************************/


static wiced_result_t ak4961_upd_bits(ak4961_device_cmn_data_t *ak4961, uint16_t reg, uint8_t mask, uint8_t val)
{
    uint8_t old, new;

    AK4961_VERIFY( ak4961_reg_read( ak4961, reg, &old ) );

    new = ( old & ~mask ) | val;

    /*
     * At this point, we're not reading from the I2C bus,
     * we're only reading CACHED values that may or may not reflect
     * actual values in the chip registers
     * So, we'll write the updated value even it's supposedly the same as the cached value
     */
#ifdef ACTUAL_I2C_REGISTER_READ
    if ( new != old )
    {
        return ak4961_reg_write( ak4961, reg, new );
    }

    return WICED_SUCCESS;
#else
    return ak4961_reg_write( ak4961, reg, new );
#endif
}


/* Convert an ascending or descending range to a 0-indexed integer range. */
static uint8_t ak4961_double_range_to_index( ak4961_device_cmn_data_t *ak4961, double val, const ak4961_dbl_range_t *r )
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

static wiced_result_t ak4961_pram_write(ak4961_device_cmn_data_t *ak4961, const ak4961_dsp_ram_resource_t *pram_res, uint8_t pram_tab_size)
{
    ak4961_device_runtime_data_t    *rtd            = ak4961->rtd;
    uint32_t                        sample_rate     = rtd->sample_rate;
    const uint8_t                   *pram_buf;
    uint32_t                        pram_res_size;
    uint8_t                         i;

    for( i = 0; i < pram_tab_size; i++)
    {
        if(sample_rate == pram_res[i].sample_rate)
            break;
    }

    if ( i == pram_tab_size )
    {
        /* Requested sample rate cannot be supported */
        return WICED_UNSUPPORTED;
    }

    AK4961_VERIFY( ak4961_get_dsp_ram_resource( pram_res[i].ram_res, &pram_buf, &pram_res_size ) );

    /* Need to subtract AK4961_CRC_RESULT_BYTES */
    pram_res_size = (pram_res_size - AK4961_CRC_RESULT_BYTES);

    AK4961_VERIFY( ak4961_ram_write( ak4961, pram_buf, pram_res_size) );

    AK4961_VERIFY( ak4961_free_dsp_ram_resource( pram_res[i].ram_res, pram_buf ) );

    return WICED_SUCCESS;
}

static wiced_result_t ak4961_cram_write(ak4961_device_cmn_data_t *ak4961, const ak4961_dsp_ram_resource_t *cram_res, uint8_t cram_tab_size)
{
    ak4961_device_runtime_data_t    *rtd            = ak4961->rtd;
    uint32_t                        sample_rate     = rtd->sample_rate;
    const uint8_t                   *cram_buf;
    uint32_t                        cram_res_size;
    uint8_t                         i;

    for( i = 0; i < cram_tab_size; i++)
    {
        if(sample_rate == cram_res[i].sample_rate)
            break;
    }

    if ( i == cram_tab_size )
    {
        /* Requested sample rate cannot be supported */
        return WICED_UNSUPPORTED;
    }

    AK4961_VERIFY( ak4961_get_dsp_ram_resource( cram_res[i].ram_res, &cram_buf, &cram_res_size ) );

    /* Need to subtract AK4961_CRC_RESULT_BYTES */
    cram_res_size = (cram_res_size - AK4961_CRC_RESULT_BYTES);

    AK4961_VERIFY( ak4961_ram_write( ak4961, cram_buf, cram_res_size) );

    AK4961_VERIFY( ak4961_free_dsp_ram_resource( cram_res[i].ram_res, cram_buf ) );

    return WICED_SUCCESS;
}

static wiced_result_t ak4961_ram_download(ak4961_device_cmn_data_t *ak4961, const ak4961_dsp_ram_resource_t *pram_res, uint8_t pram_tab_size, const ak4961_dsp_ram_resource_t *cram_res, uint8_t cram_tab_size)
{
    /* PMSW = SWRSTN bits = 0 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_PMSW_MASK, 0 ) );
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_SWRSTN_MASK, 0 ) );
    /* PMSW = SWRSTN bits = 1 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_PMSW_MASK, AK4961_FC1_PMSW ) );
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_SWRSTN_MASK, AK4961_FC1_SWRSTN ) );

    /* DSP setting register1&2 sould be initialized to zero after DSP Power-up */
    /* DRAMS = DRAD = BANK bits = 0 */
    AK4961_VERIFY( ak4961_reg_write( ak4961, AK4961_REG_DSP_SETTING1, 0 ) );
    /* POMODE = WAVP bits = 0 */
    AK4961_VERIFY( ak4961_reg_write( ak4961, AK4961_REG_DSP_SETTING2, 0 ) );

    /* DSPRSTN bit = 0 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC3, AK4961_FC3_DSPRSTN_MASK, 0 ) );
    /* DLRDY bit = 1 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC2, AK4961_FC2_DLRDY_MASK, AK4961_FC2_DLRDY ) );
    /* PRIF bit = 1 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PRAM_READY, AK4961_PRAM_READY_MASK, AK4961_PRAM_READY_PRIF ) );

    /* PRAM Download*/
    AK4961_VERIFY( ak4961_pram_write( ak4961, pram_res, pram_tab_size) );

    /* PRIF bit = 0 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PRAM_READY, AK4961_PRAM_READY_MASK, 0 ) );

    /* CRAM Download */
    AK4961_VERIFY( ak4961_cram_write( ak4961, cram_res, cram_tab_size) );

    /* DLRDY bit = 0 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC2, AK4961_FC2_DLRDY_MASK, 0 ) );

    /* DSPRSTN bit = 1 */
    AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC3, AK4961_FC3_DSPRSTN_MASK, AK4961_FC3_DSPRSTN ) );

    return WICED_SUCCESS;
}

wiced_result_t ak4961_chip_reset(ak4961_device_cmn_data_t *ak4961)
{
    if (ak4961->pdn != WICED_GPIO_NONE)
    {
        /* Pulse PDN line. */
        AK4961_VERIFY( wiced_gpio_output_low( ak4961->pdn ) );
        AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_RESET_HOLD_TIME_IN_MILLIS ) );
        AK4961_VERIFY( wiced_gpio_output_high( ak4961->pdn ) );
        AK4961_VERIFY( wiced_rtos_delay_milliseconds( 1 ) );
    }

    return WICED_SUCCESS;
}


wiced_result_t ak4961_init(void *driver_data, wiced_audio_data_port_t* data_port)
{
    ak4961_device_data_t            *dd                = (ak4961_device_data_t *) driver_data;
    ak4961_device_cmn_data_t        *ak4961            = dd->cmn;
    ak4961_device_runtime_data_t    *rtd               = ak4961->rtd;
    const ak4961_route_data_t       *route_data        = (const ak4961_route_data_t *) dd->route;
    ak4961_sync_domain_select_t     sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1;
    const ak4961_route_id_t         route_id           = route_data->id;
    wiced_result_t                  result             = WICED_SUCCESS;

    LOCK_RTD(rtd);

    if ( ( rtd->init & ( 1 << route_id ) ) != 0 )
    {
        /* This route is already initialized! */
        wiced_assert("already initialized", !(rtd->init & 1 << route_id));
        result = WICED_ERROR;
    }
    else if ( rtd->init != 0 )
    {
        /* Initialization is only required once per device. */
        WPRINT_LIB_INFO(("ak4961 reusing first initialization\n"));
        goto already_initialized;
    }

    if ( route_data->device_type == AK4961_DEVICE_TYPE_PLAYBACK )
    {
        sync_domain_select = ((ak4961_dac_route_data_t *)route_data)->sync_domain_select;
    }
    else if ( route_data->device_type == AK4961_DEVICE_TYPE_CAPTURE )
    {
        sync_domain_select = ((ak4961_adc_route_data_t *)route_data)->sync_domain_select;
    }

    /* Initialize GPIOs. */
    if ( ak4961->pdn != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_init( ak4961->pdn, OUTPUT_PUSH_PULL ), result, ak4961_init_unlock );
    }
    if ( ak4961->switcher_3v3_ps_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_init( ak4961->switcher_3v3_ps_enable, OUTPUT_PUSH_PULL ), result, ak4961_init_unlock );
    }
    if ( ak4961->switcher_2v_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_init( ak4961->switcher_2v_enable, OUTPUT_PUSH_PULL ), result, ak4961_init_unlock );
    }
    if ( ak4961->ldo_1v8_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_init( ak4961->ldo_1v8_enable, OUTPUT_PUSH_PULL ), result, ak4961_init_unlock );
    }

    /* Power chip. */
    if ( ak4961->switcher_3v3_ps_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_output_low( ak4961->switcher_3v3_ps_enable ), result, ak4961_init_unlock );
    }
    if ( ak4961->switcher_2v_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_output_high( ak4961->switcher_2v_enable ), result, ak4961_init_unlock );
    }
    if ( ak4961->ldo_1v8_enable != WICED_GPIO_NONE )
    {
        AK4961_VERIFY_GOTO( wiced_gpio_output_high( ak4961->ldo_1v8_enable ), result, ak4961_init_unlock );
    }

    /* Enable register access. */
    AK4961_VERIFY_GOTO( ak4961_reg_init( ak4961 ), result, ak4961_init_unlock );

    /* Reset chip/registers to defaults. */
    AK4961_VERIFY_GOTO( ak4961_reg_reset( ak4961 ), result, ak4961_init_unlock );

#ifdef POWER_PLL_EARLY
    if ( ak4961->ck.pll_enab )
    {
        /* Enable PLL. */
        AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, AK4961_REG_PM1, AK4961_PM1_PMPLL1 ), result, ak4961_init_unlock );
    }
#endif
    if ( ak4961->ck.is_frame_master )
    {
        if ( sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_1 )
        {
            AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, AK4961_REG_MCLK1_SRC_SEL, AK4961_MCLK_SRC_SEL_MSN ), result, ak4961_init_unlock );
        }
        else if ( sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_2 )
        {
            AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, AK4961_REG_MCLK2_SRC_SEL, AK4961_MCLK_SRC_SEL_MSN ), result, ak4961_init_unlock );
        }

        else if ( sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_3 )
        {
            AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, AK4961_REG_MCLK3_SRC_SEL, AK4961_MCLK_SRC_SEL_MSN ), result, ak4961_init_unlock );
        }
        else
        {
            wiced_assert("sync domain is not supported", 0==1);
            result = WICED_ERROR;
        }
    }

    /* Power-up serial audio bus. */
    AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, AK4961_REG_PM2, AK4961_PM2_PMAIF ), result, ak4961_init_unlock );
    /* EXIF */
    //AK4961_VERIFY_GOTO( ak4961_reg_write( ak4961, 0x008F, 0x80 ), result, ak4961_init_unlock );

already_initialized:
    /* Mark device as initialized. */
    rtd->init |= 1 << route_id;

    data_port->type = PLATFORM_AUDIO_LINE;
    data_port->port = dd->data_port;
    switch (route_data->device_type)
    {
        case AK4961_DEVICE_TYPE_PLAYBACK:
            data_port->channel = WICED_PLAY_CHANNEL;
            break;

        case AK4961_DEVICE_TYPE_CAPTURE:
            data_port->channel = WICED_RECORD_CHANNEL;
            break;

        default:
            result = WICED_ERROR;
            break;
    }

ak4961_init_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


wiced_result_t ak4961_deinit(void *driver_data)
{
    ak4961_device_data_t            *dd         = ( ak4961_device_data_t* ) driver_data;
    ak4961_device_cmn_data_t        *ak4961     = dd->cmn;
    ak4961_device_runtime_data_t    *rtd        = ak4961->rtd;
    const ak4961_route_data_t       *route_data = ( const ak4961_route_data_t *) dd->route;
    const ak4961_route_id_t         route_id    = route_data->id;
    ak4961_rfn_t * const            rfn         = route_data->device_route->fn;
    wiced_result_t                  result      = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Power-down audio route. */
    if ( rfn != NULL )
    {
        AK4961_VERIFY_GOTO( (*rfn)( dd, WICED_FALSE ), result, ak4961_deinit_unlock );
    }

    /* This audio route is no longer configured. */
    /* Remove in-use flags. */
    rtd->cfg  &= ~( 1 << route_id );
    rtd->init &= ~( 1 << route_id );

    /* No other initializations; clean-up. */
    if ( ( rtd->init & ~( 1 << route_id ) ) == 0 )
    {
        /* Power-down PLL. */
        if (ak4961->ck.pll_enab)
        {
            AK4961_VERIFY_GOTO( ak4961_upd_bits( ak4961, AK4961_REG_PM1, AK4961_PM1_PMPLL1, 0 ), result, ak4961_deinit_unlock );
        }

        /* Power-down audio bus. */
        AK4961_VERIFY_GOTO( ak4961_upd_bits( ak4961, AK4961_REG_PM2, AK4961_PM2_PMAIF, 0 ), result, ak4961_deinit_unlock );

        /* Power-down chip. */
        if (ak4961->pdn != WICED_GPIO_NONE)
        {
            AK4961_VERIFY_GOTO( wiced_gpio_output_low( ak4961->pdn ), result, ak4961_deinit_unlock );
        }
        if ( ak4961->ldo_1v8_enable != WICED_GPIO_NONE )
        {
            AK4961_VERIFY_GOTO( wiced_gpio_output_low( ak4961->ldo_1v8_enable ), result, ak4961_deinit_unlock );
        }
        if ( ak4961->switcher_2v_enable != WICED_GPIO_NONE )
        {
            AK4961_VERIFY_GOTO( wiced_gpio_output_low( ak4961->switcher_2v_enable ), result, ak4961_deinit_unlock );
        }
    }

    /* Don't deinitialize I2C since it might be used by other modules.
     * Diddo for IOE.
     */
ak4961_deinit_unlock:
    UNLOCK_RTD(rtd);

    return result;
}


/* PLL Slave Mode. */
wiced_result_t ak4961_spll(ak4961_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    ak4961_device_cmn_data_t    *ak4961            = dd->cmn;
    ak4961_route_data_t         *route_data_base   = (ak4961_route_data_t *)dd->route;
    ak4961_sync_domain_select_t sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1;
    size_t                      i;
    uint8_t                     clock_src_addr     = AK4961_CLK_SRC_ADDR_MCKI1;
    uint32_t                    mcki               = 0;
    uint32_t                    fs                 = 0;
    uint32_t                    pll_ref_clk        = 0;
    uint16_t                    pll1_ref_div       = 0;
    uint16_t                    pll1_fb_div        = 0;
    uint16_t                    mdiv2              = 0;
    uint16_t                    mdivd              = 0;
    uint8_t                     codec_cm           = 0;
    uint8_t                     codec_speed        = 0;

    if ( route_data_base->device_type == AK4961_DEVICE_TYPE_PLAYBACK )
    {
        sync_domain_select = ((ak4961_dac_route_data_t *)route_data_base)->sync_domain_select;
    }
    else if ( route_data_base->device_type == AK4961_DEVICE_TYPE_CAPTURE )
    {
        sync_domain_select = ((ak4961_adc_route_data_t *)route_data_base)->sync_domain_select;
    }

    if ( sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_1 )
    {
        for ( i = 0; i < ARRAYSIZE( ak4961_fs2mcki_map ); i++ )
        {
            /* Find a sample rate range that is supported. */
            if ( config->sample_rate == ak4961_fs2mcki_map[i].fs )
            {
                if ( mclk == ak4961_fs2mcki_map[i].mcki )
                {
                    break;
                }
            }
        }
        if ( i == ARRAYSIZE( ak4961_fs2mcki_map ) )
        {
            return WICED_UNSUPPORTED;
        }

        mcki           = ak4961_fs2mcki_map[i].mcki;
        fs             = ak4961_fs2mcki_map[i].fs;
        clock_src_addr = AK4961_CLK_SRC_ADDR_MCKI1;
        pll1_ref_div   = ak4961_fs2mcki_map[i].pll1_ref_div;
        pll1_fb_div    = ak4961_fs2mcki_map[i].pll1_fb_div;
        mdiv2          = ak4961_fs2mcki_map[i].mdiv2;
        mdivd          = ak4961_fs2mcki_map[i].mdivd;
        /* Set PLL1 ref clock divider MSByte and LSByte, respectively. */
        pll_ref_clk    = mcki / (pll1_ref_div + 1);

        /* Set codec master clock frequency. */
        for (i = 0; i < ARRAYSIZE( ak4961_fs_cm_dsmlp_map ); i++ )
        {
            /* Find a sample rate range that is supported. */
            if ( (config->sample_rate >= ak4961_fs_cm_dsmlp_map[i].fs1) &&
                 (config->sample_rate <= ak4961_fs_cm_dsmlp_map[i].fs2) )
            {
                break;
            }
        }
        if ( i == ARRAYSIZE( ak4961_fs_cm_dsmlp_map ) )
        {
            return WICED_UNSUPPORTED;
        }

        codec_cm    = ak4961_fs_cm_dsmlp_map[i].cm;
        codec_speed = ak4961_fs_cm_dsmlp_map[i].speed;
    }
    else if ( sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_2 ||
              sync_domain_select == AK4961_SYNC_DOMAIN_SELECT_3)
    {
        for ( i = 0; i < ARRAYSIZE( ak4961_fs2bitclock_map ); i++ )
        {
            /* Find a sample rate range that is supported. */
            if ( config->sample_rate == ak4961_fs2bitclock_map[i].fs )
            {
                break;
            }
        }
        if ( i == ARRAYSIZE( ak4961_fs2bitclock_map ) )
        {
            return WICED_UNSUPPORTED;
        }

        mcki           = 0;
        fs             = ak4961_fs2bitclock_map[i].fs;
        clock_src_addr = AK4961_CLK_SRC_ADDR_BCLK3;
        pll1_ref_div   = ak4961_fs2bitclock_map[i].pll1_ref_div;
        pll1_fb_div    = ak4961_fs2bitclock_map[i].pll1_fb_div;
        mdiv2          = ak4961_fs2bitclock_map[i].mdiv2;
        mdivd          = ak4961_fs2bitclock_map[i].mdivd;

        /* Set PLL1 ref clock divider MSByte and LSByte, respectively. */
        pll_ref_clk = ak4961_fs2bitclock_map[i].bitclock / (pll1_ref_div + 1);

        /* Set codec master clock frequency. */
        for (i = 0; i < ARRAYSIZE( ak4961_fs_cm_dsmlp_alternative_map ); i++ )
        {
            /* Find a sample rate range that is supported. */
            if ( (config->sample_rate >= ak4961_fs_cm_dsmlp_alternative_map[i].fs1) &&
                 (config->sample_rate <= ak4961_fs_cm_dsmlp_alternative_map[i].fs2) )
            {
                break;
            }
        }
        if ( i == ARRAYSIZE( ak4961_fs_cm_dsmlp_alternative_map ) )
        {
            return WICED_UNSUPPORTED;
        }

        codec_cm    = ak4961_fs_cm_dsmlp_alternative_map[i].cm;
        codec_speed = ak4961_fs_cm_dsmlp_alternative_map[i].speed;
    }
    else
    {
        wiced_assert("ak4961_spll: sync domain is not supported", 0==1);
        return WICED_UNSUPPORTED;
    }

    /* MCKI->PLLCLK1 (SYNC DOMAIN 1) OR BCLK3->PLLCLK1 (SYNC DOMAIN 3) */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PLL1_CLK_SRC_SEL, AK4961_PLL1_CLK_SRC_SEL_PLS1_MASK, clock_src_addr) );

    if ( (pll_ref_clk >= 256000) && (pll_ref_clk <= 3072000) )
    {
        AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PLL1_REF_CLK_DIV1, AK4961_PLL1_CLK_DIV_PL1_MASK, (uint8_t) (pll1_ref_div >> 8) ) );
        AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PLL1_REF_CLK_DIV2, AK4961_PLL1_CLK_DIV_PL1_MASK, (uint8_t) (pll1_ref_div) ) );
    }
    else
    {
        wiced_assert( "pll ref clock not in range", pll_ref_clk >= 256000 && pll_ref_clk <= 3072000);
        return WICED_UNSUPPORTED;
    }

    /* Set PLL1 feedback clock divider MSByte and LSByte, respectively. */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PLL1_FB_CLK_DIV1, AK4961_PLL1_CLK_DIV_PL1_MASK, (uint8_t) (pll1_fb_div >> 8) ) );
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PLL1_FB_CLK_DIV2, AK4961_PLL1_CLK_DIV_PL1_MASK, (uint8_t) (pll1_fb_div) ) );

    /* PLLCLK1->CODECCLK. */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_CODEC_CLK_SRC_SEL, AK4961_CODEC_CLK_SRC_SEL_MCKS2_MASK, AK4961_CLK_SRC_ADDR_PLLCLK1) );
    /* Set codec clock divider */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_CODEC_CLK_DIV, AK4961_CODEC_CLK_DIV_MDIV2_MASK, mdiv2 ) );

    /* PLLCLK1->DSPCLK. */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_DSP_MCLK_SRC_SEL, AK4961_DSP_MCLK_SRC_SEL_CKSD_MASK, AK4961_CLK_SRC_ADDR_PLLCLK1) );
    if ( ( mcki == 0 ) || ( ( mcki >= (fs * 128) ) && ( mcki <= 24576000 ) ) )
    {
        AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_BUS_CLK_DIV, AK4961_BUS_CLK_DIV_MDIVD_MASK, mdivd ) );
    }
    else
    {
        wiced_assert( "busclk frequency not in range", 0==1 );
        return WICED_UNSUPPORTED;
    }

    /*
     * Select the codec's clock mode (CM bits).
     */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_CM_SEL, AK4961_CM_SEL_CM_MASK, codec_cm) );
    /*
     * Select DAC1/2 speed mode (DSMLP bits).
     * Do this for both DACs regardless of direction selected and DAC selection, since fs is locked-in
     * at this point.
     */
    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_MC, AK4961_MC_DSMLP1_MASK | AK4961_MC_DSMLP2_MASK, codec_speed) );

#ifndef POWER_PLL_EARLY
    if ( ak4961->ck.pll_enab )
    {
        /* Enable PLL. */
        AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PM1, AK4961_PM1_PMPLL1, AK4961_PM1_PMPLL1 ) );
    }
#endif

    return WICED_SUCCESS;
}


static wiced_result_t ak4961_clock(ak4961_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk)
{
    ak4961_device_cmn_data_t    *ak4961            = dd->cmn;
    ak4961_route_data_t         *route_data_base   = (ak4961_route_data_t *)dd->route;
    ak4961_sync_domain_select_t sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1;
    size_t                      i;
    uint8_t                     dlc1;
    uint16_t                    audio_iface_reg    = 0;

    if ( route_data_base->device_type == AK4961_DEVICE_TYPE_PLAYBACK )
    {
        sync_domain_select = ((ak4961_dac_route_data_t *)route_data_base)->sync_domain_select;
    }
    else if ( route_data_base->device_type == AK4961_DEVICE_TYPE_CAPTURE )
    {
        sync_domain_select = ((ak4961_adc_route_data_t *)route_data_base)->sync_domain_select;
    }

    /* Sample frequency. */
    for ( i = 0; i < ARRAYSIZE( fs_map ); i++ )
    {
        if ( config->sample_rate == fs_map[i].fs )
        {
            AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_CM_SEL, AK4961_CM_SEL_FS_MASK, fs_map[i].fsbits) );
            break;
        }
    }

    if ( i == ARRAYSIZE( fs_map ) )
    {
        /* Requested sample rate cannot be supported */
        return WICED_UNSUPPORTED;
    }

    /* Set Sampling depth */
    switch (config->bits_per_sample)
    {
        case 16:
            dlc1 = AK4961_SDTIO1_IF_FMT_DLC1_16BIT;
            break;
        case 24:
            dlc1 = AK4961_SDTIO1_IF_FMT_DLC1_24BIT;
            break;
        case 32:
            dlc1 = AK4961_SDTIO1_IF_FMT_DLC1_32BIT;
            break;
        default:
            return WICED_UNSUPPORTED;
    }

    AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_CODEC_IF_FORMAT, AK4961_SDTIO1_IF_FMT_DLC1_MASK, dlc1) );

    switch (sync_domain_select)
    {
        case AK4961_SYNC_DOMAIN_SELECT_1:
            audio_iface_reg = 0;
            break;
        case AK4961_SYNC_DOMAIN_SELECT_2:
            audio_iface_reg = AK4961_REG_SDTIO2_IF_FORMAT;
            break;
        case AK4961_SYNC_DOMAIN_SELECT_3:
            audio_iface_reg = AK4961_REG_SDTIO3_IF_FORMAT;
            break;
        default:
            return WICED_UNSUPPORTED;
    }

    if ( audio_iface_reg != 0 )
    {
        AK4961_VERIFY( ak4961_upd_bits(ak4961, audio_iface_reg, AK4961_SDTIO1_IF_FMT_DLC1_MASK, dlc1) );
    }

    /* Clock configuration. */
    if ( ak4961->ck.fn != NULL)
    {
        AK4961_VERIFY( (*ak4961->ck.fn)(dd, config, mclk) );
    }

    return WICED_SUCCESS;
}


static inline wiced_bool_t ak4961_clock_config_eq(ak4961_device_runtime_data_t *rtd, const wiced_audio_config_t *config)
{
    if ( config->bits_per_sample == rtd->bits_per_sample &&
         config->channels        == rtd->channels        &&
         config->sample_rate     == rtd->sample_rate )
    {
        return WICED_TRUE;
    }
    return WICED_FALSE;
}


static inline wiced_result_t ak4961_set_clock_config(ak4961_device_runtime_data_t *rtd, const wiced_audio_config_t *config)
{
    rtd->bits_per_sample = config->bits_per_sample;
    rtd->channels = config->channels;
    rtd->sample_rate = config->sample_rate;

    return WICED_SUCCESS;
}


wiced_result_t ak4961_configure( void *driver_data, wiced_audio_config_t *config, uint32_t* mclk)
{
    ak4961_device_data_t            *dd         = ( ak4961_device_data_t *) driver_data;
    ak4961_device_cmn_data_t        *ak4961     = dd->cmn;
    ak4961_device_runtime_data_t    *rtd        = ak4961->rtd;
    const ak4961_route_data_t       *route_data = (const ak4961_route_data_t *) dd->route;
    const ak4961_route_id_t         route_id    = route_data->id;
    ak4961_rfn_t * const            rfn         = route_data->device_route->fn;
    wiced_result_t                  result      = WICED_SUCCESS;

    LOCK_RTD(rtd);

    /* Must be initialized first! */
    if ( ( rtd->init & ( 1 << route_id ) ) == 0 )
    {
        result = WICED_ERROR;
        goto ak4961_configure_unlock;
    }

    /*
     * Configure clocking.
     * Shared between playback/capture, so configs need to match.
     */
    if ( ( rtd->cfg & ~( 1 << route_id ) ) != 0 )
    {
        /* Device configured in opposite direction.
         * Clocking arguments need to match to satisfy duplex requirements.
         */
        if ( ak4961_clock_config_eq(rtd, config) == WICED_FALSE )
        {
            result = WICED_UNSUPPORTED;
            goto ak4961_configure_unlock;
        }
    }
    else if ( ( rtd->cfg & ( 1 << route_id ) ) != 0 )
    {
        /* Re-configuring is not permitted. */
        result = WICED_ERROR;
        goto ak4961_configure_unlock;
    }
    else
    {
        /* First time. */
        /* Lock clock configuration. */
        AK4961_VERIFY_GOTO( ak4961_clock( dd, config, *mclk ), result, ak4961_configure_unlock );

        /* Configure audio route. */
        AK4961_VERIFY_GOTO( ak4961_set_clock_config( rtd, config ), result, ak4961_configure_unlock );
    }

    if ( rfn != NULL )
    {
        /* Power-up audio route. */
        AK4961_VERIFY_GOTO( (*rfn)( dd, WICED_TRUE ), result, ak4961_configure_unlock );
    }

    /* The direction is now configured. */
    rtd->cfg |= ( 1 << route_id );

ak4961_configure_unlock:
    UNLOCK_RTD(rtd);

    return WICED_SUCCESS;
}


wiced_result_t ak4961_dac_route(ak4961_device_data_t *driver_data, wiced_bool_t is_pwr_up)
{
    ak4961_device_cmn_data_t        *ak4961     = driver_data->cmn;
    ak4961_device_runtime_data_t    *rtd        = ak4961->rtd;
    ak4961_dac_route_data_t         *data       = (ak4961_dac_route_data_t *) driver_data->route;
    const ak4961_dac_route_t        *dac_route  = (const ak4961_dac_route_t *) data->base.device_route->instance;
    const ak4961_dac_t              *dac        = dac_route->dac;
    const ak4961_amp_t              *amp        = dac_route->amp;
    const ak4961_route_id_t         route_id    = data->base.id;

    if ( is_pwr_up == WICED_TRUE )
    {
        /*
         * Pre- Power-up configuration.
        */

        /* Configure source & sink, set sync domain */
        const ak4961_sync_domain_t *sync_domain = data->source_port->sync_domain;
        AK4961_VERIFY( ak4961_upd_bits( ak4961, sync_domain->reg, sync_domain->mask, data->sync_domain_select << sync_domain->shift) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_dac_src_sel, AK4961_SRC_SEL_MASK, data->source_port->source_address_select ) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_ADC_SYNC_DOMAIN_SET, AK4961_ADC_SYNC_SET_SDCDC_MASK, data->sync_domain_select) );

        /* Select Mono Mix. */
        uint8_t mix = ( (data->output_right_select << AK4961_DAC_MIX_RIGHT_SHIFT) | (data->output_left_select << AK4961_DAC_MIX_LEFT_SHIFT) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_dac_mono_mix, AK4961_DAC_MIX_RIGHT_MASK | AK4961_DAC_MIX_LEFT_MASK, mix ) );

        /* Reset AMP gain (volume) to supplied default. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, amp->reg_vol_ctrl, amp->vol_ctrl_gain_mask, data->amp_gain_mute ) );
        data->amp_gain_current = data->amp_gain_default;

        /* Reset Digital Volume to supplied default. */
        /* Left volume tied to right by default. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_lch_out_vol, AK4961_OUT_VOL_MASK, data->digital_volume ) );

        /*
         * Power-up sequence.
        */

        if ( rtd->pm3_3 == 0 )
        {
            /* CP3. */
            AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP3_MASK, AK4961_PM3_PMCP3_SET) );
            AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_PMCP3_WAIT_IN_MILLIS ) );

            /* LDO3P and LDO3N. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMLDO3N_MASK | AK4961_PM3_PMLDO3P_MASK, AK4961_PM3_PMLDO3N_SET | AK4961_PM3_PMLDO3P_SET ) );
            AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_PMLDO3NP_WAIT_IN_MILLIS ) );
        }
        rtd->pm3_3 |= (1U << route_id);

        /* DAC. */
        AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PM8, dac->pm8_pmda_mask, dac->pm8_pmda_mask ) );

        /* CP2. */
        if ( dac == &ak4961_dac1 )
        {
            /* Head-phone. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP2_MASK, AK4961_PM3_PMCP2_SET ) );
            AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_PMCP2_WAIT_IN_MILLIS ) );
        }

        /* Single-ended or differential output mode (Table 43). */
        if ( amp == &ak4961_amp_lout2 || amp == &ak4961_amp_lout2_diff )
        {
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_LOUT2, AK4961_LOUT2_LO2DIF_MASK, amp->is_differential ) );
        }

        /* AMP. */
        uint8_t pmamp_sel = 0;
        if ( data->output_right_select != AK4961_DAC_OUTPUT_SELECT_MUTE )
        {
            pmamp_sel |= amp->pmamp_left_mask;
        }
        if ( data->output_left_select != AK4961_DAC_OUTPUT_SELECT_MUTE )
        {
            pmamp_sel |= amp->pmamp_right_mask;
        }
        AK4961_VERIFY( ak4961_upd_bits( ak4961, amp->reg_pmamp, amp->pmamp_left_mask | amp->pmamp_right_mask, pmamp_sel ) );
        AK4961_VERIFY( wiced_rtos_delay_milliseconds( amp->pwr_up_time_in_millis ) );
    }
    else
    {
        /*
         * Power-down sequence.
        */

        /* Amp. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, amp->reg_pmamp, amp->pmamp_left_mask | amp->pmamp_right_mask, 0 ) );

        /* CP2. */
        if ( dac == &ak4961_dac1 )
        {
            /* Head-phone. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP2_MASK, 0 ) );
        }

        /* DAC. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM8, dac->pm8_pmda_mask, 0 ) );

        rtd->pm3_3 &= ~(1U << route_id);
        if ( rtd->pm3_3 == 0 )
        {
            /* LDO3P/N. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMLDO3N_MASK | AK4961_PM3_PMLDO3P_MASK, 0 ) );

            /* CP3. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP3_MASK, 0 ) );
        }

    }
    return WICED_SUCCESS;
}


static uint8_t ak4961_mic_pwr_lvl_shift( ak4961_mic_power_supply_t mic_power_supply )
{
    wiced_assert( "bad value", mic_power_supply != AK4961_MIC_POWER_SUPPLY_MAX );
    return mic_power_supply != AK4961_MIC_POWER_SUPPLY_MPWR2 ? AK4961_MIC_PWR_LVL_MICL1_SHIFT : AK4961_MIC_PWR_LVL_MICL2_SHIFT;
}

/*
 * AN1-6 -> MIC-Amp1/2/3 -> ADC1/2
 * Single-ended or full-differential.
 *
 */
wiced_result_t ak4961_adc_mic_route(ak4961_device_data_t *driver_data, wiced_bool_t is_pwr_up)
{
    ak4961_device_cmn_data_t        *ak4961         = driver_data->cmn;
    ak4961_device_runtime_data_t    *rtd            = ak4961->rtd;
    const ak4961_adc_route_data_t   *data           = (const ak4961_adc_route_data_t *) driver_data->route;
    const ak4961_adc_route_t        *adc_route      = (const ak4961_adc_route_t *) data->base.device_route->instance;
    const ak4961_adc_t              *adc            = adc_route->adc;
    const ak4961_adc_analog_t       *analog         = &data->type.analog;
    const ak4961_route_id_t         route_id        = data->id;
    uint8_t                         pm4_pmmp_mask   = 0;
    uint8_t                         pm5_pmain_mask  = 0;

    if ( analog->input_left != NULL )
    {
        if ( analog->input_left->power_supply != AK4961_MIC_POWER_SUPPLY_MAX )
        {
            pm4_pmmp_mask |= (1U << analog->input_left->power_supply);
        }
        if ( analog->input_left->adc_input_select != AK4961_ADC_INPUT_SELECT_MUTE )
        {
            pm5_pmain_mask |= (1U << analog->input_left->adc_input_select);
        }
    }
    if ( analog->input_right != NULL )
    {
        if ( analog->input_right->power_supply != AK4961_MIC_POWER_SUPPLY_MAX )
        {
            pm4_pmmp_mask |= (1U << analog->input_right->power_supply);
        }
        if ( analog->input_right->adc_input_select != AK4961_ADC_INPUT_SELECT_MUTE )
        {
            pm5_pmain_mask |= (1U << analog->input_right->adc_input_select);
        }
    }

    if ( is_pwr_up == WICED_TRUE )
    {
        /*
         * Pre- Power-up configuration.
        */

        uint8_t mic_input_select    = 0;
        uint8_t pm6_pmad_select     = 0;
        if ( analog->input_left != NULL )
        {
            mic_input_select |= (analog->input_left->adc_input_select << AK4961_MIC_INPUT_SEL_LEFT_SHIFT);
            pm6_pmad_select  |= (1U << adc->pm6_pmad_shift);
        }
        if ( analog->input_right != NULL)
        {
            mic_input_select |= (analog->input_right->adc_input_select << AK4961_MIC_INPUT_SEL_RIGHT_SHIFT);
            pm6_pmad_select  |= (2U << adc->pm6_pmad_shift);
        }

        /* Select source/sink and setup sync domain */
        const ak4961_sync_domain_t *sync_domain = data->sink_port->sync_domain;
        AK4961_VERIFY( ak4961_upd_bits( ak4961, data->sink_port->reg_source_address, AK4961_SRC_SEL_MASK, adc->source_address_select ) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, sync_domain->reg, sync_domain->mask, data->sync_domain_select << sync_domain->shift ) );
        /* XXX For both ADC1/2. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_ADC_SYNC_DOMAIN_SET, AK4961_ADC_SYNC_SET_SDCDC_MASK, data->sync_domain_select ) );

        /* Analog/Digital MIC select: select analog.*/
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_DMIC_MASK, 0 ) );

        /* Analog MIC/Line Selector. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_mic_input_select, AK4961_MIC_INPUT_SEL_RIGHT_MASK | AK4961_MIC_INPUT_SEL_LEFT_MASK, mic_input_select ) );

        /*
         * Analog MIC power-up sequence.
         *
         *      1. MIC Power ON
         *      2. MIC-Amp ON
         *      3. ADC ON
         *
        */

        if ( rtd->pm3_1 == 0 )
        {
            /* CP1. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP1_MASK, AK4961_PM3_PMCP1_SET ) );
            AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_PMCP1_WAIT_IN_MILLIS ) );

            /* LDO1. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMLDO1_MASK, AK4961_PM3_PMLDO1_MASK ) );
            AK4961_VERIFY( wiced_rtos_delay_milliseconds( AK4961_PMLDO1_WAIT_IN_MILLIS ) );
        }
        rtd->pm3_1 |= (1U << route_id);

        if ( pm4_pmmp_mask != 0 )
        {
            uint8_t mic_pwr_lvl      = 0;
            uint8_t mic_pwr_lvl_mask = 0;

            if ( (analog->input_left != NULL) && (analog->input_left->power_supply != AK4961_MIC_POWER_SUPPLY_MAX) )
            {
                const uint8_t mic_pwr_lvl_shift = ak4961_mic_pwr_lvl_shift( analog->input_left->power_supply );
                mic_pwr_lvl         |= ( analog->input_left->output_voltage << mic_pwr_lvl_shift );
                mic_pwr_lvl_mask    |= ( 0x3 << mic_pwr_lvl_shift );
            }
            if ( (analog->input_right != NULL) && (analog->input_right->power_supply != AK4961_MIC_POWER_SUPPLY_MAX) )
            {
                const uint8_t mic_pwr_lvl_shift = ak4961_mic_pwr_lvl_shift( analog->input_right->power_supply );
                mic_pwr_lvl         |= ( analog->input_right->output_voltage << mic_pwr_lvl_shift );
                mic_pwr_lvl_mask    |= ( 0x3 << mic_pwr_lvl_shift );
            }

            /* Output power voltage. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_MIC_PWR_LVL, mic_pwr_lvl_mask, mic_pwr_lvl ) );

            /* Analog MIC power supply (full-differential). */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM4, pm4_pmmp_mask, pm4_pmmp_mask ) );
        }

        /* AMP Gain. */
        if ( analog->input_left != NULL )
        {
            AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_mic_amp_lch_gain, AK4961_MIC_AMP_GAIN_MASK, analog->input_left->amp_gain ) );
        }
        if (analog->input_right != NULL )
        {
            AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_mic_amp_rch_gain, AK4961_MIC_AMP_GAIN_MASK, analog->input_right->amp_gain ) );
        }

        /* Input circuit. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM5, pm5_pmain_mask, pm5_pmain_mask ) );

        /* ADC / AMP / Mono/Stereo. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM6, adc->pm6_pmad_mask, pm6_pmad_select ) );
    }
    else
    {
        /*
         * Power-down sequence.
        */

        /* ADC. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM6, adc->pm6_pmad_mask, 0 ) );

        /* AMP.*/
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM5, pm5_pmain_mask, 0 ) );

        /* MIC.*/
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM4, pm4_pmmp_mask, 0 ) );

        rtd->pm3_1 &= ~( 1U << route_id );
        if ( rtd->pm3_1 == 0 )
        {
            /* CP1. */
            AK4961_VERIFY( ak4961_upd_bits(ak4961, AK4961_REG_PM3, AK4961_PM3_PMCP1_MASK, 0 ) );

            /* LDO1. */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_PM3, AK4961_PM3_PMLDO1_MASK, 0 ) );
        }
    }

    return WICED_SUCCESS;
}

/*
 * DMIC1 -> ADC1
 * DMIC2 -> ADC2
 */
wiced_result_t ak4961_adc_dmic_route(ak4961_device_data_t *driver_data, wiced_bool_t is_pwr_up)
{
    ak4961_device_cmn_data_t        *ak4961     = driver_data->cmn;
    const ak4961_adc_route_data_t   *data       = (const ak4961_adc_route_data_t *) driver_data->route;
    const ak4961_adc_route_t        *adc_route  = (const ak4961_adc_route_t *) data->base.device_route->instance;
    const ak4961_adc_t              *adc        = adc_route->adc;
    const ak4961_adc_digital_t      *digital    = &data->type.digital;

    if ( is_pwr_up == WICED_TRUE )
    {
        /*
         * Pre- Power-up configuration.
        */

        /* Select source/sink and setup sync domain */
        const ak4961_sync_domain_t *sync_domain = data->sink_port->sync_domain;
        AK4961_VERIFY( ak4961_upd_bits( ak4961, data->sink_port->reg_source_address, AK4961_SRC_SEL_MASK, adc->source_address_select ) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, sync_domain->reg, sync_domain->mask, data->sync_domain_select << sync_domain->shift ) );
        /* XXX For both ADC1/2. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_ADC_SYNC_DOMAIN_SET, AK4961_ADC_SYNC_SET_SDCDC_MASK, data->sync_domain_select ) );

        /* Digital MIC select. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_DMIC_MASK, AK4961_DMIC_DMIC_SEL ) );

        /* Clock polarity. */
        const uint8_t dmic_dclkp = digital->polarity << AK4961_DMIC_DCLKP_SHIFT;
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_DCLKP_MASK, dmic_dclkp ) );

        /* Digital MIC power-up / Mono/Stereo.
         * Digital block (decimation filter and HPF) powered.
        */
        uint8_t pmdm_select = 0;
        if ( digital->lch_enabled != 0 )
        {
            pmdm_select = (1U << AK4961_DMIC_PMDML_SHIFT );
        }
        if ( digital->rch_enabled != 0 )
        {
            pmdm_select = (1U << AK4961_DMIC_PMDMR_SHIFT );
        }
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_PMDML_MASK | AK4961_DMIC_PMDMR_MASK, pmdm_select ) );

        /* Enable DMCLK line. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_DCLKE_MASK, AK4961_DMIC_DCLKE_MASK ) );
    }
    else
    {
        /*
         * Power-down sequence.
        */

        /* Disable DMCLK line. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_DCLKE_MASK, 0 ) );

        /* DMIC power-down. */
        AK4961_VERIFY( ak4961_upd_bits( ak4961, adc->reg_dmic, AK4961_DMIC_PMDML_MASK | AK4961_DMIC_PMDMR_MASK, 0 ) );
    }

    return WICED_SUCCESS;
}

wiced_result_t ak4961_spkr(ak4961_device_data_t *driver_data, wiced_bool_t is_pwr_up)
{
    return WICED_UNSUPPORTED;
}

wiced_result_t ak4961_start_play ( void* driver_data )
{
    ak4961_device_data_t            *dd         = ( ak4961_device_data_t *) driver_data;
    const ak4961_route_data_t       *route_data = (const ak4961_route_data_t *) dd->route;
    wiced_result_t                  result      = WICED_SUCCESS;

    if (route_data->device_type == AK4961_DEVICE_TYPE_PLAYBACK)
    {
        ak4961_dac_route_data_t         *dac_data   = (ak4961_dac_route_data_t *) route_data;
        const ak4961_dac_route_t        *dac_route  = (const ak4961_dac_route_t *) dac_data->base.device_route->instance;
        const ak4961_amp_t              *amp        = dac_route->amp;

        result = ak4961_upd_bits(dd->cmn, amp->reg_vol_ctrl, amp->vol_ctrl_gain_mask, dac_data->amp_gain_current << amp->vol_ctrl_gain_shift);
    }

    return result;
}


wiced_result_t ak4961_stop_play ( void* driver_data )
{
    ak4961_device_data_t            *dd         = ( ak4961_device_data_t* ) driver_data;
    const ak4961_route_data_t       *route_data = ( const ak4961_route_data_t *) dd->route;
    wiced_result_t                  result      = WICED_SUCCESS;

    if (route_data->device_type == AK4961_DEVICE_TYPE_PLAYBACK)
    {
        ak4961_dac_route_data_t         *dac_data   = (ak4961_dac_route_data_t *) route_data;
        const ak4961_dac_route_t        *dac_route  = (const ak4961_dac_route_t *) dac_data->base.device_route->instance;
        const ak4961_amp_t              *amp        = dac_route->amp;

        result = ak4961_upd_bits(dd->cmn, amp->reg_vol_ctrl, amp->vol_ctrl_gain_mask, dac_data->amp_gain_mute << amp->vol_ctrl_gain_shift);
    }

    return result;
}


static wiced_result_t ak4961_set_volume(void *driver_data, double decibels, const ak4961_dbl_range_t *r, uint16_t reg, uint8_t mask, uint8_t shift)
{
    ak4961_device_data_t        *dd     = (ak4961_device_data_t *) driver_data;
    ak4961_device_cmn_data_t    *ak4961 = dd->cmn;
    uint8_t                     val;

    val = ak4961_double_range_to_index(ak4961, decibels, r);

    /* Set Volume */
    return ak4961_upd_bits(ak4961, reg, mask, val << shift);
}


wiced_result_t ak4961_set_playback_volume(void *driver_data, double decibels)
{
    ak4961_device_data_t            *dd         = (ak4961_device_data_t *) driver_data;
    ak4961_dac_route_data_t         *data       = (ak4961_dac_route_data_t *) dd->route;
    const ak4961_dac_route_t        *dac_route  = (const ak4961_dac_route_t *) data->base.device_route->instance;
    const ak4961_amp_t              *amp        = dac_route->amp;

    data->amp_gain_current = ak4961_double_range_to_index(dd->cmn, decibels, amp->gain_range);

    /* Set Volume */
    return ak4961_upd_bits(dd->cmn, amp->reg_vol_ctrl, amp->vol_ctrl_gain_mask, data->amp_gain_current << amp->vol_ctrl_gain_shift);
}

wiced_result_t ak4961_set_capture_volume(void *driver_data, double decibels)
{
    ak4961_device_data_t            *dd         = (ak4961_device_data_t *) driver_data;
    const ak4961_adc_route_data_t   *data       = (const ak4961_adc_route_data_t *) dd->route;
    const ak4961_adc_route_t        *adc_route  = (const ak4961_adc_route_t *) data->base.device_route->instance;
    const ak4961_adc_t              *adc        = adc_route->adc;

    if ( adc_route->is_dmic == 0 )
    {
        const ak4961_adc_analog_t   *analog     = &data->type.analog;

        /* Analog MIC. */
        if ( analog->input_left != NULL )
        {
            AK4961_VERIFY( ak4961_set_volume(driver_data, decibels, adc_route->mic_amp_gain_range, adc->reg_mic_amp_lch_gain, AK4961_MIC_AMP_GAIN_MASK, AK4961_MIC_AMP_GAIN_SHIFT) );
        }
        if ( analog->input_right != NULL )
        {
            AK4961_VERIFY( ak4961_set_volume(driver_data, decibels, adc_route->mic_amp_gain_range, adc->reg_mic_amp_rch_gain, AK4961_MIC_AMP_GAIN_MASK, AK4961_MIC_AMP_GAIN_SHIFT) );
        }
    }
    else
    {
        /* Digital MIC. */
        wiced_assert("Digital MIC doesn't have a digital volume!", 0==1);
        return WICED_FALSE;
    }

    return WICED_SUCCESS;
}


wiced_result_t ak4961_get_playback_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    ak4961_device_data_t            *dd         = (ak4961_device_data_t *) driver_data;
    const ak4961_adc_route_data_t   *data       = (const ak4961_adc_route_data_t *) dd->route;
    const ak4961_dac_route_t        *dac_route  = (const ak4961_dac_route_t *) data->base.device_route->instance;
    const ak4961_amp_t              *amp        = dac_route->amp;

    *min_volume_in_decibels = amp->gain_range->min;
    *max_volume_in_decibels = amp->gain_range->max;

    return WICED_SUCCESS;
}

wiced_result_t ak4961_get_capture_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    ak4961_device_data_t            *dd         = (ak4961_device_data_t *) driver_data;
    const ak4961_adc_route_data_t   *data       = (const ak4961_adc_route_data_t *) dd->route;
    const ak4961_adc_route_t        *adc_route  = (const ak4961_adc_route_t *) data->base.device_route->instance;

    if ( adc_route->mic_amp_gain_range != 0 )
    {
        /* Analog MIC. */
        *min_volume_in_decibels = adc_route->mic_amp_gain_range->min;
        *max_volume_in_decibels = adc_route->mic_amp_gain_range->max;
    }
    else
    {
        /* Digital MIC. */
        *min_volume_in_decibels = 0.0;
        *max_volume_in_decibels = 0.0;
    }

    return WICED_SUCCESS;
}

static wiced_result_t ak4961_set_effect(void *driver_data, uint8_t mode)
{
    ak4961_device_data_t            *dd              = (ak4961_device_data_t *) driver_data;
    ak4961_device_cmn_data_t        *ak4961          = dd->cmn;
    ak4961_device_runtime_data_t    *rtd             = ak4961->rtd;
    const ak4961_route_data_t       *route_data      = (const ak4961_route_data_t *) dd->route;
    const ak4961_route_id_t         route_id         = route_data->id;

    if (  ( rtd->init & ( 1 << route_id ) ) == 0  )
    {
        return WICED_NOTUP;
    }
    else if ( mode >= AK4961_DSP_EFFECT_MODE_MAX )
    {
        return WICED_BADARG;
    }

    if ( route_data->device_type != AK4961_DEVICE_TYPE_PLAYBACK )
    {
        return WICED_UNSUPPORTED;
    }
    else
    {
        ak4961_dac_route_data_t        *dac_data            = (ak4961_dac_route_data_t *) route_data;
        const ak4961_dsp_effect_data_t *dsp_effect_data     = &ak4961->dsp_effect_data[mode];
        const ak4961_dac_route_t       *dac_route           = (const ak4961_dac_route_t *) dac_data->base.device_route->instance;
        const ak4961_dac_t             *dac                 = dac_route->dac;
        const ak4961_dsp_t             *dsp                 = dsp_effect_data->dsp;
        const uint8_t                  *dsp_mode_ram        = dsp_effect_data->dsp_mode_ram;
        uint8_t                        dsp_mode_ram_size    = dsp_effect_data->dsp_mode_ram_size;

        if ( mode == AK4961_DSP_EFFECT_MODE_NONE )
        {
            const ak4961_sync_domain_t *sync_domain = dac_data->source_port->sync_domain;

            AK4961_VERIFY( ak4961_upd_bits( ak4961, sync_domain->reg, sync_domain->mask, dac_data->sync_domain_select << sync_domain->shift ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_dac_src_sel, AK4961_SRC_SEL_MASK, dac_data->source_port->source_address_select ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_ADC_SYNC_DOMAIN_SET, AK4961_ADC_SYNC_SET_SDCDC_MASK, dac_data->sync_domain_select ) );

            /* Power-down DSP */
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_PMSW_MASK, 0 ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC1, AK4961_FC1_SWRSTN_MASK, 0 ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_FC3, AK4961_FC3_DSPRSTN_MASK, 0 ) );
        }
        else
        {
            const ak4961_sync_domain_t *sync_domain = dac_data->source_port->sync_domain;
            const ak4961_sync_domain_t *dsp_sync_domain = dsp_effect_data->dsp_source_port->sync_domain;

            AK4961_VERIFY( ak4961_upd_bits( ak4961, sync_domain->reg, sync_domain->mask, dac_data->sync_domain_select << sync_domain->shift ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, dsp_sync_domain->reg, dsp_sync_domain->mask, dsp_effect_data->dsp_sync_domain_select << dsp_sync_domain->shift ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, dsp->reg_dsp_src_sel, AK4961_SRC_SEL_MASK, dac_data->source_port->source_address_select ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_dac_src_sel, AK4961_SRC_SEL_MASK, dsp_effect_data->dsp_source_port->source_address_select ) );
            AK4961_VERIFY( ak4961_upd_bits( ak4961, AK4961_REG_ADC_SYNC_DOMAIN_SET, AK4961_ADC_SYNC_SET_SDCDC_MASK, dac_data->sync_domain_select ) );

            /* Power-up DSP and download DSP program(PRAM and CRAM) */
            AK4961_VERIFY ( ak4961_ram_download( ak4961, dsp_effect_data->pram_res, dsp_effect_data->pram_tab_size, dsp_effect_data->cram_res, dsp_effect_data->cram_tab_size ) );

            /* Set DSP effect mode */
            AK4961_VERIFY( ak4961_ram_write( ak4961, dsp_mode_ram, dsp_mode_ram_size ) );
            AK4961_VERIFY( ak4961_reg_write( ak4961, AK4961_REG_CRAM_OP_RUN_STATE, AK4961_CRAM_OP_RUNC ) );
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t ak4961_set_dac_output_mixing(void *driver_data, wiced_audio_dac_output_mixing_t *mode)
{
    ak4961_device_data_t            *dd              = (ak4961_device_data_t *) driver_data;
    ak4961_device_cmn_data_t        *ak4961          = dd->cmn;
    ak4961_device_runtime_data_t    *rtd             = ak4961->rtd;
    const ak4961_route_data_t       *route_data      = (const ak4961_route_data_t *) dd->route;
    const ak4961_route_id_t         route_id         = route_data->id;

    if (  ( rtd->init & ( 1 << route_id ) ) == 0  )
    {
        return WICED_NOTUP;
    }

    if ( (mode->left_channel_select > AK4961_DAC_OUTPUT_SELECT_LCH_RCH_DIV_2) ||
         (mode->right_channel_select > AK4961_DAC_OUTPUT_SELECT_LCH_RCH_DIV_2) )
    {
        return WICED_BADARG;
    }

    if ( route_data->device_type != AK4961_DEVICE_TYPE_PLAYBACK )
    {
        return WICED_UNSUPPORTED;
    }
    else
    {
        ak4961_dac_route_data_t  *dac_data  = (ak4961_dac_route_data_t *) route_data;
        const ak4961_dac_route_t *dac_route = (const ak4961_dac_route_t *) dac_data->base.device_route->instance;
        const ak4961_dac_t       *dac       = dac_route->dac;
        uint8_t                   mix       = 0;

        /* Select Mono Mix. */
        mix = ( (mode->right_channel_select << AK4961_DAC_MIX_RIGHT_SHIFT) | (mode->left_channel_select << AK4961_DAC_MIX_LEFT_SHIFT) );
        AK4961_VERIFY( ak4961_upd_bits( ak4961, dac->reg_dac_mono_mix, AK4961_DAC_MIX_RIGHT_MASK | AK4961_DAC_MIX_LEFT_MASK, mix ) );
    }

    return WICED_SUCCESS;
}

wiced_result_t ak4961_ioctl( void *driver_data, wiced_audio_device_ioctl_t cmd, wiced_audio_device_ioctl_data_t *cmd_data )
{
    wiced_result_t result = WICED_UNSUPPORTED;

    if ( cmd_data == NULL )
    {
        return WICED_BADARG;
    }

    switch ( cmd )
    {
        case WICED_AUDIO_IOCTL_SET_DSP_EFFECT:
            result = ak4961_set_effect( driver_data, cmd_data->dsp_effect_mode );
            break;

        case WICED_AUDIO_IOCTL_SET_DAC_OUTPUT_MIXING:
            result = ak4961_set_dac_output_mixing(driver_data, &cmd_data->dac_output_mode);
            break;
    }

    return result;
}


static wiced_result_t ak4961_init_device_runtime_data( ak4961_device_runtime_data_t *rtd )
{
    if ( rtd->rdy == 1 )
    {
        /* Already initialized */
        return WICED_SUCCESS;
    }

    AK4961_VERIFY( wiced_rtos_init_mutex( &rtd->lock ) );

    rtd->init   = 0;
    rtd->cfg    = 0;
    rtd->rdy    = 1;
    rtd->pm3_1  = 0;
    rtd->pm3_3  = 0;

    return WICED_SUCCESS;
}


/* This function can only be called from the platform initialization routine */
wiced_result_t ak4961_device_register( ak4961_device_data_t *device_data, const platform_audio_device_id_t device_id )
{
    if ( device_data == NULL )
    {
        return WICED_BADARG;
    }

    /* Initialize private portion of device interface. */
    const ak4961_route_data_t       *data = (const ak4961_route_data_t *) device_data->route;
    wiced_audio_device_interface_t  *adi  = data->device_route->adi;
    adi->audio_device_driver_specific = device_data;
    adi->device_id = device_id;

    /* Initialize runtime data. */
    AK4961_VERIFY( ak4961_init_device_runtime_data( device_data->cmn->rtd ) );

    /* Register a device to the audio device list and keep device data internally from this point */
    return wiced_register_audio_device( device_id, adi );
}
