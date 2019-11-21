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

/**
 * @file
 *
 * Collection of CONSTANT tables for PCM Apollo specification
 *
 * Copyright (C) 2012 Broadcom Corporation
 *
 * $Id$
 */
#ifndef _H_AUDIOPCM_TABLES_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "audiopcm_types.h"

/* format field generic type : 8bit field */
typedef uint8_t pcm_fmt_field_t;

/*
 * table_fmt_type column definition
 */
#define LUT_FMT_TYPE_ID          ( 0 )
#define LUT_FMT_TYPE_BPS         ( 1 )
#define LUT_FMT_TYPE_CBPS        ( 2 )
#define LUT_FMT_TYPE_SR          ( 3 )
#define LUT_FMT_TYPE_ENDIAN      ( 4 )
#define LUT_FMT_TYPE_SIGN        ( 5 )
#define LUT_FMT_TYPE_INTERL      ( 6 )
#define LUT_FMT_TYPE_FLOAT       ( 7 )
#define LUT_FMT_TYPE_MAXCHCNT    ( 8 )
#define LUT_FMT_TYPE_CODEC       ( 9 )

/*
 * table_fmt_type values
 */
#define LUT_FMT_BPS_16          ( 16 )
#define LUT_FMT_BPS_24          ( 24 )
#define LUT_FMT_BPS_32          ( 32 )

#define LUT_FMT_CBPS_16         ( 16 )
#define LUT_FMT_CBPS_24         ( 24 )
#define LUT_FMT_CBPS_32         ( 32 )

#define LUT_FMT_SR_44100         ( 0 )
#define LUT_FMT_SR_48000         ( 1 )
#define LUT_FMT_SR_96000         ( 2 )
#define LUT_FMT_SR_192000        ( 3 )

#define LUT_FMT_ENDIAN_L         ( 0 )
#define LUT_FMT_ENDIAN_B         ( 1 )

#define LUT_FMT_INTERL_C         ( 0 )
#define LUT_FMT_INTERL_I         ( 1 )

#define LUT_FMT_FLOAT_I          ( 0 )
#define LUT_FMT_FLOAT_F          ( 1 )

/**
 *
 * Note: this table is put in a separate file because it is statically allocated.
 * This table is crucial for the demux behavior, it does specify which formats are
 * supported and how they are mapped to the media type.
 *
 * Note: only supported format combinations are listed, each one identified by
 *       a unique id on the first column for internal reference comparison.
 *
 * Note: the very last line of the table is closed with "unknowns", this is MANDATORY
 *       to stop the search loop when looking for a match, so DO NOT forget to close
 *       the last line of this table.
 */
static const pcm_fmt_field_t lut_fmt_type[][ 10 ] =
{
    /*ID,    BPS            CBPS               SR                ENDIANNESS        SIGNEDNESS       INTERLEAVING      FLOATING   MAX.CH.CNT   |   SUPPORTED CODEC       */
    /* ---------------------------------------------------------------------------------------------------------------------------------------------------------------- */
    /* ID==0 is RESERVED                                                                                                                                                */

    {                  1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              9, PCM_FMT_CODEC_PCM16 },
    {                  2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              9, PCM_FMT_CODEC_PCM16 },
    {                  3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              6, PCM_FMT_CODEC_PCM16 },
    {                  4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_192000, PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              2, PCM_FMT_CODEC_PCM16 },

    {                  5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              9, PCM_FMT_CODEC_PCM32 },
    {                  6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              9, PCM_FMT_CODEC_PCM32 },
    {                  7, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              3, PCM_FMT_CODEC_PCM32 },
    {                  8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_192000, PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              1, PCM_FMT_CODEC_PCM32 },

    {                  9, LUT_FMT_BPS_24, LUT_FMT_CBPS_32, LUT_FMT_SR_44100,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              8, PCM_FMT_CODEC_PCM32 },
    {                 10, LUT_FMT_BPS_24, LUT_FMT_CBPS_32, LUT_FMT_SR_48000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              8, PCM_FMT_CODEC_PCM32 },
    {                 11, LUT_FMT_BPS_24, LUT_FMT_CBPS_32, LUT_FMT_SR_96000,  PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              3, PCM_FMT_CODEC_PCM32 },
    {                 12, LUT_FMT_BPS_24, LUT_FMT_CBPS_32, LUT_FMT_SR_192000, PCM_FMT_ANY,    PCM_FMT_ANY,    PCM_FMT_ANY,    LUT_FMT_FLOAT_I,              1, PCM_FMT_CODEC_PCM32 },

    { PCM_FMT_ID_UNKNOWN, PCM_FMT_UKNOWN, PCM_FMT_UKNOWN,  PCM_FMT_UKNOWN,    PCM_FMT_UKNOWN, PCM_FMT_UKNOWN, PCM_FMT_UKNOWN, PCM_FMT_UKNOWN,  PCM_FMT_UKNOWN, PCM_FMT_UKNOWN      }
};



#define LUT_FMT_CHMAP_MONO ( 0 )

/**
 *
 */
static const uint8_t         lut_pcm_bps[ 2 ] = { 16, 24 };
static const uint32_t        lut_pcm_sr[ 4 ] = { 44100, 48000, 96000, 192000 };


#define LUT_CHMAP_CHNUM ( 0 )
#define LUT_CHMAP_CHMAP ( 1 )

/*
 * CHMAP definition as per Apollo 2.0 specs (6 bits)
 * first column is the number of channels in a map
 * second column is the actual map string for channel demux matching
 */
static const uint32_t        lut_chmap[ 64 ][ 2 ] =
{
    { /*  0 */ 1,  ( APCM_CHANNEL_MAP_MONO )                                                                                                                                            },
    { /*  1 */ 2,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR )                                                                                                                       },
    { /*  2 */ 3,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_LFE1 )                                                                                              },
    { /*  3 */ 3,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC )                                                                                                },
    { /*  4 */ 3,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_BC )                                                                                                },
    { /*  5 */ 4,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC )                                                                        },
    { /*  6 */ 4,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BC )                                                                        },
    { /*  7 */ 4,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BC )                                                                        },
    { /*  8 */ 5,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC )                                                },
    { /*  9 */ 5,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BR )                                                },
    { /* 10 */ 5,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_SIL  | APCM_CHANNEL_MAP_SIR )                                               },
    { /* 11 */ 6,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR )                        },
    { /* 12 */ 6,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_LS  | APCM_CHANNEL_MAP_RS )                         },
    { /* 13 */ 6,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BC   | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR )                        },
    { /* 14 */ 7,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_LS  | APCM_CHANNEL_MAP_RS )  },
    { /* 15 */ 7,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR ) },
    { /* 16 */ 7,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BR  | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR ) },
    { /* 17 */ 7,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BR  | APCM_CHANNEL_MAP_FLC | APCM_CHANNEL_MAP_FRC ) },
    { /* 18 */ 7,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_LS  | APCM_CHANNEL_MAP_RS )  },
    { /* 19 */ 8,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR |
                     APCM_CHANNEL_MAP_BLC | APCM_CHANNEL_MAP_BRC ) },
    { /* 20 */ 8,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_FLC | APCM_CHANNEL_MAP_FRC ) },
    { /* 21 */ 8,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_FLC | APCM_CHANNEL_MAP_FRC |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR ) },
    { /* 22 */ 8,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_TPLS | APCM_CHANNEL_MAP_TPRS ) },
    { /* 23 */ 9,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_LS  | APCM_CHANNEL_MAP_RS ) },
    { /* 24 */ 9,  ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BR  | APCM_CHANNEL_MAP_SIL |
                     APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR ) },
    { /* 25 */ 10, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR | APCM_CHANNEL_MAP_LS   | APCM_CHANNEL_MAP_RS ) },
    { /* 26 */ 10, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR ) },
    { /* 27 */ 10, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_TPSIR | APCM_CHANNEL_MAP_TPSIR | APCM_CHANNEL_MAP_TPLS   | APCM_CHANNEL_MAP_RS ) },
    { /* 28 */ 10, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_TPLS | APCM_CHANNEL_MAP_TPRS | APCM_CHANNEL_MAP_LS   | APCM_CHANNEL_MAP_RS ) },
    { /* 29 */ 11, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_BL   | APCM_CHANNEL_MAP_BR  | APCM_CHANNEL_MAP_FLC |
                     APCM_CHANNEL_MAP_FRC | APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR  | APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR ) },
    { /* 30 */ 12, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_FLC | APCM_CHANNEL_MAP_FRC | APCM_CHANNEL_MAP_SIL  | APCM_CHANNEL_MAP_SIR  | APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR ) },
    { /* 31 */ 12, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_TPSIL | APCM_CHANNEL_MAP_TPSIR | APCM_CHANNEL_MAP_TPLS  | APCM_CHANNEL_MAP_TPRS  | APCM_CHANNEL_MAP_LS | APCM_CHANNEL_MAP_RS ) },
    { /* 32 */ 12, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPLS  | APCM_CHANNEL_MAP_TPRS  | APCM_CHANNEL_MAP_LS | APCM_CHANNEL_MAP_RS ) },
    { /* 33 */ 12, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_LFE2  |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPFL  | APCM_CHANNEL_MAP_TPFR  | APCM_CHANNEL_MAP_LS | APCM_CHANNEL_MAP_RS ) },
    { /* 34 */ 14, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BL  | APCM_CHANNEL_MAP_BR  |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPSIL | APCM_CHANNEL_MAP_TPSIR | APCM_CHANNEL_MAP_TPLS | APCM_CHANNEL_MAP_TPRS |
                     APCM_CHANNEL_MAP_TPLS | APCM_CHANNEL_MAP_TPRS ) },
    { /* 35 */ 14, ( APCM_CHANNEL_MAP_FL  | APCM_CHANNEL_MAP_FR  | APCM_CHANNEL_MAP_FC   | APCM_CHANNEL_MAP_LFE1 | APCM_CHANNEL_MAP_BC  | APCM_CHANNEL_MAP_LFE2  |
                     APCM_CHANNEL_MAP_SIL | APCM_CHANNEL_MAP_SIR | APCM_CHANNEL_MAP_TPFL | APCM_CHANNEL_MAP_TPFR | APCM_CHANNEL_MAP_LS | APCM_CHANNEL_MAP_RS |
                     APCM_CHANNEL_MAP_BLC | APCM_CHANNEL_MAP_BRC ) },
    { /* 36 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 37 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 38 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 39 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 40 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 41 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 42 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 43 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 44 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 45 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 46 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 47 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 48 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 49 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 50 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 51 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 52 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 53 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 54 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 55 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 56 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 57 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 58 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 59 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 60 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 61 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 62 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                },
    { /* 63 */ 0,  APCM_CHANNEL_MAP_NONE                                                                                                                                                }
};


#define LUT_VIRTUALCH_CHNUM  ( 0 )
#define LUT_VIRTUALCH_VMAP   ( 1 )
#define LUT_VIRTUALCH_MAP    ( 2 )
#define LUT_VIRTUALCH_FITLER ( 3 )

/*
 * virtual channel corrispondance to demux the proper channels
 * based on the requested virtual channel
 */
static const uint32_t        lut_virtualch[][ 4 ] =
{
    { 2, ( APCM_CHANNEL_MAP_C  | APCM_CHANNEL_VIRTUAL ),  ( APCM_CHANNEL_MAP_FL | APCM_CHANNEL_MAP_FR | APCM_CHANNEL_VIRTUAL ), 0 }, /* virtual center channel from L+R */
    { 2, ( APCM_CHANNEL_MAP_LFE | APCM_CHANNEL_VIRTUAL ), ( APCM_CHANNEL_MAP_FL | APCM_CHANNEL_MAP_FR | APCM_CHANNEL_VIRTUAL ), 0 }, /* virtual LFE channel from L+R */
    /* always close lut with nulls*/
    { 0, APCM_CHANNEL_MAP_NONE,                           APCM_CHANNEL_MAP_NONE,                                                0 }
};



#define LUT_STREAM_CFG_IDX         ( 0 )
#define LUT_STREAM_CFG_CHNUM       ( 1 )
#define LUT_STREAM_CFG_BPS         ( 2 )
#define LUT_STREAM_CFG_CBPS        ( 3 )
#define LUT_STREAM_CFG_SR          ( 4 )
#define LUT_STREAM_CFG_BL          ( 5 )
#define LUT_STREAM_CFG_SL          ( 6 )
#define LUT_STREAM_CFG_SAMPLES     ( 7 )
#define LUT_STREAM_CFG_LATENCY     ( 8 )
#define LUT_STREAM_CFG_PRIORITY    ( 9 )

static const uint16_t        lut_stream_cfg[][ 10 ] =
{
    /*ID, CH.NUM    BPS        CBPS             SR             BL  SL SAMPLES  Latency (ms)  priority */
    /* ------------------------------------------------------------------------------------------------ */
    /* ID==0 is RESERVED */

    /* 16 BIT, 1CH */
    {   1,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 352,  63, 100 },
    {   2,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   6,   6, 348,  47, 255 },
    {   3,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   6,   3, 348,  47, 200 },
    {   4,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 352,  58, 100 },
    {   5,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   6, 348,  43, 255 },
    {   6,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   3, 348,  43, 200 },
    {   7,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,  16, 352,  58, 200 },
    {   8,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,   8, 352,  58, 100 },
    {   9,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   8, 352,  29, 255 },
    {  10,   1, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   4, 352,  29, 100 },
    /* 16 BIT, 2 CH */
    {  11,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 352,  63, 100 },
    {  12,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   6,   6, 348,  47, 255 },
    {  13,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   6,   3, 348,  47, 200 },
    {  14,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 352,  58, 100 },
    {  15,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   6, 348,  43, 255 },
    {  16,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   3, 348,  43, 200 },
    {  17,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,  16, 352,  58, 200 },
    {  18,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,   8, 352,  58, 100 },
    {  19,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   8, 352,  29, 255 },
    {  20,   2, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   4, 352,  29, 100 },
    /* 16 BIT, 3 CH */
    {  21,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  10,  10, 230,  52, 200 },
    {  22,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 232,  42, 255 },
    {  23,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  10,  10, 230,  47, 200 },
    {  24,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 232,  38, 255 },
    {  25,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   6, 234,  29, 200 },
    {  26,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  20,  20, 220,  45, 200 },
    {  27,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,  16, 224,  37, 255 },
    {  28,   3, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   8, 224,  18, 200 },
    /* 16 BIT, 4 CH */
    {  28,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  10,  10, 170,  39, 200 },
    {  29,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 176,  32, 255 },
    {  30,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  10,  10, 170,  35, 200 },
    {  31,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 176,  29, 255 },
    {  32,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   6, 174,  22, 200 },
    {  33,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  20,  20, 160,  33, 200 },
    {  34,   4, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,  16, 176,  29, 255 },
    /* 16 BIT, 5 CH */
    {  34,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  10,  10, 140,  32, 255 },
    {  35,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 136,  25, 200 },
    {  36,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  10,  10, 140,  29, 255 },
    {  37,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 136,  23, 200 },
    {  38,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   6,   6, 138,  18, 200 },
    {  39,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  20,  20, 140,  29, 255 },
    {  40,   5, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,   8,   8, 136,  11, 200 },
    /* 16 BIT, 6 CH */
    {  41,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  16,  16, 112,  40, 255 },
    {  42,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  16,   8, 112,  40, 200 },
    {  43,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,   8,   8, 112,  20, 200 },
    {  44,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  20,  20, 100,  41, 100 },
    {  45,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  16,  16, 112,  37, 255 },
    {  46,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  16,   8, 112,  37, 128 },
    {  47,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8, 112,  18, 100 },
    {  48,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  36,  36, 108,  40,  85 },
    {  49,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  36,  36,  72,  27, 200 },
    {  50,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  32,  32,  96,  32, 255 },
    {  51,   6, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  18,  18,  72,  13,  80 },
    /* 16 BIT, 8 CH */
    {  52,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  36,  36,  72,  58, 100 },
    {  53,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  32,  32,  64,  46,  80 },
    {  54,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  28,  28,  84,  53, 100 },
    {  55,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  16,  16,  80,  29, 255 },
    {  56,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_44100,  16,   8,  80,  29, 100 },
    {  57,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  36,  36,  72,  54, 200 },
    {  58,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,  16,  16,  80,  26, 255 },
    {  59,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_48000,   8,   8,  80,  13, 100 },
    {  60,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  80,  80,  80,  66,  80 },
    {  61,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  40,  40,  80,  33, 200 },
    {  62,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  20,  20,  80,  16, 200 },
    {  63,   8, LUT_FMT_BPS_16, LUT_FMT_CBPS_16, LUT_FMT_SR_96000,  16,  16,  80,  13, 255 },
    /* 24 BIT, 1 CH */
    {  64,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  16,  16, 224,  81,   0 },
    {  65,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  10,  10, 220,  49, 200 },
    {  66,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   8,   8, 224,  40, 255 },
    {  67,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,  16, 224,  74,   0 },
    {  68,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  10,  10, 220,  45, 200 },
    {  69,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,   6,   6, 228,  28, 255 },
    {  70,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  20,  20, 220,  45, 200 },
    {  71,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16, 224,  37, 255 },
    {  72,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8, 224,  18, 200 },
    {  73,   1, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   4, 224,  18, 200 },
    /* 24 BIT, 2 CH */
    {  74,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  16,  16, 224,  81,   0 },
    {  75,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  10,  10, 220,  49, 200 },
    {  76,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   8,   8, 224,  40, 255 },
    {  77,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,  16, 224,  74,   0 },
    {  78,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  10,  10, 220,  45, 200 },
    {  79,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,   6,   6, 228,  28, 255 },
    {  80,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  20,  20, 220,  45, 200 },
    {  81,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16, 224,  37, 255 },
    {  82,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8, 224,  18, 200 },
    {  83,   2, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   4, 224,  18, 200 },
    /* 24 BIT, 3 CH */
    {  84,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  16,  16, 144,  52, 255 },
    {  85,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   8,   8, 144,  26, 200 },
    {  86,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   6,   6, 144,  19, 200 },
    {  87,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,  16, 144,  48, 255 },
    {  88,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,   8, 144,  48, 200 },
    {  89,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  10,  10, 140,  29, 100 },
    {  90,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,   6,   6, 144,  18, 200 },
    {  91,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  20,  20, 140,  29, 100 },
    {  92,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16, 144,  24, 255 },
    {  93,   3, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8, 144,  12, 200 },
    /* 24 BIT, 4 CH */
    {  94,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  16,  16, 112,  40, 200 },
    {  95,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   8,   8, 112,  20, 255 },
    {  96,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   6,   6, 114,  15, 200 },
    {  97,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,  16, 112,  37, 255 },
    {  98,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,   8, 112,  37, 200 },
    {  99,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  10,  10, 110,  23, 200 },
    { 100,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,   6,   6, 114,  14, 200 },
    { 101,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16, 112,  18, 255 },
    { 102,   4, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8, 112,   9, 200 },
    /* 24 BIT, 5 CH */
    { 103,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  20,  20,  80,  36, 200 },
    { 104,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  15,  15,  90,  30, 255 },
    { 105,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8,  88,  16, 200 },
    { 106,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  30,  30,  90,  56, 200 },
    { 107,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  15,  15,  90,  28, 255 },
    { 108,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  10,  10,  90,  19, 200 },
    { 109,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  40,  40,  80,  33, 200 },
    { 110,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  30,  30,  90,  28, 255 },
    { 111,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16,  80,  13, 200 },
    { 112,   5, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  15,  15,  90,  14, 200 },
    /* 24 BIT, 6 CH */
    { 113,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  36,  36,  72,  58, 255 },
    { 114,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  32,  32,  64,  46, 200 },
    { 115,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  16,  16,  64,  23, 200 },
    { 116,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,   8,   8,  72,  13, 200 },
    { 117,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  36,  36,  72,  54, 255 },
    { 118,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  32,  32,  64,  42, 200 },
    { 119,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  16,  16,  64,  21, 200 },
    { 120,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,   8,   8,  72,  12, 200 },
    { 121,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  36,  36,  72,  27, 255 },
    { 122,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  32,  32,  64,  21, 200 },
    { 123,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  16,  16,  64,  10, 200 },
    { 124,   6, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,   8,   8,  72,   6, 200 },
    /* 24 BIT, 8 CH */
    { 125,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  48,  48,  48,  52, 255 },
    { 126,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  32,  32,  32,  23, 100 },
    { 127,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  24,  24,  48,  26, 255 },
    { 128,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_44100,  12,  12,  48,  13, 200 },
    { 129,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  48,  48,  48,  48, 255 },
    { 130,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_48000,  24,  24,  48,  24, 200 },
    { 131,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  48,  48,  48,  24, 255 },
    { 132,   8, LUT_FMT_BPS_24, LUT_FMT_CBPS_24, LUT_FMT_SR_96000,  24,  24,  48,  12, 200 },
    /* *** */
    { 255, 255,            255,             255,              255, 255, 255, 255, 255, 255 }
};


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_AUDIOPCM_TABLES_H_ */
