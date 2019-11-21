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
 * @file Apollo common RTP definitions and parameters
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/


#define RTP_DEFAULT_PORT                5004

#define RTP_PACKET_MAX_SIZE             1500
#define RTP_IP_HEADER_SIZE              28
#define RTP_BASE_HEADER_SIZE            12
#define RTP_EXT_SIZE_2_0                32

#define RTP_HEADER_SIZE_2_0             (RTP_BASE_HEADER_SIZE + RTP_EXT_SIZE_2_0)

#define RTP_HEADER_SIZE                 RTP_HEADER_SIZE_2_0                         /* The "current" header size */
#define RTP_PACKET_MAX_DATA             (RTP_PACKET_MAX_SIZE - RTP_HEADER_SIZE - RTP_IP_HEADER_SIZE)
#define RTP_PACKET_MIN_DATA             64

#define RTP_PAYLOAD_AUDIO               98
#define RTP_PAYLOAD_AUDIO_DUP           99
#define RTP_PAYLOAD_FEC                 100
#define RTP_MAX_SEQ_NUM                 ((uint32_t)0x0000FFFF)

/*
 * RTP Audio extension header definitions.
 */

#define RTP_AUDIO_EXT_ID_LPCM           0x00010000
#define RTP_AUDIO_EXT_AUDIO_LENGTH_2_0  7           /* Length of extension header in 32 bit units */

#define RTP_AUDIO_EXT_ID_FEC_XOR        0x00000000
#define RTP_AUDIO_EXT_FEC_LENGTH        1           /* Length of extension header in 32 bit units */

#define RTP_AUDIO_EXT_VER_MAJOR_MASK    (0x0F)
#define RTP_AUDIO_EXT_VER_MINOR_MASK    (0x0F)
#define RTP_AUDIO_EXT_VER_MAJOR_SHIFT   (28)
#define RTP_AUDIO_EXT_VER_MINOR_SHIFT   (24)

#define RTP_AUDIO_EXT_VERSION_MJR       (4)
#define RTP_AUDIO_EXT_VERSION_MIN       (0)
#define RTP_AUDIO_EXT_VERSION_MJRMIN    ( ((RTP_AUDIO_EXT_VERSION_MJR & RTP_AUDIO_EXT_VER_MAJOR_MASK)<<RTP_AUDIO_EXT_VER_MAJOR_SHIFT) | \
                                          ((RTP_AUDIO_EXT_VERSION_MIN & RTP_AUDIO_EXT_VER_MINOR_MASK)<<RTP_AUDIO_EXT_VER_MINOR_SHIFT) )

#define RTP_AUDIO_BL_MAX_LENGTH         36

/*
 * Audio format definitions.
 */

#define AUDIO_FORMAT_BPS_16                 (0 << 31)
#define AUDIO_FORMAT_BPS_24                 (1 << 31)

#define AUDIO_FORMAT_CONTAINER_NATIVE       (0 << 30)
#define AUDIO_FORMAT_CONTAINER_32BIT        (1 << 30)

#define AUDIO_FORMAT_LITTLE_ENDIAN          (0 << 29)
#define AUDIO_FORMAT_BIG_ENDIAN             (1 << 29)

#define AUDIO_FORMAT_UNSIGNED               (0 << 28)
#define AUDIO_FORMAT_SIGNED                 (1 << 28)

#define AUDIO_FORMAT_SPATIAL_INTERLEAVING   (0 << 27)
#define AUDIO_FORMAT_TIME_INTERLEAVING      (1 << 27)

#define AUDIO_FORMAT_INTEGER                (0 << 26)
#define AUDIO_FORMAT_FLOATING_POINT         (1 << 26)

#define AUDIO_FORMAT_SAMPLE_RATE_441        (0 << 24)
#define AUDIO_FORMAT_SAMPLE_RATE_48         (1 << 24)
#define AUDIO_FORMAT_SAMPLE_RATE_96         (2 << 24)
#define AUDIO_FORMAT_SAMPLE_RATE_192        (3 << 24)
#define AUDIO_FORMAT_SAMPLE_RATE_MASK       (3 << 24)

#define AUDIO_FORMAT_CUSTOM_CHMAP           (1 << 23)

#define AUDIO_FORMAT_CHMAP_MASK             (0x3F)
#define AUDIO_FORMAT_CHMAP_SHIFT            17


#define AUDIO_FORMAT_LATENCY_MASK           (0x3F)
#define AUDIO_FORMAT_SINK_JITTER_MASK       (0x0F)
#define AUDIO_FORMAT_VOL_ATT_MASK           (0x3F)
#define AUDIO_FORMAT_LATENCY_SHIFT          (0)
#define AUDIO_FORMAT_SINK_JITTER_SHIFT      (6)
#define AUDIO_FORMAT_VOL_ATT_SHIFT          (10)

#define AUDIO_FORMAT_BL_MASK                (0xFF)
#define AUDIO_FORMAT_BLSYNC_SHIFT           (24)
#define AUDIO_FORMAT_BL_SHIFT               (16)
#define AUDIO_FORMAT_SL_MASK                (0xFF)
#define AUDIO_FORMAT_SL_SHIFT               (16)

#define AUDIO_FORMAT_MCLOCK_UNIT_MASK       (0x3)
#define AUDIO_FORMAT_MCLOCK_UNIT_NSECS      (0)
#define AUDIO_FORMAT_MCLOCK_UNIT_PPB        (1)
#define AUDIO_FORMAT_MCLOCK_UNIT_TICKS      (2)
#define AUDIO_FORMAT_MCLOCK_UNIT_USR        (3)

#define AUDIO_FORMAT_MCU_MASK               (0x03)
#define AUDIO_FORMAT_MCU_SHIFT              (0)
#define AUDIO_FORMAT_ACT_MASK               (0x03)
#define AUDIO_FORMAT_ACT_SHIFT              (2)

/*
 * Uint32 offsets within the RTP packet.
 */

#define RTP_AUDIO_OFFSET_BASE           0
#define RTP_AUDIO_OFFSET_TIMESTAMP      1
#define RTP_AUDIO_OFFSET_SSRC           2
#define RTP_AUDIO_OFFSET_ID             3
#define RTP_AUDIO_OFFSET_VERSION        4
#define RTP_AUDIO_OFFSET_FEC_REF        4
#define RTP_AUDIO_OFFSET_VOL_ATT        4
#define RTP_AUDIO_OFFSET_SINK_JITTER    4
#define RTP_AUDIO_OFFSET_LATENCY_TARGET 4
#define RTP_AUDIO_OFFSET_SCLOCK_LO      5
#define RTP_AUDIO_OFFSET_SCLOCK_HI      6
#define RTP_AUDIO_OFFSET_FORMAT         7
#define RTP_AUDIO_OFFSET_MCLOCK_LO      8
#define RTP_AUDIO_OFFSET_MCLOCK_HI      9
#define RTP_AUDIO_OFFSET_BL            10
#define RTP_AUDIO_OFFSET_MCLOCK_UNIT   10

/*
 * Uint8 offsets within the Apollo RTP packet.
 */

#define RTP_AUDIO_OFFSET_U8_RAWDATA    (44)
#define RTP_AUDIO_OFFSET_U8_MCU        (43)
#define RTP_AUDIO_OFFSET_U8_ACT        (RTP_AUDIO_OFFSET_U8_MCU)

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

#ifdef __cplusplus
} /* extern "C" */
#endif
