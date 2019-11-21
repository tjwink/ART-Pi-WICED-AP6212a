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
 * @file globals.h
 *
 */

#ifndef _H_AUDIOPCM_GLOBALS_H_
#define _H_AUDIOPCM_GLOBALS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* general no error value for return function */
#define NO_ERR ( (int8_t) ( 0 ) )
#define TRUE ( (int8_t) ( 1 ) )
#define FALSE ( (int8_t) ( 0 ) )


/* audiopcm control register status flags */
#define APCM_CTRL_ONOFF_STATUS            ( 1 << 0 ) /**< Audiopcm CTRL register bit 0 mask */
#define APCM_CTRL_INPUT_STATUS            ( 1 << 1 ) /**< Audiopcm CTRL register bit 1 mask */
#define APCM_CTRL_RTP_STATUS              ( 1 << 2 ) /**< Audiopcm CTRL register bit 2 mask */
#define APCM_CTRL_DSP_STATUS              ( 1 << 3 ) /**< Audiopcm CTRL register bit 3 mask */
#define APCM_CTRL_OUTPUT_STATUS           ( 1 << 4 ) /**< Audiopcm CTRL register bit 4 mask */
#define APCM_CTRL_CLOCK_STATUS            ( 1 << 5 ) /**< Audiopcm CTRL register bit 5 mask */
#define APCM_CTRL_WATCHDOG_STATUS         ( 1 << 6 ) /**< Audiopcm CTRL register bit 6 mask */
#define APCM_CTRL_AUTORESET_REQUEST       ( 1 << 7 ) /**< Audiopcm CTRL register bit 6 mask */

/* inner components watermarking
 * definitions are GLOBALS to force uniqueness of the WMARKs
 */
#define APCM_WMARK       ( (uint32_t) ( 0xefb0bb1e ) )

/* defines for MEDIA_CLOCK_UNIT (MCU) support */
#define APCM_MEDIA_CLOCK_UNIT_NS              ( 0 )
#define APCM_MEDIA_CLOCK_UNIT_PPB             ( 1 )
#define APCM_MEDIA_CLOCK_UNIT_TK              ( 2 )
#define APCM_MEDIA_CLOCK_UNIT_USR             ( 3 )

#define THREAD_DEFAULT_PRIORITY ( 4 )

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_AUDIOPCM_GLOBALS_H_ */
