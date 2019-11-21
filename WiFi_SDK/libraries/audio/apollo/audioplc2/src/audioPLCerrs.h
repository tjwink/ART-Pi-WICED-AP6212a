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

/*
 *
 * */

/* Copyright 2015 Broadcom Corporation.  All Rights Reserved. */
/* Author: Rob Zopf                                           */
/* April 27, 2015                                             */

#ifndef AUDIOPLCERRS_H
#define AUDIOPLCERRS_H

#ifdef __cplusplus
extern "C" {
#endif

#define ALLOC_MEM_TRUE 12345  /* a random number to check */
#define INIT_MEM_TRUE  23456

#define AUDIOPLC_NO_ERROR                          0
#define AUDIOPLC_AUDIOPLCSTRUCT_NOT_INIT           1    /* AUDIOPLC_STRUCT not initialized and audioDecoding() called */
#define AUDIOPLC_SAMPLING_RATE_FS_NOT_SUPPORTED    2    /* Combination of sampling rate and frame size not supported */
#define AUDIOPLC_PLC_MALLOC_FAIL                   4
#define AUDIOPLC_FREE_LC_PLC_WITHOUT_INIT          8
#define AUDIOPLC_PLC_STATE_XQ_NULL_FREE            16
#define AUDIOPLC_LCPLCSTRUCT_NOT_INIT              32
#define AUDIOPLC_AUDIOPLCSTRUCT_FREE_NO_INIT       64
#define AUDIOPLC_FREE_NO_ALLOC                     128
#define AUDIOPLC_INIT_NO_ALLOC                     256
#define AUDIOCONCEALMENT_RUN_NO_INIT               512
#define AUDIOPLC_RUN_NO_INIT                       1024
#define SLC_RUN_NO_INIT                            2048
#define AUDIOPLC_MAX_CHAN_EXCEEDED                 4096
#define SLC_UNSTABLE                               8192
#define AUDIOPLC_SL_MAX_EXCEEDED                   16384


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
