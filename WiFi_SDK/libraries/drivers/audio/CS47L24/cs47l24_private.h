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

#include "cs47l24.h"
#include "wiced_utilities.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

#define CS47L24_CHECK_RETRY_COUNT( count, result, label )  \
    do                                                     \
    {                                                      \
        if (loop_count == CS47L24_READ_RETRY_COUNT)        \
        {                                                  \
            result = WICED_ERROR; goto _exit;              \
        }                                                  \
    } while (0)

/*
 * Endianness handling; assuming toolchain that uses __BYTE_ORDER__ macro
 */

#ifndef htonl
    #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        #define htonl(v) (v)
    #else /* __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || __BYTE_ORDER__ == __ORDER_PDP_ENDIAN__ */
        #define htonl(v) WICED_SWAP32(v)
    #endif /* __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ */
#endif /* #ifndef htonl */

#ifndef ntohl
    #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        #define ntohl(v) (v)
    #else /* __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || __BYTE_ORDER__ == __ORDER_PDP_ENDIAN__ */
        #define ntohl(v) WICED_SWAP32(v)
    #endif /* __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ */
#endif /* #ifndef ntohl */

/******************************************************
 *                     Constants
 ******************************************************/

#define CS47L24_READ_RETRY_COUNT (500)

/******************************************************
 *                    Enumerations
 ******************************************************/

typedef enum
{
    CS47L24_SYSCLOCK_6M144_5M6448     = 0x0,
    CS47L24_SYSCLOCK_12M288_11M2896,
    CS47L24_SYSCLOCK_24M576_22M5792,
    CS47L24_SYSCLOCK_49M152_45M1584,         /* That's the max frequency for the ASYNC_SYSCLOCK */
    CS47L24_SYSCLOCK_73M728_67M7376,
    CS47L24_SYSCLOCK_98M304_90M3168,
    CS47L24_SYSCLOCK_147M456_135M4752,
} cs47l24_sysclock_frequency_t;

/******************************************************
 *                  Type Definitions
 ******************************************************/

/******************************************************
 *                     Structures
 ******************************************************/

/* The start addresses of the different memory regions of this core */
typedef struct
{
    /* Memory region start addresses */
    uint32_t                     pm_start;          ///< Start address of program memory
    uint32_t                     xm_start;          ///< Start address of X memory
    uint32_t                     ym_start;          ///< Start address of Y memory
    uint32_t                     zm_start;          ///< Start address of Z memory
    /* Control registers */
    uint32_t                     dsp_control1;      ///< DSP control register
    uint32_t                     dsp_clocking1;     ///< DSP clocking register
    uint32_t                     dsp_status1;       ///< DSP status 1 register
    uint32_t                     dma_config1;       ///< DMA buffer length register
    uint32_t                     wdma_config2;      ///< Write DMA enable register
    uint32_t                     rdma_config1;      ///< Read DMA enable register
    /* Clocking specifics */
    cs47l24_sysclock_frequency_t dsp_clock_request;
} cs47l24_dsp_core_details_t;

/******************************************************
 *                Variable declarations
 ******************************************************/

extern const cs47l24_dsp_core_details_t g_cs47l24_dsp[CS47L24_FIRMWARE_DSP_MAX];

/******************************************************
 *                Function declarations
 ******************************************************/

wiced_result_t cs47l24_reg_write(cs47l24_device_cmn_data_t* cs47l24, uint32_t address, uint16_t reg_data);
wiced_result_t cs47l24_reg_read(cs47l24_device_cmn_data_t* cs47l24, uint32_t address, uint16_t* reg_data);
wiced_result_t cs47l24_upd_bits(cs47l24_device_cmn_data_t *cs47l24, uint32_t reg, uint16_t mask, uint16_t val);
wiced_result_t cs47l24_block_write(cs47l24_device_cmn_data_t* cs47l24, uint8_t* buffer, uint32_t buffer_length);

wiced_result_t cs47l24_process_wmfw_blob(cs47l24_device_cmn_data_t* cs47l24);
wiced_result_t cs47l24_dsp_tuning(cs47l24_device_cmn_data_t* cs47l24);
wiced_result_t cs47l24_dsp_core_reset(cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type);
wiced_result_t cs47l24_dsp_wupd_irq_reset( cs47l24_device_cmn_data_t* cs47l24 );

#ifdef __cplusplus
} /* extern "C" */
#endif
