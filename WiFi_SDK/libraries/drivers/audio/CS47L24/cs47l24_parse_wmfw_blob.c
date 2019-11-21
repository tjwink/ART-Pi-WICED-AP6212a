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
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2014-2015 Cirrus Logic International (UK) Ltd.  All rights reserved.
//
// This software as well as any related documentation is furnished under
// license and may only be used or copied in accordance with the terms of the
// license. The information in this file is furnished for informational use
// only, is subject to change without notice, and should not be construed as
// a commitment by Cirrus Logic International (UK) Ltd.  Cirrus Logic International
// (UK) Ltd assumes no responsibility or liability for any errors or inaccuracies
// that may appear in this document or any software that may be provided in
// association with this document.
//
// Except as permitted by such license, no part of this document may be
// reproduced, stored in a retrieval system, or transmitted in any form or by
// any means without the express written consent of Cirrus Logic International
// (UK) Ltd or affiliated companies.
//
////////////////////////////////////////////////////////////////////////////////

/**
 * @file Parses a WMFW file/blob for loading to Cirrus CS47L24 DSPs
 */

#include <stdlib.h>
#include <string.h>

#include "wiced_log.h"
#include "wiced_rtos.h"

#include "cs47l24.h"
#include "cs47l24_private.h"
#include "cs47l24_register_map.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/
/******************************************************
 *                    Constants
 ******************************************************/

#define CS47L24_REG_ADDR_LENGTH              (sizeof(uint32_t))
#define CS47L24_PADDING_LENGTH               (sizeof(uint16_t))
#define CS47L24_REG_ADDR_PLUS_PADDING_LENGTH (CS47L24_REG_ADDR_LENGTH + CS47L24_PADDING_LENGTH)

/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WMFW_PM                   = 0x02,                ///< ADSPx program memory
    WMFW_DM                   = 0x03,                ///< ADSP1 data memory
    WMFW_XM                   = 0x05,                ///< ADSP2 X coefficient memory
    WMFW_YM                   = 0x06,                ///< ADSP2 Y coefficient memory
    WMFW_ZM                   = 0x04,                ///< ADSP1 & ADSP2 Z coefficient memory
    WMFW_ABS_ADDRESS          = 0xF0,                ///< Absolute addressing for the device
    WMFW_REL_ADDRESS          = 0xF1,                ///< Relative addressing from the start of the first coefficients block
    WMFW_ALGO_INFO_BLOCK      = 0xF2,                ///< Algorithm information data block
    WMFW_COEFF_INFO_SUB_BLOCK = 0xF3,                ///< Coefficient information data sub-block
    WMFW_USER_TEXT            = 0xFE,                ///< User-defined name text
    WMFW_INFO_STRING          = 0xFF,                ///< Informational ASCII string
} wmfw_region_t;

/******************************************************
 *                    Structures
 ******************************************************/
/*
 * Structure defining the header of a WMFW block
 *
 * Format for a block:
 *
 *      31       24 23      16 15       8 7        0
 *  0   +----------+----------+----------+---------+
 *      | type[7:0]|          offset[23:0]         |
 *  4   +----------+-------------------------------+
 *      |                dataLength                |
 *  8   +------------------------------------------+
 *      |                   data                   |
 *      :                   ....                   :
 *      :                                          :
 *
 * The offset/region and data length are little endian.
 * The data is formatted big-endian to facilitate writing straight to the core.
 */
typedef struct
{
    uint32_t offset : 24;
    uint32_t region : 8;
    uint32_t data_length;
} wmfw_block_header_t;

typedef struct
{
    uint32_t reg;
    uint16_t value;
} cs47l24_dsp_register_value_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const char      g_wmfw_id_string[] = "WMFW";
static const uint32_t  g_wmfw_id_length   = 4;

static cs47l24_dsp_register_value_t g_cs47l24_dsp2_settings_table[] =
{
   { 0x200000, 0x0040 }, // DSP2 PM 0(200000H):      0040  DSP2_PM_START=0100_0000
   { 0x200001, 0x0118 }, // DSP2 PM 1(200001H):      0118  DSP2_PM_START=0000_0001_0001_1000
   { 0x200002, 0x0003 }, // DSP2 PM 2(200002H):      0003  DSP2_PM_START=0000_0000_0000_0011
   { 0x205FFD, 0x009A }, // DSP2 PM 24573(205FFDH):  009A  DSP2_PM_END=1001_1010
   { 0x205FFE, 0x9E35 }, // DSP2 PM 24574(205FFEH):  9E35  DSP2_PM_END=1001_1110_0011_0101
   { 0x205FFF, 0xA9CD }, // DSP2 PM 24575(205FFFH):  A9CD  DSP2_PM_END=1010_1001_1100_1101
   { 0x280000, 0x0070 }, // FIRMWARE_LARGO_EZ2CONTROL_MAX_SYSTEM_GAIN_1(280000H): 0070  FIRMWARE_LARGO_EZ2CONTROL_MAX_SYSTEM_GAIN=0111_0000
   { 0x280003, 0x0010 }, // FIRMWARE_LARGO_EZ2CONTROL_BITS_PER_SAMPLE_0(280003H): 0010  FIRMWARE_LARGO_EZ2CONTROL_BITS_PER_SAMPLE=10
   { 0x280005, 0x0001 }, // FIRMWARE_LARGO_EZ2CONTROL_APPLY_COMPRESSION_0(280005H): 0001  FIRMWARE_LARGO_EZ2CONTROL_APPLY_COMPRESSION=1

   { 0x280008, 0x0000 }, // FIRMWARE_LARGO_EZ2CONTROL_BUFFERTRIGGERONLY_1(280008): 0000
   { 0x280009, 0x0001 }, // FIRMWARE_LARGO_EZ2CONTROL_BUFFERTRIGGERONLY_1(280009): 0001  FIRMWARE_LARGO_EZ2CONTROL_BUFFERTRIGGERONLY=1

   { 0x28000B, 0x0002 }, // FIRMWARE_LARGO_EZ2CONTROL_CMD_TIMEOUT_0(28000BH): 0002  FIRMWARE_LARGO_EZ2CONTROL_CMD_TIMEOUT=2
   { 0x28000D, 0x8000 }, // SENSORY_ALGORITHM_SENSORY_ALG_GAIN_0(28000DH): 8000  SENSORY_ALGORITHM_SENSORY_ALG_GAIN=1000_0000_0000_0000
   { 0x28000F, 0x0001 }, // SENSORY_ALGORITHM_DCBLOCKENABLE_0(28000FH): 0001  SENSORY_ALGORITHM_DCBLOCKENABLE=1
   { 0x280011, 0x0001 }, // SENSORY_ALGORITHM_LPSDENABLE_0(280011H): 0001  SENSORY_ALGORITHM_LPSDENABLE=1
   { 0x280015, 0x0001 }, // SENSORY_ALGORITHM_SVTHRESHOLDOVERRIDE_0(280015H): 0001  SENSORY_ALGORITHM_SVTHRESHOLDOVERRIDE=1
   { 0x280019, 0x0001 }, // SENSORY_ALGORITHM_PARAMAOVERRIDE_0(280019H): 0001  SENSORY_ALGORITHM_PARAMAOVERRIDE=1
   { 0x280021, 0x04B0 }, // SENSORY_ALGORITHM_MAXFIXEDTHRESHOLD_0(280021H): 04B0  SENSORY_ALGORITHM_MAXFIXEDTHRESHOLD=4B0
   { 0x280023, 0x0580 }, // SENSORY_ALGORITHM_TFACTOR_Q10_0(280023H): 0580  SENSORY_ALGORITHM_TFACTOR_Q10=580
   { 0x280029, 0x0001 }, // SENSORY_ALGORITHM_INCLUDETRIGGERPHRASE_0(280029H): 0001  SENSORY_ALGORITHM_INCLUDETRIGGERPHRASE=1
   { 0x28002D, 0x00FA }, // SENSORY_ALGORITHM_SENSORY_PID_NOISETHRESHOLD_0(28002DH): 00FA  SENSORY_ALGORITHM_SENSORY_PID_NOISETHRESHOLD=FA
   { 0x281FFE, 0x0094 }, // DSP2 ZM 8190(281FFEH):   0094  DSP2_ZM_END=1001_0100
   { 0x281FFF, 0xAF66 }, // DSP2 ZM 8191(281FFFH):   AF66  DSP2_ZM_END=1010_1111_0110_0110
   { 0x290000, 0x0002 }, // DSP2_DATA_HEADER_COREID_1(290000H): 0002  DSP2_DATA_HEADER_COREID=2
   { 0x290003, 0x0501 }, // DSP2_DATA_HEADER_COREREVISION_0(290003H): 0501  DSP2_DATA_HEADER_COREREVISION=501
   { 0x290004, 0x0007 }, // DSP2_DATA_HEADER_FIRMWAREID_1(290004H): 0007  DSP2_DATA_HEADER_FIRMWAREID=7
   { 0x290005, 0x000D }, // DSP2_DATA_HEADER_FIRMWAREID_0(290005H): 000D  DSP2_DATA_HEADER_FIRMWAREID=D
   { 0x290006, 0x0001 }, // DSP2_DATA_HEADER_FIRMWAREREVISION_1(290006H): 0001  DSP2_DATA_HEADER_FIRMWAREREVISION=1
   { 0x290007, 0x0700 }, // DSP2_DATA_HEADER_FIRMWAREREVISION_0(290007H): 0700  DSP2_DATA_HEADER_FIRMWAREREVISION=700
   { 0x29000B, 0x000E }, // DSP2_DATA_HEADER_FIRMWAREXMBASE_0(29000BH): 000E  DSP2_DATA_HEADER_FIRMWAREXMBASE=E
   { 0x29000D, 0xFFFF }, // DSP2_DATA_HEADER_FIRMWAREYMBASE_0(29000DH): FFFF  DSP2_DATA_HEADER_FIRMWAREYMBASE=FFFF
   { 0x29000F, 0x0001 }, // DSP2_DATA_HEADER_ALGORITHMCOUNT_0(29000FH): 0001  DSP2_DATA_HEADER_ALGORITHMCOUNT=1
   { 0x290011, 0x0024 }, // DSP2_ALGORITHM_HEADER1_ID_0(290011H): 0024  DSP2_ALGORITHM_HEADER1_ID=24
   { 0x290013, 0x130C }, // DSP2_ALGORITHM_HEADER1_REVISION_0(290013H): 130C  DSP2_ALGORITHM_HEADER1_REVISION=130C
   { 0x290015, 0x0006 }, // DSP2_ALGORITHM_HEADER1_ZMBASE_0(290015H): 0006  DSP2_ALGORITHM_HEADER1_ZMBASE=6
   { 0x290017, 0x0038 }, // DSP2_ALGORITHM_HEADER1_XMBASE_0(290017H): 0038  DSP2_ALGORITHM_HEADER1_XMBASE=38
   { 0x29001A, 0x00BE }, // DSP2_ALGORITHM_LIST_TERMINATOR_1(29001AH): 00BE  DSP2_ALGORITHM_LIST_TERMINATOR=BE
   { 0x29001B, 0xDEAD }, // DSP2_ALGORITHM_LIST_TERMINATOR_0(29001BH): DEAD  DSP2_ALGORITHM_LIST_TERMINATOR=DEAD
   { 0x29001D, 0x0001 }, // FIRMWARE_LARGO_EZ2CONTROL_SYS_ENABLE_0(29001DH): 0001  FIRMWARE_LARGO_EZ2CONTROL_SYS_ENABLE=1
   { 0x29001E, 0x0007 }, // FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_ID_1(29001EH): 0007  FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_ID=7
   { 0x29001F, 0x000D }, // FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_ID_0(29001FH): 000D  FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_ID=D
   { 0x290020, 0x0001 }, // FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_VRS_1(290020H): 0001  FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_VRS=1
   { 0x290021, 0x0700 }, // FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_VRS_0(290021H): 0700  FIRMWARE_LARGO_EZ2CONTROL_PLUGIN_FMW_VRS=700
   { 0x290023, 0x0003 }, // FIRMWARE_LARGO_EZ2CONTROL_ADSP2_STATE_0(290023H): 0003  FIRMWARE_LARGO_EZ2CONTROL_ADSP2_STATE=3
   { 0x290025, 0x4783 }, // FIRMWARE_LARGO_EZ2CONTROL_ADSP2_WATCHDOG_0(290025H): 4783  FIRMWARE_LARGO_EZ2CONTROL_ADSP2_WATCHDOG=4783
   { 0x290027, 0x00F0 }, // FIRMWARE_LARGO_EZ2CONTROL_DMA_BUFFER_SIZE_0(290027H): 00F0  FIRMWARE_LARGO_EZ2CONTROL_DMA_BUFFER_SIZE=F0
   { 0x290029, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE1_0(290029H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE1=FFFF
   { 0x29002B, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE2_0(29002BH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE2=FFFF
   { 0x29002D, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE3_0(29002DH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE3=FFFF
   { 0x29002F, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE4_0(29002FH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE4=FFFF
   { 0x290031, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE5_0(290031H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE5=FFFF
   { 0x290033, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE6_0(290033H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_RDMA_CONTROL_TABLE6=FFFF
   { 0x290035, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE1_0(290035H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE1=FFFF
   { 0x290037, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE2_0(290037H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE2=FFFF
   { 0x290039, 0x04DE }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE3_0(290039H): 04DE  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE3=4DE
   { 0x29003B, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE4_0(29003BH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE4=FFFF
   { 0x29003D, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE5_0(29003DH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE5=FFFF
   { 0x29003F, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE6_0(29003FH): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE6=FFFF
   { 0x290041, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE7_0(290041H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE7=FFFF
   { 0x290043, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE8_0(290043H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_WDMA_CONTROL_TABLE8=FFFF
   { 0x290044, 0x0045 }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME1_1(290044H): 0045  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME1=45
   { 0x290045, 0x7A32 }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME1_0(290045H): 7A32  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME1=7A32
   { 0x290046, 0x0043 }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME2_1(290046H): 0043  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME2=43
   { 0x290047, 0x7472 }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME2_0(290047H): 7472  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME2=7472
   { 0x290048, 0x006C }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME3_1(290048H): 006C  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME3=6C
   { 0x290049, 0x5F5F }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME3_0(290049H): 5F5F  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NAME3=5F5F
   { 0x29004B, 0x0018 }, // FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NUMBER_0(29004BH): 0018  FIRMWARE_LARGO_EZ2CONTROL_BUILD_JOB_NUMBER=18
   { 0x29004C, 0x0049 }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_MAGIC_1(29004CH): 0049  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_MAGIC=0100_1001
   { 0x29004D, 0xAEC7 }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_MAGIC_0(29004DH): AEC7  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_MAGIC=1010_1110_1100_0111
   { 0x29004F, 0x051F }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_SMOOTHING_0(29004FH): 051F  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_SMOOTHING=0000_0101_0001_1111
   { 0x290051, 0x0010 }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_THRESHOLD_0(290051H): 0010  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_THRESHOLD=0000_0000_0001_0000
   { 0x290056, 0x00FF }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_HIGH_WATER_MARK_1(290056H): 00FF  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_HIGH_WATER_MARK=FF
   { 0x290057, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_HIGH_WATER_MARK_0(290057H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_HIGH_WATER_MARK=FFFF
   { 0x290058, 0x00FF }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_LOW_WATER_MARK_1(290058H): 00FF  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_LOW_WATER_MARK=FF
   { 0x290059, 0xFFFF }, // FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_LOW_WATER_MARK_0(290059H): FFFF  FIRMWARE_LARGO_EZ2CONTROL_COMPRESSION_BUF_HDR_LOW_WATER_MARK=FFFF
   { 0x29005D, 0x0002 }, // FIRMWARE_LARGO_EZ2CONTROL_TRIGGER_ID_0(29005DH): 0002  FIRMWARE_LARGO_EZ2CONTROL_TRIGGER_ID=2
   { 0x29005F, 0x08F3 }, // FIRMWARE_LARGO_EZ2CONTROL_TRIGGER_SCORE_0(29005FH): 08F3  FIRMWARE_LARGO_EZ2CONTROL_TRIGGER_SCORE=8F3

   { 0x290062, 0x0000 }, // FIRMWARE_LARGO_EZ2CONTROL_STREAMING_TIMEOUT_0(290062H): 0000
   { 0x290063, 0x0002 }, // FIRMWARE_LARGO_EZ2CONTROL_STREAMING_TIMEOUT_0(290063H): 0002  FIRMWARE_LARGO_EZ2CONTROL_STREAMING_TIMEOUT=2

   { 0x290065, 0x0046 }, // FIRMWARE_LARGO_EZ2CONTROL_LOWDSPCLKSPEED_0(290065H): 0046  FIRMWARE_LARGO_EZ2CONTROL_LOWDSPCLKSPEED=46
   { 0x290067, 0x0046 }, // FIRMWARE_LARGO_EZ2CONTROL_HIGHDSPCLKSPEED_0(290067H): 0046  FIRMWARE_LARGO_EZ2CONTROL_HIGHDSPCLKSPEED=46

   { 0x290068, 0x0000 }, // FIRMWARE_LARGO_EZ2CONTROL_INTERRUPT_AND_STREAM_0(290068H): 0000
   { 0x290069, 0x0000 }, // FIRMWARE_LARGO_EZ2CONTROL_INTERRUPT_AND_STREAM_0(290069H): 0000  FIRMWARE_LARGO_EZ2CONTROL_INTERRUPT_AND_STREAM=0

   { 0x29006B, 0x0028 }, // FIRMWARE_LARGO_EZ2CONTROL_SADDEBOUNCEVALUE_0(29006BH): 0028  FIRMWARE_LARGO_EZ2CONTROL_SADDEBOUNCEVALUE=28
   { 0x29006D, 0x0002 }, // FIRMWARE_LARGO_EZ2CONTROL_GPIO_TRIGGER_INDICATOR_0(29006DH): 0002  FIRMWARE_LARGO_EZ2CONTROL_GPIO_TRIGGER_INDICATOR=2
   { 0x290071, 0x0001 }, // SENSORY_ALGORITHM_SENSORY_ALGORITHM_ENABLE_0(290071H): 0001  SENSORY_ALGORITHM_SENSORY_ALGORITHM_ENABLE=1
   { 0x2A7FFE, 0x00FF }, // DSP2 XM EXT 8190(2A7FFEH): 00FF  DSP2_XM_EXT_END=FF
   { 0x2A7FFF, 0x6B34 }, // DSP2 XM EXT 8191(2A7FFFH): 6B34  DSP2_XM_EXT_END=6B34
   { 0x2A8001, 0x0026 }, // SENSORY_ALGORITHM_GRAMMAR_DATA_PTR_0(2A8001H): 0026  SENSORY_ALGORITHM_GRAMMAR_DATA_PTR=0000_0000_0010_0110
   { 0x2B3FFE, 0x002E }, // DSP2 YM 49150(2B3FFEH):  002E  DSP2_YM_END=0010_1110
   { 0x2B3FFF, 0x877C }, // DSP2 YM 49151(2B3FFFH):  877C  DSP2_YM_END=1000_0111_0111_1100
};

static cs47l24_dsp_register_value_t g_cs47l24_dsp3_settings_table[] =
{
   { 0x3A8020, 0x0000 }, // SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_100000800
   { 0x3A8021, 0x0800 },
   { 0x3A8004, 0x0000 }, // SC_FF_DUET_WRITEREGID_100000003
   { 0x3A8005, 0x0003 },
   { 0x3A8024, 0x0000 }, // SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_100001000
   { 0x3A8025, 0x1000 },
   { 0x3A8004, 0x0000 }, // SC_FF_DUET_WRITEREGID_100000003
   { 0x3A8005, 0x0003 },
};

const cs47l24_dsp_core_details_t g_cs47l24_dsp[CS47L24_FIRMWARE_DSP_MAX] =
{
    {
        CS47L24_DSP2_PM_START,       // PM start
        CS47L24_DSP2_XM_START,       // XM start
        CS47L24_DSP2_YM_START,       // YM start
        CS47L24_DSP2_ZM_START,       // ZM start
        CS47L24_DSP2_CONTROL_1,      // DSP2 control 1
        CS47L24_DSP2_CLOCKING_1,     // DSP2 clocking 1
        CS47L24_DSP2_STATUS_1,       // DSP2 status 1
        CS47L24_DSP2_WDMA_CONFIG_1,  // DSP2 WDMA Config 1 (DMA buffer length)
        CS47L24_DSP2_WDMA_CONFIG_2,  // DSP2 WDMA Config 2
        CS47L24_DSP2_RDMA_CONFIG_1,  // DSP2 RDMA Config 1
        CS47L24_SYSCLOCK_73M728_67M7376,
    },
    {
        CS47L24_DSP3_PM_START,       // PM start
        CS47L24_DSP3_XM_START,       // XM start
        CS47L24_DSP3_YM_START,       // YM start
        CS47L24_DSP3_ZM_START,       // ZM start
        CS47L24_DSP3_CONTROL_1,      // DSP3 control 1
        CS47L24_DSP3_CLOCKING_1,     // DSP3 clocking 1
        CS47L24_DSP3_STATUS_1,       // DSP3 status 1
        CS47L24_DSP3_WDMA_CONFIG_1,  // DSP3 WDMA Config 1 (DMA buffer length)
        CS47L24_DSP3_WDMA_CONFIG_2,  // DSP3 WDMA Config 2
        CS47L24_DSP3_RDMA_CONFIG_1,  // DSP3 RDMA Config 1
        CS47L24_SYSCLOCK_147M456_135M4752,
    },
};

/******************************************************
 *            Static Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t cs47l24_dsp_core_reset(cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Disable DSP core first */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, 0), result, _exit);
    /* Re-enable core */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, CS47L24_DSP_CORE_ENA), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_core_reset: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_prepare_core_for_upload(cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type)
{
    wiced_result_t result     = WICED_SUCCESS;
    uint32_t       loop_count;
    uint16_t       ram_ready;

    /*
     * SYSCLK must be up and running at this point !
     */

    /* Enable DSP core clocking */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, CS47L24_DSP_SYS_ENA), result, _exit);
    /* Make sure DSP memory is disabled */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_MEM_ENA_MASK, 0), result, _exit);
    /* Disable/Re-enable DSP core clocking */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, 0), result, _exit);
    wiced_rtos_delay_milliseconds(1);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, CS47L24_DSP_SYS_ENA), result, _exit);
    /* Disable DSP core first */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, 0), result, _exit);
    /* Clear out the DMA length. It will be re-initialized by the new firmware */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dma_config1, CS47L24_DSP_WDMA_BUFFER_LENGTH_MASK, 0), result, _exit);
    /* Disable the DMA channels */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].wdma_config2, CS47L24_DSP_WDMA_CHANNEL_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].rdma_config1, CS47L24_DSP_RDMA_CHANNEL_ENA_MASK, 0), result, _exit);
    /*
     * Check for RAM readiness
     */
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, g_cs47l24_dsp[dsp_type].dsp_status1, &ram_ready), result, _exit );
    loop_count = 0;
    while (((ram_ready & CS47L24_DSP_RAM_RDY_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &ram_ready), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_prepare_core_for_upload: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_process_wmfw_header( uint8_t* wmfw_buffer, uint32_t wmfw_buffer_length, uint32_t* wmfw_buffer_index )
{
    wiced_result_t result        = WICED_SUCCESS;
    uint32_t       length;       /* Header length */

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < g_wmfw_id_length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to read magic marker !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /* Check for {'W','M','F','W'} magic marker */
    if ( memcmp(g_wmfw_id_string, &wmfw_buffer[*wmfw_buffer_index], g_wmfw_id_length) != 0)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: WMFW not found at the start of the file [%c%c%c%c]!\n",
                      wmfw_buffer[*wmfw_buffer_index], wmfw_buffer[*wmfw_buffer_index + 1], wmfw_buffer[*wmfw_buffer_index + 2], wmfw_buffer[*wmfw_buffer_index + 3]);
        result = WICED_ERROR;
        goto _done;
    }

    /* Skip magic marker */
    *wmfw_buffer_index += g_wmfw_id_length;

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < sizeof(length)) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to read header length !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /*
     * Read header length
     */

    length = WICED_READ_32(&wmfw_buffer[*wmfw_buffer_index]);
    *wmfw_buffer_index += sizeof(length);

    /*
     * Now skip the rest of the header.
     * This is in bytes and includes the magic and length, so we need to subtract 8.
     */

    length -= (g_wmfw_id_length + sizeof(length));

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to skip header !\n");
        result = WICED_ERROR;
        goto _done;
    }

    *wmfw_buffer_index += length;

    /* And we're done. */

 _done:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: failed !\n");
    }
    return result;
}


/*
 * This function assumes the buffer is pre-formatted for an auto-incrementing
 * write to the registers across the control bus (e.g. I2C or SPI).
 */
static wiced_result_t cs47l24_write_data_to_dsp( cs47l24_device_cmn_data_t* cs47l24, uint8_t* buffer, uint32_t start_address, uint32_t length_in_bytes )
{
    wiced_result_t result      = WICED_SUCCESS;
    uint32_t       i           = 0;
    uint32_t       reg_address = start_address;

    /*
     * Check we have an even number of bytes - i.e. a full number of registers
     */
    if ( length_in_bytes & 1 )
    {
        /*
         * We have an odd number of bytes, so drop the half register.
         */
        wiced_log_msg(WLF_DRIVER, WICED_LOG_WARNING, "cs47l24_write_data_to_dsp: dropping odd byte\n" );
        length_in_bytes &= ~1;
    }

    /*
     * We're "making room" to setup an uninterrupted transfer over SPI.
     * The gist of it is that the device will start auto-incrementing the register address
     * after the following sequence: 4-byte of register start address + 2-byte padding + 2-byte of actual data
     * After this initial 8-byte sequence, the device will auto-increment register address with every 2-bytes of data.
     * See the "control interface" section of the datasheet for details.
     * Problem is our contiguous buffer only contains data; no register address and no padding.
     * So, we have to "free up" the first 6 bytes in order to write the register address followed by 2-bytes of padding
     */
    for (i = 0; i < MIN(CS47L24_REG_ADDR_PLUS_PADDING_LENGTH, length_in_bytes); i += 2)
    {
        uint16_t reg_data = 0;

        reg_data = (buffer[i] << 8) | buffer[i+1];
        WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, reg_address, reg_data), result, _exit );

        reg_address++;
    }

    if (length_in_bytes <= CS47L24_REG_ADDR_PLUS_PADDING_LENGTH)
    {
        /*
         * That's it; we're done here
         */
        goto _exit;
    }

    /*
     * 1. Write 4-byte register address in network / BE byte order
     * 2. Add 2-byte padding
     * 3. Start transfer
     */
    reg_address = htonl(reg_address);
    memcpy(buffer, &reg_address, sizeof(reg_address));
    memset(&buffer[sizeof(reg_address)], 0, sizeof(uint16_t));

    WICED_VERIFY_GOTO( cs47l24_block_write(cs47l24, buffer, length_in_bytes), result, _exit );

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_write_data_to_dsp: failed !\n");
    }
    return result;
}


/*
 * Processes a block of the WMFW file and writes it to the DSP if appropriate
 */
static wiced_result_t cs47l24_process_next_wmfw_block( cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type,
                                                       uint8_t*  wmfw_buffer, uint32_t  wmfw_buffer_length, uint32_t* wmfw_buffer_index )
{
    wiced_result_t      result                = WICED_SUCCESS;
    wmfw_block_header_t block_header          = { 0 };
    uint32_t            region_start;
    uint32_t            registers_per_address;
    uint32_t            offset_in_registers;
    uint32_t            start_address;

    /*
     * Format for a block:
     *
     *      31       24 23      16 15       8 7        0
     *  0   +----------+----------+----------+---------+
     *      | type[7:0]|          offset[23:0]         |
     *  4   +----------+-------------------------------+
     *      |                dataLength                |
     *  8   +------------------------------------------+
     *      |                   data                   |
     *      :                   ....                   :
     *      :                                          :
     *
     * The offset/region and data length are little endian.
     * The data is formatted big-endian to facilitate writing straight to the core.
     */

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < sizeof(block_header)) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to read WMFW block header !\n");
        result = WICED_ERROR;
        goto _done;
    }

    memcpy(&block_header, &wmfw_buffer[*wmfw_buffer_index], sizeof(block_header));
    *wmfw_buffer_index += sizeof(block_header);

    /*
     * Check it's a block we understand
     */
    switch (block_header.region)
    {
        case WMFW_PM:
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Program data\n");
            region_start = g_cs47l24_dsp[dsp_type].pm_start;
            registers_per_address = 3;
            break;

        case WMFW_XM:
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: XM region data\n");
            region_start = g_cs47l24_dsp[dsp_type].xm_start;
            registers_per_address = 2;
            break;

        case WMFW_YM:
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: YM region data\n");
            region_start = g_cs47l24_dsp[dsp_type].ym_start;
            registers_per_address = 2;
            break;

        case WMFW_ZM:
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: ZM region data\n");
            region_start = g_cs47l24_dsp[dsp_type].zm_start;
            registers_per_address = 2;
            break;
        case WMFW_ABS_ADDRESS:
            if (block_header.region == WMFW_ABS_ADDRESS)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Absolute addressing\n");
            }
            /* And fall through */
        case WMFW_REL_ADDRESS:
            if (block_header.region == WMFW_REL_ADDRESS)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Relative addressing from first ZM block\n");
            }
            /* And fall through */
        case WMFW_ALGO_INFO_BLOCK:
            if (block_header.region == WMFW_ALGO_INFO_BLOCK)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Algorithm information data block\n");
            }
            /* And fall through */
        case WMFW_COEFF_INFO_SUB_BLOCK:
            if (block_header.region == WMFW_COEFF_INFO_SUB_BLOCK)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Coefficient information data sub-block\n");
            }
            /* And fall through */
        case WMFW_USER_TEXT:
            if (block_header.region == WMFW_USER_TEXT)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: User-defined name text\n");
            }
            /* And fall through */
        case WMFW_INFO_STRING:
            if (block_header.region == WMFW_INFO_STRING)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: It's an info string\n");
            }
            /* And fall through */
        default: /* Unknown block ! */
            /*
             * Skip over the rest of the block
             */
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Skipping over %lu bytes to end of 0x%X block\n", block_header.data_length, block_header.region);
            if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < block_header.data_length) )
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to skip WMFW block header data !\n");
                result = WICED_ERROR;
            }
            else
            {
                *wmfw_buffer_index += block_header.data_length;
            }
            /*
             * And move on
             */
            goto _done;
    }

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < block_header.data_length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to consume WMFW block header data !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /*
     * Work out where to write data to
     */
    offset_in_registers = block_header.offset * registers_per_address;
    start_address       = region_start + offset_in_registers;

    /*
     * And write the data.
     */
    WICED_VERIFY_GOTO( cs47l24_write_data_to_dsp(cs47l24, &wmfw_buffer[*wmfw_buffer_index], start_address, block_header.data_length), result, _done );

    /*
     * We've finished this block.  Go round for the next one
     */

    *wmfw_buffer_index += block_header.data_length;

 _done:
    if (*wmfw_buffer_index >= wmfw_buffer_length)
    {
        result = WICED_PENDING;
    }
    if ((result != WICED_SUCCESS) && (result != WICED_PENDING))
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: failed !\n");
    }
    return result;
}


/*
 * This function reads in a WMFW blob, processes it, and sends it block by block to the DSP
 */
wiced_result_t cs47l24_process_wmfw_blob( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       index;
    uint8_t*       buffer;
    uint32_t       buffer_size;
    uint32_t       buffer_index;
    uint32_t       j;

    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "cs47l24_process_wmfw_blob: begins...\n");

    for (index = 0; index < CS47L24_FIRMWARE_DSP_MAX; index++)
    {
        if (cs47l24->dsp->dsp_res_table[index] == NULL)
        {
            continue;
        }

        buffer       = NULL;
        buffer_size  = 0;
        buffer_index = 0;

        if (resource_get_readonly_buffer(cs47l24->dsp->dsp_res_table[index], 0, cs47l24->dsp->dsp_res_table[index]->size, &buffer_size, (const void**)&buffer) != RESOURCE_SUCCESS)
        {
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_blob: resource_get_readonly_buffer() failed\n");
            result = WICED_ERROR;
            goto _exit;
        }

        /* Prepare the core for upload */
        WICED_VERIFY_GOTO( cs47l24_prepare_core_for_upload(cs47l24, index), result, _exit );

        /* Read and process the header */
        WICED_VERIFY_GOTO( cs47l24_process_wmfw_header(buffer, buffer_size, &buffer_index), result, _exit );

        /* Now process all the blocks in the buffer */
        do
        {
            result = cs47l24_process_next_wmfw_block(cs47l24, index, buffer, buffer_size, &buffer_index);
        } while (result == WICED_SUCCESS);

        resource_free_readonly_buffer(cs47l24->dsp->dsp_res_table[index], buffer);
        buffer       = NULL;
        buffer_size  = 0;
        buffer_index = 0;

        if ((result != WICED_SUCCESS) && (result != WICED_PENDING))
        {
            goto _exit;
        }

        /*
         * Now the firmware image is downloaded;
         * send any coefficients and parameters ready for starting
         */

        if (index == CS47L24_FIRMWARE_DSP2)
        {
            for (j = 0; j < ARRAY_SIZE(g_cs47l24_dsp2_settings_table); j++)
            {
                WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, g_cs47l24_dsp2_settings_table[j].reg, g_cs47l24_dsp2_settings_table[j].value), result, _exit );
            }
        }
        else if (index == CS47L24_FIRMWARE_DSP3)
        {
            for (j = 0; j < ARRAY_SIZE(g_cs47l24_dsp3_settings_table); j++)
            {
                WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, g_cs47l24_dsp3_settings_table[j].reg, g_cs47l24_dsp3_settings_table[j].value), result, _exit );
            }
        }

        /*
         * Set the core going again.
         * This will send a DSP_START to the core and get everything up and running.
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[index].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, CS47L24_DSP_CORE_ENA), result, _exit);
        /*
         * Make sure DSP memory persists across software/hardware resets
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[index].dsp_control1, CS47L24_DSP_MEM_ENA_MASK, CS47L24_DSP_MEM_ENA), result, _exit);
    }

 _exit:
    if ((cs47l24->dsp->dsp_res_table[index] != NULL) && (buffer != NULL))
    {
        resource_free_readonly_buffer(cs47l24->dsp->dsp_res_table[index], buffer);
        buffer       = NULL;
        buffer_size  = 0;
        buffer_index = 0;
    }

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "cs47l24_process_wmfw_blob: ...done.\n");
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_blob: failed\n");
    }
    return result;
}


wiced_result_t cs47l24_dsp_tuning( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       j;

    for (j = 0; j < ARRAY_SIZE(g_cs47l24_dsp2_settings_table); j++)
    {
        WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, g_cs47l24_dsp2_settings_table[j].reg, g_cs47l24_dsp2_settings_table[j].value), result, _exit );
    }

    WICED_VERIFY_GOTO( cs47l24_dsp_core_reset(cs47l24, CS47L24_FIRMWARE_DSP2),  result, _exit );

    for (j = 0; j < ARRAY_SIZE(g_cs47l24_dsp3_settings_table); j++)
    {
        WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, g_cs47l24_dsp3_settings_table[j].reg, g_cs47l24_dsp3_settings_table[j].value), result, _exit );
    }

  _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_tuning: failed !\n");
    }
    return result;
}


wiced_result_t cs47l24_dsp_wupd_irq_reset( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result     = WICED_SUCCESS;
    uint16_t       irq_status;

    /*
     * DSP2 signals WUPD on DSP_IRQ1 as well as on GP2;
     * however, DSP_IRQ1 needs to be (re)set to 1 after successful keyword trigger
     */
    WICED_VERIFY_GOTO( cs47l24_reg_read (cs47l24, CS47L24_ADSP2_IRQ0, &irq_status), result, _exit );
    WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, CS47L24_ADSP2_IRQ0, (irq_status & ~CS47L24_DSP_IRQ1_MASK) | CS47L24_DSP_IRQ1), result, _exit );

 _exit:
    return result;
}
