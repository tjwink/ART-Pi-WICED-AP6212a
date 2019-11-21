/*
 * $ Copyright Broadcom Corporation $
 */
#pragma once

#include "wiced_utilities.h"
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t bt_firmware_download( const uint8_t* firmware_image, uint32_t size, const char* version );
wiced_result_t bt_firmware_download_lowrate( const uint8_t* firmware_image, uint32_t size, const char* version );

#ifdef __cplusplus
} /* extern "C" */
#endif
