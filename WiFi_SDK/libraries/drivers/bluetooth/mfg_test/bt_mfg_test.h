/*
 * $ Copyright Broadcom Corporation $
 */
#pragma once

#include "wiced_utilities.h"

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

wiced_result_t bt_mfgtest_start( const wiced_uart_config_t* config );
wiced_result_t bt_mfgtest_console_start( const wiced_uart_config_t* config );
wiced_result_t bt_mfgtest_console_send_hci( uint8_t *cmd, uint8_t len, \
                                            uint8_t *res, uint8_t res_len );
wiced_result_t bt_mfgtest_console_download_fw();
wiced_result_t bt_mfgtest_console_receive_hci( uint8_t *read_data, uint8_t len);

#ifdef __cplusplus
} /* extern "C" */
#endif
