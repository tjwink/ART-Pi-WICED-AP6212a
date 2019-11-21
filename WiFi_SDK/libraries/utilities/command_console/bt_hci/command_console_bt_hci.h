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

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/
#define BT_COMMANDS \
    { "bt_hci_reset",          bt_hci_reset,         0, NULL, NULL, NULL, "Send BT HCI command"}, \
    { "bt_download_firmware",  bt_download_firmware, 0, NULL, NULL, NULL, "Download Firmware"}, \
    { "bt_le_tx_test",         bt_le_tx_test, 0, NULL, NULL, NULL, "Test BT packet transmit test through BLE channel"}, \
    { "bt_le_rx_test",         bt_le_rx_test, 0, NULL, NULL, NULL, "BT packet receive test through BLE channel"}, \
    { "bt_le_test_end",        bt_le_test_end, 0, NULL, NULL, NULL, "Stop BLE transmit/receive test"}, \
    { "bt_radio_tx_test",      bt_radio_tx_test, 0, NULL, NULL, NULL, "Test BT packet transmit test through BR/EDR channel"}, \
    { "bt_radio_rx_test",      bt_radio_rx_test, 0, NULL, NULL, NULL, "BT packet receive test through BR/EDR channel"},

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
int bt_hci_reset( int argc, char* argv[] );
int bt_download_firmware( int argc, char* argv[] );
int bt_le_tx_test( int argc, char* argv[] );
int bt_le_rx_test( int argc, char* argv[] );
int bt_le_test_end( int argc, char* argv[] );
int bt_radio_tx_test( int argc, char* argv[] );
int bt_radio_rx_test( int argc, char* argv[] );

#ifdef __cplusplus
} /* extern "C" */
#endif
