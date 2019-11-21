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

/******************************************************
 *                    Constants
 ******************************************************/

#define DUKTAPE_COMMANDS \
    { (char*) "duktape_eval",      command_console_duktape_eval,      0, NULL, NULL, (char*) "<string>",    (char*) "Use Duktape to evaluate Ecmascript passed in as string" }, \
    { (char*) "duktape_file",      command_console_duktape_file,      0, NULL, NULL, (char*) "<full_path>", (char*) "Use Duktape to evaluate an Ecmascript from the filesystem" }, \
    { (char*) "duktape_tftp",      command_console_duktape_tftp,      0, NULL, NULL, (char*) "[device]",    (char*) "Start a TFTP server and use Duktape to evaluate the uploaded Ecmascript, and save script to filesystem if device is specified" }, \
    { (char*) "duktape_stop",      command_console_duktape_stop,      0, NULL, NULL, NULL,                  (char*) "Stop any Duktape evaluations" }, \
    { (char*) "duktape_api_test",  command_console_duktape_api_test,  0, NULL, NULL, (char*) "[test_name]", (char*) "Run the specified Duktape API test, else run all tests" }, \

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

int command_console_duktape_eval( int argc, char* argv[] );

int command_console_duktape_file( int argc, char* argv[] );

int command_console_duktape_tftp( int argc, char* argv[] );

int command_console_duktape_stop( int argc, char* argv[] );

int command_console_duktape_api_test( int argc, char* argv[] );

#ifdef __cplusplus
} /*extern "C" */
#endif
