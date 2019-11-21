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

#if defined(TX_ENABLE_EVENT_TRACE)
#include "TraceX.h"
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#if defined(TX_ENABLE_EVENT_TRACE)
#define TRACEX_COMMANDS \
    { (char*) "tracex_enable",  command_console_tracex_enable,  0, NULL, NULL, (char*) "[-l] [-t <ip_addr>] [-p <port>] [-d <len>]",                                      (char*) "Enable TraceX; -l to enable loop-recording; -t, -p, and -d to specify TCP server IP address, port, and max data packet length for sending the TraceX buffer."}, \
    { (char*) "tracex_disable", command_console_tracex_disable, 0, NULL, NULL, NULL,                                                                                      (char*) "Disable TraceX and send current event buffer trace to the TCP server if specified."}, \
    { (char*) "tracex_restart", command_console_tracex_restart, 0, NULL, NULL, NULL,                                                                                      (char*) "Restart TraceX with same configuration."}, \
    { (char*) "tracex_status",  command_console_tracex_status,  0, NULL, NULL, NULL,                                                                                      (char*) "Print status of TraceX."}, \
    { (char*) "tracex_filter",  command_console_tracex_filter,  0, NULL, NULL, (char*) "[-l] [-a] [-c] [-d] [-f <filters...>] [-F <mask>] [-u <filters...>] [-U <mask>]", (char*) "Print/modify TraceX filters: no args to print current filters; -l to list all known filters; -a to filter out everything; -c to unfilter everything; -d to use default filter; -f/-u to filter/unfilter out by comma-delimited filter labels; -F/-U to filter/unfilter by filter masks"}, \
    { (char*) "tracex_send",    command_console_tracex_send,    0, NULL, NULL, (char*) "[-t <ip_addr>] [-p <port>] [-d <len>]",                                           (char*) "Send TraceX buffer to TCP server (TraceX must be disabled): no args to use current TCP server configuration; -t, -p, and -d to specify TCP server IP address, port, and max data packet length for sending the TraceX buffer."}, \
    { (char*) "tracex_test",    command_console_tracex_test,    1, NULL, NULL, (char*) "-n <num> [-i <id>]",                                                              (char*) "Test TraceX: -n to specify number of events to insert; -i to specify event ID (default is 4096)."},
#else
#define TRACEX_COMMANDS
#endif

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

#if defined(TX_ENABLE_EVENT_TRACE)
int command_console_tracex_enable(int argc, char* argv[]);
int command_console_tracex_disable(int argc, char* argv[]);
int command_console_tracex_restart(int argc, char* argv[]);
int command_console_tracex_status(int argc, char* argv[]);
int command_console_tracex_filter(int argc, char* argv[]);
int command_console_tracex_send(int argc, char* argv[]);
int command_console_tracex_test(int argc, char* argv[]);
#endif

#ifdef __cplusplus
} /*extern "C" */
#endif
