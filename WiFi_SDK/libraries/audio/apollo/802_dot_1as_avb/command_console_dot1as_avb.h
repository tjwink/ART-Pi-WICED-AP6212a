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

#define DOT1AS_CONSOLE_COMMANDS \
    { (char*) "avb_init",       dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"avb_init"}, \
    { (char*) "avb_sync_send",  dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"avb_sync_send"}, \
    { (char*) "avb_ts_get",     dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"avb_ts_get"}, \
    { (char*) "init",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"init <device_name> <dbg/debug>"}, \
    { (char*) "auto",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"auto 1 | 0  (on/off)"}, \
    { (char*) "bdelay",         dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"bdelay_calibrate"}, \
    { (char*) "bmca",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"bmca"}, \
    { (char*) "disable",        dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"disable <port> | dot1as"}, \
    { (char*) "enable",         dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"enable  <port> |dot1as "}, \
    { (char*) "get_local",      dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"get_local"}, \
    { (char*) "get_buscal",     dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"get_buscal"}, \
    { (char*) "interval",       dot1as_console_command,    2, NULL, NULL, (char *)"", (char *)"interval <port> [sync_tx|bmca_tx|pdelay_tx|sync_rx|bmca_rx] <val>"}, \
    { (char*) "pdelay",         dot1as_console_command,    3, NULL, NULL, (char *)"", (char *)"pdelay num_lst_resp <port> <val>"}, \
    { (char*) "pri1",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"pri1 <val>"}, \
    { (char*) "pri2",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"pri2 <val>"}, \
    { (char*) "stats",          dot1as_console_command,    2, NULL, NULL, (char *)"", (char *)"stats <port> clear"}, \
    { (char*) "timesync",       dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"timesync"}, \
    { (char*) "send",           dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)"send <announce | sync | follow_up | pdelay_req | pdelay_resp >"}, \
    { (char*) "iovar",          dot1as_console_command,    3, NULL, NULL, (char *)"", (char *)"iovar <do1as wl command>"}, \
    { (char*) "enable",         dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)" (iovar) enable"}, \
    { (char*) "avb_local_time", dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)" (iovar) avb_local_time"}, \
    { (char*) "ascapable",      dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)" (iovar)ascapabl"}, \
    { (char*) "utc_offset",     dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)" (iovar) utc_offset"}, \
    { (char*) "get_cntrs",      dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)" (iovar) get_cntrs"}, \
    { (char*) "clr_cntrs",      dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)" (iovar) clr_cntrs"}, \
    { (char*) "dot1as_role",    dot1as_console_command,    1, NULL, NULL, (char *)"", (char *)" (iovar) dot1as_role"}, \
    { (char*) "as_help",        dot1as_console_command,    0, NULL, NULL, (char *)"", (char *)"dot1as_help"}, \

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

int dot1as_console_command(int argc, char *argv[]);

#ifdef __cplusplus
} /* extern "C" */
#endif
