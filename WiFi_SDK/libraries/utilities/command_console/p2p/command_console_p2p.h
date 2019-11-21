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

#define P2P_COMMANDS \
    { (char*) "p2p_connect",                 p2p_connect,                 0,  NULL, NULL, (char*) "<pin|pbc> [8 digit PIN]",                   (char*) "Connect to an existing Group Owner or negotiate to form a group"}, \
    { (char*) "p2p_client_test",             p2p_client_test,             0,  NULL, NULL, (char*) "<SSID> <8 digit PIN> [Iterations]",         (char*) "Test P2P client connection to existing group owner"}, \
    { (char*) "p2p_discovery_disable",       p2p_discovery_disable,       0,  NULL, NULL, (char*) "",                                          (char*) "Disable P2P discovery mode"}, \
    { (char*) "p2p_discovery_enable",        p2p_discovery_enable,        0,  NULL, NULL, (char*) "[listen]",                                          (char*) "Enable P2P discovery mode"}, \
    { (char*) "p2p_discovery_test",          p2p_discovery_test,          0,  NULL, NULL, (char*) "[iterations]",                              (char*) "Test enable/disable of P2P discovery mode"}, \
    { (char*) "p2p_go_start",                p2p_go_start,                0,  NULL, NULL, (char*) "<p|n> [<ssid_suffix> <key> <operating channel>]", (char*) "Start P2P (p)ersistent or (n)on-persistent group"}, \
    { (char*) "p2p_go_stop",                 p2p_go_stop,                 0,  NULL, NULL, (char*) "",                                          (char*) "Stop P2P group owner"}, \
    { (char*) "p2p_go_test",                 p2p_go_test,                 0,  NULL, NULL, (char*) "<p|n> [iterations]",                        (char*) "Test start/stop of (p)ersistent or (n)on-persistent group"}, \
    { (char*) "p2p_go_client_test_mode",     p2p_go_client_test_mode,     0,  NULL, NULL, (char*) "<enable=1|disable=0> <pin value>",          (char*) "Allow test client to join using PIN value"}, \
    { (char*) "p2p_leave",                   p2p_leave,                   0,  NULL, NULL, (char*) "",                                          (char*) "Disassociate from P2P group owner"}, \
    { (char*) "p2p_negotiation_test",        p2p_negotiation_test,        0,  NULL, NULL, (char*) "<8 digit PIN> [Iterations]",                (char*) "Test P2P negotiation handshake"}, \
    { (char*) "p2p_peer_list",               p2p_peer_list,               0,  NULL, NULL, (char*) "",                                          (char*) "Print P2P peer list"}, \
    { (char*) "p2p_registrar_start",         p2p_registrar_start,         0,  NULL, NULL, (char*) "<pin|pbc> [8 digit PIN]",                   (char*) "Start P2P group owner's WPS registrar"}, \
    { (char*) "p2p_registrar_stop",          p2p_registrar_stop,          0,  NULL, NULL, (char*) "",                                          (char*) "Stop P2P group owner's WPS registrar"}, \
    { (char*) "p2p_set_go_intent",           p2p_set_go_intent,           0,  NULL, NULL, (char*) "<intent value>",                            (char*) "Set P2P group owner intent (0..15 where 15 = must be GO)"}, \
    { (char*) "p2p_set_go_pbc_mode_support", p2p_set_go_pbc_mode_support, 0,  NULL, NULL, (char*) "<0|1>",                                     (char*) "Set P2P group owner support for PBC mode (1 = allow, 0 = disallow)"}, \
    { (char*) "p2p_set_listen_channel",      p2p_set_listen_channel,      0,  NULL, NULL, (char*) "<channel>",                                 (char*) "Set listen channel (1, 6 or 11)"}, \
    { (char*) "p2p_set_operating_channel",   p2p_set_operating_channel,   0,  NULL, NULL, (char*) "<channel>",                                 (char*) "Set operating channel (1..11)"}, \
    { (char*) "p2p_enable_ops",              p2p_enable_ops,              0,  NULL, NULL, (char*) "<ctwindow [10 - 99]>",                      (char*) "Enable P2P OPS and set ctwindow\n\t- This command sets default CTWindow if executed without any argument"}, \
    { (char*) "p2p_disable_ops",             p2p_disable_ops,             0,  NULL, NULL, (char*) "",                                          (char*) "Disable P2P Opportunistic Powersave"}, \
    { (char*) "p2p_enable_noa",              p2p_enable_noa,              0,  NULL, NULL, (char*) "<interval [length of NoA interval in micro second]> <duration [max duration of absence in micro second]> <count [number of absence intervals]>",       (char*) "Enable P2P NoA. Currently it only supports NoA type: NORMAL\n\t- The command sets default NoA values if no parameters given"}, \
    { (char*) "p2p_disable_noa",             p2p_disable_noa,             0,  NULL, NULL, (char*) "",                                          (char*) "Disable P2P noa"}, \

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

extern int p2p_connect                ( int argc, char* argv[] );
extern int p2p_discovery_disable      ( int argc, char* argv[] );
extern int p2p_discovery_enable       ( int argc, char* argv[] );
extern int p2p_discovery_test         ( int argc, char* argv[] );
extern int p2p_go_start               ( int argc, char* argv[] );
extern int p2p_go_stop                ( int argc, char* argv[] );
extern int p2p_go_test                ( int argc, char* argv[] );
extern int p2p_peer_list              ( int argc, char* argv[] );
extern int p2p_registrar_start        ( int argc, char* argv[] );
extern int p2p_registrar_stop         ( int argc, char* argv[] );
extern int p2p_leave                  ( int argc, char* argv[] );
extern int p2p_client_test            ( int argc, char* argv[] );
extern int p2p_go_client_test_mode    ( int argc, char* argv[] );
extern int p2p_set_go_intent          ( int argc, char* argv[] );
extern int p2p_set_listen_channel     ( int argc, char* argv[] );
extern int p2p_set_operating_channel  ( int argc, char* argv[] );
extern int p2p_set_go_pbc_mode_support( int argc, char* argv[] );
extern int p2p_negotiation_test( int argc, char* argv[] );
extern int p2p_enable_ops             ( int argc, char* argv[] );
extern int p2p_disable_ops            ( int argc, char* argv[] );
extern int p2p_enable_noa             ( int argc, char* argv[] );
extern int p2p_disable_noa            ( int argc, char* argv[] );


#ifdef __cplusplus
} /*extern "C" */
#endif
