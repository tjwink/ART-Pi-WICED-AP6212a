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

wiced_result_t create_ping_worker_thread( void );

extern int sta_get_ip_config             ( int argc, char *argv[] );
extern int sta_set_ip_config             ( int argc, char *argv[] );
extern int sta_get_info                  ( int argc, char *argv[] );
extern int sta_get_mac_address           ( int argc, char *argv[] );
extern int sta_is_connected              ( int argc, char *argv[] );
extern int sta_verify_ip_connection      ( int argc, char *argv[] );
extern int sta_get_bssid                 ( int argc, char *argv[] );
extern int device_get_info               ( int argc, char *argv[] );
extern int device_list_interfaces        ( int argc, char *argv[] );
extern int sta_set_encryption            ( int argc, char *argv[] );
extern int sta_set_psk                   ( int argc, char *argv[] );
extern int sta_associate                 ( int argc, char *argv[] );
extern int sta_preset_testparameters     ( int argc, char *argv[] );
extern int traffic_send_ping             ( int argc, char *argv[] );
extern int traffic_stop_ping             ( int argc, char *argv[] );
extern int traffic_agent_config          ( int argc, char *argv[] );
extern int traffic_agent_reset           ( int argc, char *argv[] );
extern int traffic_agent_send            ( int argc, char *argv[] );
extern int traffic_agent_receive_start   ( int argc, char *argv[] );
extern int traffic_agent_receive_stop    ( int argc, char *argv[] );
extern int sta_set_11n                   ( int argc, char *argv[] );
extern int sta_disconnect                ( int argc, char *argv[] );
extern int sta_reassoc                   ( int argc, char *argv[] );
extern int sta_p2p_reset                 ( int argc, char *argv[] );
extern int sta_get_p2p_dev_address       ( int argc, char *argv[] );
extern int sta_set_p2p                   ( int argc, char *argv[] );
extern int sta_start_autonomous_go       ( int argc, char *argv[] );
extern int sta_get_p2p_ip_config         ( int argc, char *argv[] );
extern int sta_wps_read_pin              ( int argc, char *argv[] );
extern int sta_p2p_start_group_formation ( int argc, char *argv[] );
extern int sta_set_wps_pbc               ( int argc, char *argv[] );
extern int sta_wps_enter_pin             ( int argc, char *argv[] );
extern int sta_p2p_connect               ( int argc, char *argv[] );
extern int sta_get_psk                   ( int argc, char *argv[] );
extern int sta_set_sleep                 ( int argc, char *argv[] );
extern int sta_accept_p2p_invitation_req ( int argc, char *argv[] );
extern int sta_send_p2p_invitation_req   ( int argc, char *argv[] );
extern int sta_p2p_dissolve              ( int argc, char* argv[] );
extern int sta_reset_default             ( int argc, char* argv[] );
extern int sta_set_wireless              ( int argc, char* argv[] );
extern int wlog_read                     ( int argc, char* argv[] );
extern int reboot_sigma                  ( int argc, char *argv[] );

#define WIFI_CERT_COMMANDS \
    { "sta_get_ip_config",               sta_get_ip_config,                         0,         NULL, NULL, "", "" }, \
    { "sta_set_ip_config",               sta_set_ip_config,                         0,         NULL, NULL, "", "" }, \
    { "sta_get_info",                    sta_get_info,                              0,         NULL, NULL, "", "" }, \
    { "sta_get_mac_address",             sta_get_mac_address,                       0,         NULL, NULL, "", "" }, \
    { "sta_is_connected",                sta_is_connected,                          0,         NULL, NULL, "", "" }, \
    { "sta_verify_ip_connection",        sta_verify_ip_connection,                  0,         NULL, NULL, "", "" }, \
    { "sta_get_bssid",                   sta_get_bssid,                             0,         NULL, NULL, "", "" }, \
    { "device_get_info",                 device_get_info,                           0,         NULL, NULL, "", "" }, \
    { "device_list_interfaces",          device_list_interfaces,                    0,         NULL, NULL, "", "" }, \
    { "sta_set_encryption",              sta_set_encryption,                        0,         NULL, NULL, "", "" }, \
    { "sta_set_psk",                     sta_set_psk,                               1,         NULL, NULL, "", "" }, \
    { "sta_associate",                   sta_associate,                             0,         NULL, NULL, "", "" }, \
    { "sta_preset_testparameters",       sta_preset_testparameters,                 0,         NULL, NULL, "", "" }, \
    { "traffic_send_ping",               traffic_send_ping,                         0,         NULL, NULL, "", "" }, \
    { "traffic_stop_ping",               traffic_stop_ping,                         0,         NULL, NULL, "", "" }, \
    { "traffic_agent_config",            traffic_agent_config,                      0,         NULL, NULL, "", "" }, \
    { "traffic_agent_reset",             traffic_agent_reset,                       0,         NULL, NULL, "", "" }, \
    { "traffic_agent_send",              traffic_agent_send,                        0,         NULL, NULL, "", "" }, \
    { "traffic_agent_receive_start",     traffic_agent_receive_start,               0,         NULL, NULL, "", "" }, \
    { "traffic_agent_receive_stop",      traffic_agent_receive_stop,                0,         NULL, NULL, "", "" }, \
    { "sta_set_11n",                     sta_set_11n,                               0,         NULL, NULL, "", "" }, \
    { "sta_disconnect",                  sta_disconnect,                            0,         NULL, NULL, "", "" }, \
    { "sta_reassoc",                     sta_reassoc,                               0,         NULL, NULL, "", "" }, \
    { "sta_reset_default",               sta_reset_default,                         0,         NULL, NULL, "", "" }, \
    { "sta_set_wireless",                sta_set_wireless,                          0,         NULL, NULL, "", "" }, \
    { "wlog",                            wlog_read,                                 0,         NULL, NULL, "", "" }, \
    { "reboot",                          reboot_sigma,                              0,         NULL, NULL, NULL, "Reboot the device"},


#ifdef WICED_USE_WIFI_P2P_INTERFACE
#define WIFI_P2P_CERT_COMMANDS \
    { "sta_p2p_reset",                   sta_p2p_reset,                             0,         NULL, NULL, "", "" }, \
    { "sta_get_p2p_dev_address",         sta_get_p2p_dev_address,                   0,         NULL, NULL, "", "" }, \
    { "sta_set_p2p",                     sta_set_p2p,                               0,         NULL, NULL, "", "" }, \
    { "sta_start_autonomous_go",         sta_start_autonomous_go,                   0,         NULL, NULL, "", "" }, \
    { "sta_get_p2p_ip_config",           sta_get_p2p_ip_config,                     0,         NULL, NULL, "", "" }, \
    { "sta_wps_read_pin",                sta_wps_read_pin,                          0,         NULL, NULL, "", "" }, \
    { "sta_p2p_start_group_formation",   sta_p2p_start_group_formation,             0,         NULL, NULL, "", "" }, \
    { "sta_set_wps_pbc",                 sta_set_wps_pbc,                           0,         NULL, NULL, "", "" }, \
    { "sta_wps_enter_pin",               sta_wps_enter_pin,                         0,         NULL, NULL, "", "" }, \
    { "sta_p2p_connect",                 sta_p2p_connect,                           0,         NULL, NULL, "", "" }, \
    { "sta_get_psk",                     sta_get_psk,                               0,         NULL, NULL, "", "" }, \
    { "sta_set_sleep",                   sta_set_sleep,                             0,         NULL, NULL, "", "" }, \
    { "sta_accept_p2p_invitation_req",   sta_accept_p2p_invitation_req,             0,         NULL, NULL, "", "" }, \
    { "sta_send_p2p_invitation_req",     sta_send_p2p_invitation_req,               0,         NULL, NULL, "", "" }, \
    { "sta_p2p_dissolve",                sta_p2p_dissolve,                          0,         NULL, NULL, "", "" },
#else
#define WIFI_P2P_CERT_COMMANDS
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif
