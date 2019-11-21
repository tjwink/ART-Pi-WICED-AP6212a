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

#include "wiced_result.h"
#include "wiced_network.h"
#include "wwd_structures.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_constants.h"

#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_constants.h"
#include "internal/wwd_bcmendian.h"
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#include "wiced_low_power.h"
#include "wiced_wifi_deep_sleep.h"

#include "platform_mcu_peripheral.h"
#include "wiced_low_power.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESCAPE_SPACE_PROMPT "\n\t-->When any parameter has spaces, use quotes. E.g. \"my ssid\" \"my wpa2 key\""

/******************************************************
 *                     Macros
 ******************************************************/
/* Below code is needed as it is shared by other snippet applications */
#ifdef CONSOLE_DISABLE_ENTERPRISE_COMMANDS
#define ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS
#else
#define ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS      WIFI_ENTERPRISE_SECURITY_COMMANDS
#endif

#define WIFI_COMMANDS_LIMITED_SET \
    { (char*) "antenna",                            antenna,                            1, NULL, NULL, (char*) "<0|1|3>",                                    (char*) "Antenna selection. 3 = Auto"},\
    { (char*) "get_ap_info",                        get_ap_info,                        0, NULL, NULL, (char*) "",                                           (char*) "Get AP information"}, \
    { (char*) "get_access_category_parameters_sta", get_access_category_parameters_sta, 0, NULL, NULL, (char*) "",                                           (char*) "Get access category parameters for STA"}, \
    { (char*) "get_associated_sta_list",            get_associated_sta_list,            0, NULL, NULL, (char*) "",                                           (char*) "Get list of associated clients"}, \
    { (char*) "get_btc_params",                     get_btc_params,                     1, NULL, NULL, (char*) "[SSID]",                                     (char*) "Find AP"}, \
    { (char*) "get_counters",                       get_counters,                       0, NULL, NULL, (char*) "[-t seconds][-n][-v][rx][tx][rate]",         (char*) "Get counters. Options:\n-t num_secs: duration to collect counters\n-n: normalize counters to a per second basis\n-v: verbose (only nonzero counters are printed by default)\nCategory: rx | tx | rate  (default: print all categories)"}, \
    { (char*) "get_country",                        get_country,                        0, NULL, NULL, (char*) "",                                           (char*) "Get country."},\
    { (char*) "set_country",                        set_country,                        1, NULL, NULL, (char*) "<US/0|KR/4>",                                (char*) "Set country."},\
    { (char*) "get_rate",                           get_rate,                           0, NULL, NULL, (char*) "",                                           (char*) "Get current rate."},\
    { (char*) "get_data_rate",                      get_data_rate,                      0, NULL, NULL, (char*) "",                                           (char*) "Get data rate."},\
    { (char*) "get_mac_addr",                       get_mac_addr,                       0, NULL, NULL, (char*) "",                                           (char*) "Get the device MAC address."}, \
    { (char*) "get_preferred_association_band",     get_preferred_association_band,     0, NULL, NULL, (char*) "",                                           (char*) "Get the preferred radio band for association."}, \
    { (char*) "get_pmk",                            get_pmk,                            1, NULL, NULL, (char*) "<key>",                                      (char*) "Get PMK"}, \
    { (char*) "get_random",                         get_random,                         0, NULL, NULL, (char*) "",                                           (char*) "Get a random number."}, \
    { (char*) "get_rssi",                           get_rssi,                           0, NULL, NULL, (char*) "",                                           (char*) "Get the received signal strength of the AP (client mode only)."}, \
    { (char*) "get_soft_ap_credentials",            get_soft_ap_credentials,            0, NULL, NULL, (char*) "",                                           (char*) "Get SoftAP credentials"},\
    { (char*) "get_tx_power",                       get_tx_power,                       0, NULL, NULL, (char*) "",                                           (char*) "Gets the tx power in dBm."},\
    { (char*) "join",                               join,                               2, NULL, NULL, (char*) "<ssid> <open|wpa_aes|wpa_tkip|wpa2|wpa2_tkip|wpa2_fbt> [key] [channel] [ip netmask gateway]"ESCAPE_SPACE_PROMPT, (char*) "Join an AP. DHCP assumed if no IP address provided"}, \
    { (char*) "leave",                              leave,                              0, NULL, NULL, (char*) "",                                           (char*) "Leave an AP."}, \
    { (char*) "scan",                               scan,                               0, NULL, NULL, (char*) "[scan_type] [0 = Infra | 1 = Adhoc | 2 = Any] [optional_ssid] [optional_mac] [(optional_channel_list)] [(optional_extended_params)]\n\tscan_type flag: 0 = active, 1 = passive, 2 = pno, 4 = prohibit, 8 = no bssid filter\n\tEx: scan 0 - - - (1,2,3,4) \"-\" denotes default", \
                                                                                                                                                             (char*) "Scan all enabled channels and print a list of APs found."}, \
    { (char*) "set_preferred_association_band",     set_preferred_association_band,     0, NULL, NULL, (char*) "<0 = Auto | 1 = 5GHz | 2 = 2.4GHz>",         (char*) "Set the preferred radio band for association"}, \
    { (char*) "set_tx_power",                       set_tx_power,                       1, NULL, NULL, (char*) "<0-31>",                                     (char*) "Set the tx power in dBm."},\
    { (char*) "status",                             status,                             0, NULL, NULL, (char*) "",                                           (char*) "Print status of the Wi-Fi and network interfaces."},\
    { (char*) "wifi_powersave",                     wifi_powersave,                     1, NULL, NULL, (char*) "<mode> [delay]",                             (char*) "Enable/disable Wi-Fi powersave mode. 0 = disable. 1 = PS Poll. 2 = Wait [delay] ms before entering powersave"}, \
    { (char*) "wifi_resume",                        wifi_resume,                        0, NULL, NULL, (char*) "[<ip> <netmask> <gateway>]",                 (char*) "Resume networking after deep-sleep"}, \
    { (char*) "wlan_chip_log",                      read_wlan_chip_console_log,         0, NULL, NULL, (char*) "",                                           (char*) "Dump WLAN chip console log"}, \
    { (char*) "wlog",                               read_wlan_chip_console_log,         0, NULL, NULL, (char*) "",                                           (char*) "Dump WLAN chip console log"}, \
    { (char*) "wwd_stats",                          print_wwd_stats,                    0, NULL, NULL, (char*) "[clear]",                                    (char*) "Dump WWD stats. clear=1, will reset stats after printing"}, \
    { (char*) "find_ap",                            find_ap,                            1, NULL, NULL, (char*) "[SSID]",                                     (char*) "Find AP"}, \
    { (char*) "country",                            get_set_country,                    0, NULL, NULL, (char*) "[US|KR|JP]",                                    (char*) "Get/Set ccode."},\
    { (char*) "reset_wifi_counters",                reset_statistics_counters,          0, NULL, NULL, (char*) "<set> [value]",                              (char*) "Reset persistent WiFi statistics counters"}, \
    { (char*) "wlver",                              wlver,                              0, NULL, NULL, (char*) "",                                           (char*) "Get WLAN Firmware Version"}, \
    { (char*) "clmver",                             clmver,                             0, NULL, NULL, (char*) "",                                           (char*) "Get WLAN CLM Version"}, \
    { (char*) "memuse",                             memuse,                             0, NULL, NULL, (char*) "",                                           (char*) "Get WLAN memory uage"}, \
    { (char*) "wlog_stream",                        wl_continuous,                      0, NULL, NULL, (char*) "[start | stop]",                             (char*) "Start & Stop continuous wlan logging"}, \
    { (char*) "dump",                                dump,                                0, NULL, NULL, (char*) "<ampdu>|<amsdu>|<all>",                         (char*) "Get dump from the FW.Ex:- dump ampdu, FW binary should have the ampdu dump feature enabled."},\
    { (char*) "ampdu_clear_dump",                    ampdu_clear_dump,                    0, NULL, NULL, (char*) "",                                              (char*) "Clear ampdu dump."},\
    { (char*) "get_curr_band",                      get_curr_band,                      0, NULL, NULL, (char*) "",                                           (char*) "Get the current radio band."}, \
    { (char*) "get_bw",                             get_bw,                             0, NULL, NULL, (char*) "",                                           (char*) "Get the bandwidth."}, \
    { (char*) "get_channel",                         get_channel,                         0, NULL, NULL, (char*) "",                                           (char*) "Get the channel number."}, \
    { (char*) "get_pm_mode",                         get_pm_mode,                         0, NULL, NULL, (char*) "",                                           (char*) "Get the WLAN PM mode."}, \
    { (char*) "wilog",                               read_wlan_ioctls,                   0, NULL, NULL, (char*) "",                                           (char*) "prints wlan ioctls, iovars & events "}, \

#define WIFI_ENTERPRISE_SECURITY_COMMANDS \
    { (char*) "join_ent",                           join_ent,                           0, NULL, NULL, (char*) "<ssid> <eap_tls|peap|eap_ttls> [username] [password] [eap] [mschapv2] [client-cert] <wpa2|wpa2_tkip|wpa|wpa_tkip|wpa2_fbt>",       (char*) "Join an AP using an enterprise EAP method. DHCP assumed."}, \
    { (char*) "leave_ent",                          leave_ent,                          0, NULL, NULL, (char*) "",                                                                                                   (char*) "Leaves an enterprise AP and stops processing enterprise security events."}, \

#define WIFI_COMMANDS \
    ALL_COMMANDS_ENTERPRISE_SECURITY_COMMANDS \
    WIFI_COMMANDS_LIMITED_SET \
    { (char*) "disable_11n",                        disable_11n,                        0, NULL, NULL, (char*) "",                                           (char*) "Disable 11n mode operation"},\
    { (char*) "ds1_config",                         ds1_config,                         2, NULL, NULL, (char*) "<ulp_wait> [type [value1] [value2] [value3]]",(char*) "Configure deep sleep (ds1).\nExamples: ds1_config <ulp wait: ex. 5000> gtk\nds1_config <ulp wait: ex. 5000> magic keep_alive <period msecs: ex. 20> <packet data: ex. 0x3243567abcdef> pattern <offset in packet: ex. 2> <mask: ex. ff> <pattern: ex. ff> arp_hostip <v4 address: ex. 192.168.1.115>\nds1_config 5000 all\nds1_config 5000 deauth"}, \
    { (char*) "ds1_enable",                         ds1_enable,                         0, NULL, NULL, (char*) "",                                           (char*) "Enable Wi-Fi Deep Sleep; enter sleep when able\n"},\
    { (char*) "ds1_enter",                          ds1_enter,                          2, NULL, NULL, (char*) "<magic|keep_alive|pattern|arp_hostip> <ulp_wait> [value1] [value2] [value3] [-wowl=1] [-wowl_os=3] [-read_back]", (char*) "Enter deep sleep (ds1) with given offload.\nExamples: ds1_enter keep_alive <ulp wait: ex. 5500> <period msecs: ex. 20> <packet data: ex. 0x3243567abcdef>\nds1_enter pattern <ulp wait: ex. 8> <offset in packet: ex. 20> <mask: ex. 0xffe008> <pattern: ex. 0x34567890>\nds1_enter arp_hostip <ulp wait: ex. 8>  <v4 address: ex. 192.168.1.115>\n\nDebug options:\n-read_back: read back and print firmware values after set.\n-wowl=0x2 or -wowl_os=0x2: force wowl or wowl_os value in firmware to be 2 in process of going to DS.\n"}, \
    { (char*) "ds1_wake",                           ds1_wake,                           0, NULL, NULL, (char*) "", (char*)"Wake from DS1 and go to Powersave mode"}, \
    { (char*) "ds1_disable",                        ds1_disable,                        0, NULL, NULL, (char*) "", (char*)"Disable DS1"}, \
    { (char*) "ds1_status",                         ds1_status,                         0, NULL, NULL, (char*) "", (char*)"Current DS1 status"}, \
    { (char*) "enable_11n",                         enable_11n,                         0, NULL, NULL, (char*) "",                                           (char*) "Enable 11n mode operation"},\
    { (char*) "fbtoverds",                          fbt_over_ds,                        0, NULL, NULL, (char*) "[value]", (char*) "Use of FBT(Fast BSS Transition) Over-the-DS(Distribution System) is allowed "}, \
    { (char*) "fbt_cap",                            fbt_caps,                           0, NULL, NULL, (char*) "",                                           (char*) "Driver 4-way handshake & reassoc (WLFBT) "}, \
    { (char*) "get_noise",                          get_noise,                          0, NULL, NULL, (char*) "",                                           (char*) "Get PHY noise after successful TX"},\
    { (char*) "join_adhoc",                         join_adhoc,                         6, NULL, NULL, (char*) "<ssid> <open|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [ip netmask gateway]"ESCAPE_SPACE_PROMPT, (char*) "Join specified IBSS, no DHCP assumed."}, \
    { (char*) "join_specific",                      join_specific,                      2, NULL, NULL, (char*) "<ssid> <bssid> <channel> <open|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [ip netmask gateway]"ESCAPE_SPACE_PROMPT, (char*) "Join specified AP. DHCP assumed if no IP address provided"}, \
    { (char*) "phyrate_dump",                       get_phyrate_log,                    1, NULL, NULL, (char*) "<bin size>",                                 (char*) "Dump the phyrate log and bin averages to the console."}, \
    { (char*) "scan_disable",                       scan_disable,                       1, NULL, NULL, (char*) "<1 = disable scan|0 = enable scan>",         (char*) "Disable scanning in FW; abort any active scan."}, \
    { (char*) "roam_delta",                         roam_delta,                         0, NULL, NULL, (char*) "<2g|5g|all> [value]",                        (char*) "Set or get roam delta value. roam_delta 2g 5"},\
    { (char*) "roam_trigger",                       roam_trigger,                       0, NULL, NULL, (char*) "<2g|5g|all> [value]",                        (char*) "Set or get roam trigger value. roam_trigger all -27"},\
    { (char*) "set_data_rate",                      set_data_rate,                      1, NULL, NULL, (char*) "<1|2|5.5|6|9|11|12|18|24|36|48|54>",         (char*) "Set data rate."},\
    { (char*) "set_legacy_rate",                    set_legacy_rate,                    1, NULL, NULL, (char*) "<1|2|5.5|6|9|11|12|18|24|36|48|54>",         (char*) "Set legacy (CCK/OFDM) rate on PHY"},\
    { (char*) "set_mcs_rate",                       set_mcs_rate,                       1, NULL, NULL, (char*) "<MCS Index> <Override MCS only>",            (char*) "Set MCS rate on PHY"},\
    { (char*) "set_listen_interval",                set_listen_interval,                1, NULL, NULL, (char*) "<listen interval> <time unit>",              (char*) "Set listen interval in time unit 0 = Beacon Intervals. 1 = DTIM Intervals"}, \
    { (char*) "start_ap",                           start_ap,                           4, NULL, NULL, (char*) "<ssid> <open|wpa2|wpa2_aes|wep|wep_shared> <key> <channel> <wps> [ip] [netmask]\n-->When any parameter has spaces, use quotes.\n\tE.g. start_ap \"my ssid\" wpa2 \"my wpa2 key \" 11 no_wps 192.168.2.1 255.255.255.0.  Default settings for ip and subnet mask are 192.168.0.1 and 255.255.255.0, or the last ip and subnet specified through this command if applicable.",(char*) "Start AP mode."}, \
    { (char*) "stop_ap",                            stop_ap,                            0, NULL, NULL, (char*) "",                                           (char*) "Stop AP mode."}, \
    { (char*) "test_ap",                            test_ap,                            4, NULL, NULL, (char*) "<ssid> <open|wpa2|wpa2_aes> <key> <channel> <wps> <iterations>\n\t-->When any parameter has spaces, use quotes.\n\tE.g. test_ap \"my ssid\" wpa2 \"my wpa2 key \" 11",(char*) "Test AP mode."}, \
    { (char*) "test_join",                          test_join,                          2, NULL, NULL, (char*) "<ssid> <open|wep|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [ip netmask gateway] <iterations>"ESCAPE_SPACE_PROMPT, (char*) "Test joining an AP. DHCP assumed if no IP address provided"}, \
    { (char*) "test_join_specific",                 test_join_specific,                 2, NULL, NULL, (char*) "<ssid> <bssid> <channel> <open|wep|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [ip netmask gateway] <iterations>"ESCAPE_SPACE_PROMPT, (char*) "Test joining an AP. DHCP assumed if no IP address provided"}, \
    { (char*) "test_cred",                          test_credentials,                   2, NULL, NULL, (char*) "<ssid> <bssid> <channel> <open|wep|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key]"ESCAPE_SPACE_PROMPT, (char*) "Test joining an AP"}, \
    { (char*) "peek",                               peek,                               1, NULL, NULL, (char*) " [address]",                                 (char*) "Dump memory"}, \
    { (char*) "poke",                               poke,                               2, NULL, NULL, (char*) " [address] [value]",                         (char*) "Write memory"}, \
    { (char*) "peek_wifi",                          peek_wifi,                          1, NULL, NULL, (char*) " [address] <# repeat>",                      (char*) "Dump wifi memory for 1 or given # of contiguous memory locations"}, \
    { (char*) "poke_wifi",                          poke_wifi,                          2, NULL, NULL, (char*) " [address] [value]",                         (char*) "Write wifi memory"}, \
    { (char*) "rrm",                                set_get_rrm,                        1, NULL, NULL, (char*) "<set|get> [value]",                          (char*) "enable or disable an RRM report\n\t rrm set [value]: [value] is the bit mask to set the corresponding report\n\t rrm get : shows the currently set RRM reports\n\tE.g. rrm set +Channel_load_Measurement\n\t     rrm set 0x100\n\t     rrm get\n\t"}, \
    { (char*) "rrm_nbr_req",                        rrm_nbr_req,                        1, NULL, NULL, (char*) "[SSID]",                                     (char*) "send 11k neighbor report measurement request (works only when associated)"}, \
    { (char*) "rrm_lm_req",                         rrm_lm_req,                         1, NULL, NULL, (char*) "[da]",                                       (char*) "send 11k link measurement request"}, \
    { (char*) "rrm_bcn_req",                        rrm_bcn_req,                        7, NULL, NULL, (char*) "[bcn mode] [da] [duration] [random int] [channel] [ssid] [repetitions]", (char*) "send 11k beacon measurement request"}, \
    { (char*) "rrm_chload_req",                     rrm_req,                            6, NULL, NULL, (char*) "[regulatory] [da] [duration] [random int] [channel] [repetitions]", (char*) "send 11k channel load measurement request"}, \
    { (char*) "rrm_noise_req",                      rrm_req,                            6, NULL, NULL, (char*) "[regulatory] [da] [duration] [random int] [channel] [repetitions]", (char*) "send 11k noise measurement request"}, \
    { (char*) "rrm_frame_req",                      rrm_frame_req,                      7, NULL, NULL, (char*) "[regulatory] [da] [duration] [random int] [channel] [ta] [repetitions]", (char*) "send 11k frame measurement request"}, \
    { (char*) "rrm_stat_req",                       rrm_stat_req,                       6, NULL, NULL, (char*) "[da] [random int] [duration] [peer] [group id] [repetitions]", (char*) "send 11k stat measurement request"}, \
    { (char*) "rrm_nbr_list",                       rrm_nbr_list,                       0, NULL, NULL, (char*) "",                                           (char*) "get 11k neighbor report list"}, \
    { (char*) "rrm_nbr_del_nbr",                    rrm_nbr_del_nbr,                    1, NULL, NULL, (char*) "[bssid]",                                    (char*) "delete node from 11k neighbor report list"}, \
    { (char*) "rrm_nbr_add_nbr",                    rrm_nbr_add_nbr,                    5, NULL, NULL, (char*) "[bssid] [bssid info] [regulatory] [channel] [phytype]", (char*) "add node to 11k neighbor report list"}, \
    { (char*) "rrm_bcn_req_thrtl_win",              rrm_bcn_req_thrtl_win,              1, NULL, NULL, (char*) "<set|get> [value]",                          (char*) "set or get beacon throttle window (milli-seconds) in which off-channel time is computed "}, \
    { (char*) "rrm_bcn_req_max_off_chan_time",      rrm_bcn_req_max_off_chan_time,      1, NULL, NULL, (char*) "<set|get> [value]",                          (char*) "set or get maximum scan time allowed in beacon throttle window (milli-seconds)"}, \
    { (char*) "rrm_bcn_req_traff_meas_per",         rrm_bcn_req_traff_meas_per,         1, NULL, NULL, (char*) "<set|get> [value]",                          (char*) "set or get milli-second period to check traffic"}, \
    { (char*) "wnm",                                wnm_enable,                         1, NULL, NULL, (char*) "<set|get> [value] Example : [value] can be bitwise OR of below values \r\n" \
                                                                                                                                            "WL_WNM_BSSTRANS         0x00000001 \r\n" \
                                                                                                                                            "WL_WNM_PROXYARP         0x00000002 \r\n" \
                                                                                                                                            "WL_WNM_MAXIDLE          0x00000004 \r\n" \
                                                                                                                                            "WL_WNM_TIMBC            0x00000008 \r\n" \
                                                                                                                                            "WL_WNM_TFS              0x00000010 \r\n" \
                                                                                                                                            "WL_WNM_SLEEP            0x00000020 \r\n" \
                                                                                                                                            "WL_WNM_DMS              0x00000040 \r\n" \
                                                                                                                                            "WL_WNM_FMS              0x00000080 \r\n" \
                                                                                                                                            "WL_WNM_NOTIF            0x00000100 \r\n" \
                                                                                                                                            "WL_WNM_WBTEXT           0x00000200 \r\n",\
                                                                                                                                                             (char*) "set or get WNM capabilities "}, \
    { (char*) "wnm_bsstrans_query",                 wnm_bsstransition_query,            0, NULL, NULL, (char*) "optional [SSID]",                            (char*) "send BSS transition query with SSID or NULL (works only when associated)"}, \
    { (char*) "wnm_bsstrans_resp",                  wnm_bsstransition_response,         1, NULL, NULL, (char*) "<set|get> [value] Example: [value] can be one of the following \r\n"   \
                                                                                                                                            "WL_BSSTRANS_POLICY_ROAM_ALWAYS = 0 \r\n"  \
                                                                                                                                            "WL_BSSTRANS_POLICY_ROAM_IF_MODE = 1 \r\n" \
                                                                                                                                            "WL_BSSTRANS_POLICY_ROAM_IF_PREF = 2 \r\n" \
                                                                                                                                            "WL_BSSTRANS_POLICY_WAIT         = 3 \r\n" \
                                                                                                                                            "WL_BSSTRANS_POLICY_PRODUCT      = 4 \r\n",\
                                                                                                                                                             (char*) "send BSS transition response policy"}, \
    { (char*) "mfp_cap",                            mfp_capabilities,                   1, NULL, NULL, (char*) "<set|get> [value]",                          (char*) "Set or Get PMF (Protected Management Frame)"}, \
    { (char*) "wlf",                                wl_formatted_command_handler,       0, NULL, NULL, (char*) "<command> [value]",                          (char*) "Run a wl command; Must specify formatting. E.g. wlf -i <command> <int>\nwlf -ui <command> <uint>\nwlf -s <command> <string>\nForce a set: wlf -set <command>\nSpecify len of get: wlf -length=200 <command>\n"}, \
    { (char*) "nan",                                wl_nan,                             0, NULL, NULL, (char*) "<command> [value]",                          (char*) "Run nan command followed by attribute name and followed by set E.g. nan <nan_cluster_id> <set> if get then nan <nan_cluster_id> \n"}, \
    { (char*) "wlver",                              wlver,                              0, NULL, NULL, (char*) "",                                           (char*) "Get WLAN Firmware Version"}, \
    { (char*) "clmver",                             clmver,                             0, NULL, NULL, (char*) "",                                           (char*) "Get WLAN CLM Version"}, \
    { (char*) "mesh_set_channel",                   mesh_set_channel,                   1, NULL, NULL, (char*) "[channel]",         (char*) "Set channel"},\
    { (char*) "mesh_auto_peer",                     mesh_auto_peer,                     1, NULL, NULL, (char*) "[1=enable, 0=disable]",         (char*) "Enable/Disable mesh auto peer channel"},\
    { (char*) "mesh_filter",                        mesh_filter,                        1, NULL, NULL, (char*) "[mesh peer mac address]",         (char*) "Filter MAC address of mesh peer"},\
    { (char*) "join_mesh",                          join_mesh,                          1, NULL, NULL, (char*) "[ssid]",         (char*) "Join mesh network by SSID"},\
    { (char*) "mesh_mcast_rebroadcast",             mesh_mcast_rebroadcast,             1, NULL, NULL, (char*) "[1=enable, 0=disable]",         (char*) "Enable/Disable mesh mcast rebroadcast"},\


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

/* Console commands */
int antenna                           ( int argc, char* argv[] );
int disable_11n                       ( int argc, char* argv[] );
int enable_11n                        ( int argc, char* argv[] );
int get_noise                         ( int argc, char* argv[] );
int get_access_category_parameters_sta( int argc, char* argv[] );
int get_ap_info                       ( int argc, char* argv[] );
int get_associated_sta_list           ( int argc, char* argv[] );
int get_counters                      ( int argc, char* argv[] );
int get_country                       ( int argc, char* argv[] );
int set_country                       ( int argc, char* argv[] );
int get_rate                          ( int argc, char* argv[] );
int get_data_rate                     ( int argc, char* argv[] );
int dump                              ( int argc, char* argv[] );
int ampdu_clear_dump                  ( int argc, char* argv[] );
int get_curr_band                     ( int argc, char* argv[] );
int get_bw                            ( int argc, char* argv[] );
int get_pm_mode                       ( int argc, char* argv[] );
int get_channel                       ( int argc, char* argv[] );
int get_mac_addr                      ( int argc, char* argv[] );
int get_pmk                           ( int argc, char* argv[] );
int get_preferred_association_band    ( int argc, char* argv[] );
int get_random                        ( int argc, char* argv[] );
int get_rssi                          ( int argc, char* argv[] );
int get_soft_ap_credentials           ( int argc, char* argv[] );
int get_tx_power                      ( int argc, char* argv[] );
int join                              ( int argc, char* argv[] );
int join_adhoc                        ( int argc, char* argv[] );
int join_ent                          ( int argc, char* argv[] );
int read_wlan_ioctls                  ( int argc, char* argv[ ] );
int leave_ent                         ( int argc, char* argv[] );
int join_specific                     ( int argc, char* argv[] );
int leave                             ( int argc, char* argv[] );
int read_wlan_chip_console_log        ( int argc, char* argv[] );
int print_wwd_stats                   ( int argc, char* argv[] );
int get_phyrate_log                   ( int argc, char* argv[] );
int scan_disable                      ( int argc, char* argv[] );
int scan                              ( int argc, char* argv[] );
int get_set_country                   ( int argc, char* argv[] );
int set_data_rate                     ( int argc, char* argv[] );
int roam_trigger                     (int argc, char* argv[] );
int roam_delta                  (int argc, char* argv[] );
int set_legacy_rate                   ( int argc, char* argv[] );
int set_mcs_rate                      ( int argc, char* argv[] );
int set_listen_interval               ( int argc, char* argv[] );
int set_preferred_association_band    ( int argc, char* argv[] );
int set_tx_power                      ( int argc, char* argv[] );
int start_ap                          ( int argc, char* argv[] );
int status                            ( int argc, char* argv[] );
int stop_ap                           ( int argc, char* argv[] );
int test_ap                           ( int argc, char* argv[] );
int test_join                         ( int argc, char* argv[] );
int test_credentials                  ( int argc, char* argv[] );
int test_join_specific                ( int argc, char* argv[] );
int wifi_powersave                    ( int argc, char* argv[] );
int wifi_resume                       ( int argc, char* argv[] );
int peek                              ( int argc, char* argv[] );
int poke                              ( int argc, char* argv[] );
int peek_wifi                         ( int argc, char* argv[] );
int poke_wifi                         ( int argc, char* argv[] );
int get_btc_params                    ( int argc, char* argv[] );
int find_ap                           ( int argc, char* argv[] );
int set_get_rrm                       ( int argc, char* argv[] );
int rrm_nbr_req                       ( int argc, char* argv[] );
int rrm_lm_req                        ( int argc, char* argv[] );
int rrm_bcn_req                       ( int argc, char* argv[] );
int rrm_req                           ( int argc, char* argv[] );
int rrm_frame_req                     ( int argc, char* argv[] );
int rrm_stat_req                      ( int argc, char* argv[] );
int rrm_nbr_list                      ( int argc, char* argv[] );
int rrm_nbr_del_nbr                   ( int argc, char* argv[] );
int rrm_nbr_add_nbr                   ( int argc, char* argv[] );
int rrm_bcn_req_thrtl_win             ( int argc, char* argv[] );
int rrm_bcn_req_max_off_chan_time     ( int argc, char* argv[] );
int rrm_bcn_req_traff_meas_per        ( int argc, char* argv[] );
int wnm_enable                        ( int argc, char* argv[] );
int wnm_bsstransition_query           ( int argc, char* argv[] );
int wnm_bsstransition_response        ( int argc, char* argv[] );

int ds1_enter                         ( int argc, char* argv[] );
int ds1_wake                          ( int argc, char* argv[] );
int ds1_disable                       ( int argc, char* argv[] );
int ds1_enable                        ( int argc, char* argv[] );
int ds1_config                        ( int argc, char* argv[] );
int ds1_status                        ( int argc, char* argv[]);
int fbt_over_ds                       ( int argc, char* argv[] );
int fbt_caps                          ( int argc, char* argv[] );
int mfp_capabilities                  ( int argc, char* argv[] );
int reset_statistics_counters         ( int argc, char* argv[] );
int wl_formatted_command_handler      ( int argc, char* argv[] );
int wl_nan                            (int argc, char* argv[] );
int print_rrm_caps                    (radio_resource_management_capability_ie_t *rrm_cap);
int wlver                             (int argc, char* argv[] );
int clmver                            (int argc, char* argv[] );
int memuse                            (int argc, char* argv[] );
int wl_continuous                     (int argc, char* argv[] );
int mesh_set_channel                  ( int argc, char* argv[] );
int mesh_auto_peer                    ( int argc, char* argv[] );
int mesh_filter                       ( int argc, char* argv[] );
int join_mesh                         ( int argc, char* argv[] );
int mesh_mcast_rebroadcast            ( int argc, char* argv[] );
int wifi_join( char* ssid, uint8_t ssid_length, wiced_security_t auth_type, uint8_t* key, uint16_t key_length, char* ip, char* netmask, char* gateway );

#ifdef __cplusplus
} /* extern "C" */
#endif
