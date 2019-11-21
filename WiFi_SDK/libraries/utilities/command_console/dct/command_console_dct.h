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

/** @file
 *
 * DCT changing for Console Application
 *
 * WARNING: These functions do MINIMAL error checking.
 *          It is expected that you know what you are doing when modifying DCT elements.
 *
 * NOTE: Currently supported DCT sub structures:
 *
 *      platform_dct_wifi_config_t
 *      platform_dct_network_config_t
 *      platform_dct_bt_config_t
 *      platform_dct_ota2_config_t
 *      platform_dct_misc_config_t      field   wifi_flags
 * *
 * To add this support to your application's command console:
 *
 *  1) Add to your <application>.mk:
 *
 *  $(NAME)_COMPONENTS += utilities/command_console/dct
 *
 *  2) Add this include to your application's main file <application>.c
 *
 *  #include "dct/command_console_dct.h"
 *
 *  3) Add the DCT_CONSOLE_COMMANDS to your command_t table:
 *
 *  const command_t <applicaiton>_command_table[] =
 * {
 *     ...
 *     DCT_CONSOLE_COMMANDS
 *     ...
 *     CMD_TABLE_END
 * };
 *
 *  4) Many Applications will read the DCT info and store it in RAM.
 *     Register a callback to be informed when the DCT changes.
 *
 *     typedef void (*dct_changed_callback_t)(console_dct_struct_t dct_struct_changed, void* app_data);
 *
 *  5) You can use these Macros in your Application to print info read directly from the DCT:
 *
 *  DCT_WIFI_PRINT();
 *  DCT_NETWORK_PRINT();
 *  DCT_BLUETOOTH_PRINT();
 *  DCT_OTA2_PRINT();
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                     Macros
 ******************************************************/

/* Macros for Applications to call directly to print info from the sflash DCT */
#define DCT_WIFI_PRINT()            console_dct_wifi_print(0, NULL)
#define DCT_NETWORK_PRINT()         console_dct_network_print(0, NULL)
#define DCT_BLUETOOTH_PRINT()       console_dct_bluetooth_print(0, NULL)
#define DCT_OTA2_PRINT()            console_dct_ota2_print(0, NULL)
#define DCT_MISC_PRINT()            console_dct_misc_print( 0, NULL);


/******************************************************
 *                    Constants
 ******************************************************/

#define DCT_CONSOLE_COMMANDS \
    { (char*) "dct_wifi_configured",    console_dct_wifi_configured,  0, NULL, NULL, "<0|1|f[alse]|t[rue]>",(char*) "WiFi DCT set configured state"},   \
    { (char*) "dct_wifi_country",       console_dct_wifi_country,     0, NULL, NULL, "<US/0|KR/4,etc>",     (char*) "WiFi DCT set country code"},       \
    { (char*) "dct_wifi_mac",           console_dct_wifi_mac,         0, NULL, NULL, "<[00:11:22:33:44:]55>", (char*) "WiFi DCT set mac address"},        \
    { (char*) "dct_wifi_ap_list",       console_dct_wifi_ap_list,     2, NULL, NULL, "<0|1|2|3|4> <ssid ssid_name>\r\n"                                 \
                                                                "                     <0|1|2|3|4> <bssid <[00:11:22:33:44:]55>>\r\n"                      \
                                                                "                     <0|1|2|3|4> <<bss_type|type> <infra[structure]|adhoc|any>>\r\n"   \
                                                                "                     <0|1|2|3|4> <sec[urity] <open|wep|shared|wpa_aes|wpa_tkip|wpa_mix|wpa2_aes|\r\n" \
                                                                "                         wpa2_tkip|wpa2_mix|wpa2_aesent|wpa2_tkipent|wpa2_mixent|ibss|wps_open|wps_aes>>\r\n"\
                                                                "                     <0|1|2|3|4> <<key|pass> passphrase>\r\n"                          \
                                                                "                     <0|1|2|3|4> <chan[nel] channel_num>\r\n",                         \
                                                                                                           (char*) "WiFi DCT AP List parameters"},      \
    { (char*) "dct_wifi_soft_ap",       console_dct_wifi_soft_ap,     1, NULL, NULL, "<ssid ssid_name>\r\n"                                             \
                                                                "                     <sec[urity] <open|wep|shared|wpa_aes|wpa_tkip|wpa_mix|wpa2_aes|\r\n" \
                                                                "                         wpa2_tkip|wpa2_mix|wpa2_aesent|wpa2_tkipent|wpa2_mixent|ibss|wps_open|wps_aes>>\r\n"\
                                                                "                     <<key|pass> passphrase>\r\n"                                      \
                                                                "                     <chan[nel] channel_num>\r\n"                                      \
                                                                "                     <valid <0|1|f[alse]|t[rue]>>",(char*) "WiFi DCT soft AP parameters"},\
    { (char*) "dct_wifi_config_ap",     console_dct_wifi_config_ap,   1, NULL, NULL, "<ssid ssid_name>\r\n"                                             \
                                                                "                     <sec[urity] <open|wep|shared|wpa_aes|wpa_tkip|wpa_mix|wpa2_aes|\r\n" \
                                                                "                         wpa2_tkip|wpa2_mix|wpa2_aesent|wpa2_tkipent|wpa2_mixent|ibss|wps_open|wps_aes>>\r\n"\
                                                                "                     <<key|pass> passphrase>\r\n"                                      \
                                                                "                     <chan[nel] channel_num>\r\n"                                      \
                                                                "                     <valid <0|1|f[alse]|t[rue]>>", (char*) "WiFi DCT configuration AP parameters"},\
    { (char*) "dct_wifi_print",         console_dct_wifi_print,       0, NULL, NULL, "",                    (char*) "WiFi DCT Print"},                  \
    { (char*) "dct_net_iface",          console_dct_network_interface,0, NULL, NULL, "<STA|AP|P2P|Ether>",  (char*) "Network DCT set interface"},       \
    { (char*) "dct_net_host",           console_dct_network_hostname, 0, NULL, NULL, "<hostname>",          (char*) "Network DCT set hostname"},        \
    { (char*) "dct_net_print",          console_dct_network_print,    0, NULL, NULL, "",                    (char*) "Network DCT Print"},               \
    { (char*) "dct_bt_addr",            console_dct_bluetooth_mac,    0, NULL, NULL, "<[00:11:22:33:44:]55>", (char*) "Bluetooth DCT set device address"},\
    { (char*) "dct_bt_mac",             console_dct_bluetooth_mac,    0, NULL, NULL, "<[00:11:22:33:44:]55>", (char*) "Bluetooth DCT set device address"},\
    { (char*) "dct_bt_name",            console_dct_bluetooth_name,   0, NULL, NULL, "<device_name>",       (char*) "Bluetooth DCT set device name"},   \
    { (char*) "dct_bt_class",           console_dct_bluetooth_class,  0, NULL, NULL, "<00:11:22>",          (char*) "Bluetooth DCT set device class"},  \
    { (char*) "dct_bt_debug",           console_dct_bluetooth_debug,  0, NULL, NULL, "<0|1|f[alse]|t[rue]>",(char*) "Bluetooth DCT set ssp debug mode"},\
    { (char*) "dct_bt_print",           console_dct_bluetooth_print,  0, NULL, NULL, "",                    (char*) "Bluetooth DCT Print"},             \
    { (char*) "dct_ota2_force",         console_dct_ota2_force_factory_reset, 0, NULL, NULL, "<0|1|f[alse]|t[rue]>",(char*) "OTA2 DCT set force_factory_reset"},\
    { (char*) "dct_ota2_print",         console_dct_ota2_print,       0, NULL, NULL, "",                    (char*) "OTA2 DCT print"},                  \
    { (char*) "dct_misc_print",         console_dct_misc_print,       0, NULL, NULL, "",                    (char*) "Misc DCT print"},                  \
    { (char*) "dct_misc_wifi",          console_dct_misc_wifi,        1, NULL, NULL, " <flag> <0|1|f[alse]|t[rue]>\r\n"                                 \
                                                                "                     OR <flag> <feature> <arg>\r\n"                                    \
                                                                "                     where <flag> is one of <mesh | tbd>\r\n"                          \
                                                                "                     AND   <feature> is <rb> <0|1|f[alse]|t[rue]> (rebroadcast)\r\n" \
                                                                "                           OR           <ip> <0=dhcpc | 1=dhcps | 2=static> (ip type)\r\n",      \
                                                                                     (char*) "Misc WiFi <FLAG> set /clear"},                            \

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* Indicate in the callback to the application which DCT structure has changed */
typedef enum
{
    CONSOLE_DCT_STRUCT_TYPE_WIFI = 0,
    CONSOLE_DCT_STRUCT_TYPE_NETWORK,
    CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH,
    CONSOLE_DCT_STRUCT_TYPE_OTA2,
    CONSOLE_DCT_STRUCT_TYPE_MESH

} console_dct_struct_type_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* function to define in Application for DCT changed callback */
typedef void (*dct_changed_callback_t)(console_dct_struct_type_t dct_struct_changed, void* app_data);

/* Register a callback for when a DCT value changes
 *
 * This function will call the designated callback whenever a change is made to the DCT from
 * a console_dct_xxx console command.
 *
 * param    dct_changed_callback    callback function (NULL to disable)
 * param    app_data                opaque data supplied by application and returned in the callback
 *
 */
void console_dct_register_callback(dct_changed_callback_t dct_changed_callback, void *app_data);

int console_dct_wifi_configured( int argc, char* argv[] );
int console_dct_wifi_country( int argc, char* argv[] );
int console_dct_wifi_mac( int argc, char* argv[] );
int console_dct_wifi_ap_list( int argc, char* argv[] );
int console_dct_wifi_soft_ap( int argc, char* argv[] );
int console_dct_wifi_config_ap( int argc, char* argv[] );
int console_dct_wifi_print( int argc, char* argv[] );

int console_dct_network_interface( int argc, char* argv[] );
int console_dct_network_hostname( int argc, char* argv[] );
int console_dct_network_print( int argc, char* argv[] );

int console_dct_bluetooth_mac( int argc, char* argv[] );
int console_dct_bluetooth_name( int argc, char* argv[] );
int console_dct_bluetooth_class( int argc, char* argv[] );
int console_dct_bluetooth_debug( int argc, char* argv[] );
int console_dct_bluetooth_print( int argc, char* argv[] );

int console_dct_ota2_force_factory_reset( int argc, char* argv[] );
int console_dct_ota2_print( int argc, char* argv[] );

int console_dct_misc_wifi( int argc, char* argv[] );
int console_dct_misc_print( int argc, char* argv[] );



#ifdef __cplusplus
}   /*extern "C"    */
#endif
