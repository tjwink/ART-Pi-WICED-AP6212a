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
#include "wiced_wifi.h"
#include "string.h"
#include "wwd_debug.h"
#include "command_console.h"
#include "wwd_assert.h"
#include "wwd_network.h"
#include "stdlib.h"
#include "wwd_management.h"
#include "WWD/internal/wwd_sdpcm.h"
#include "wifi.h"
#include "internal/wwd_internal.h"
#include "network/wwd_buffer_interface.h"
#include "wiced_management.h"
#include "dhcp_server.h"
#include "wiced_crypto.h"
#include "wiced.h"
#include "wifi_utils.h"

/* define this to print out the WIFI DCT info */
//#define PRINT_WIFI_DCT
#ifdef PRINT_WIFI_DCT
static wiced_result_t print_wifi_config_dct( platform_dct_wifi_config_t* dct_wifi_config );
#else
#define print_wifi_config_dct( config )
#endif

int wifi_join(char* ssid, uint8_t ssid_length, wiced_security_t auth_type, uint8_t* key, uint16_t key_length, char* ip, char* netmask, char* gateway)
{
    wiced_network_config_t network_config;
    wiced_ip_setting_t* ip_settings = NULL;
    wiced_ip_setting_t static_ip_settings;
    platform_dct_wifi_config_t* dct_wifi_config;

    if (wwd_wifi_is_ready_to_transceive(WWD_STA_INTERFACE) == WWD_SUCCESS)
    {
        return ERR_CMD_OK;
    }

    // Read config
    wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
    print_wifi_config_dct(dct_wifi_config);

    // Modify config
    memcpy((char*)dct_wifi_config->stored_ap_list[0].details.SSID.value, ssid, ssid_length);
    dct_wifi_config->stored_ap_list[0].details.SSID.length = ssid_length;
    dct_wifi_config->stored_ap_list[0].details.security = auth_type;
    memcpy((char*)dct_wifi_config->stored_ap_list[0].security_key, (char*)key, MAX_PASSPHRASE_LEN);
    dct_wifi_config->stored_ap_list[0].security_key_length = key_length;

    // Write config
    wiced_dct_write( (const void*) dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
    print_wifi_config_dct(dct_wifi_config);

    /* Tell the network stack to setup it's interface */
    if ( ip == NULL )
    {
        network_config = WICED_USE_EXTERNAL_DHCP_SERVER;
    }
    else
    {
        network_config = WICED_USE_STATIC_IP;
        if ( str_to_ip( ip, &static_ip_settings.ip_address ) != WICED_SUCCESS )
        {
            static_ip_settings.ip_address.version = WICED_IPV4; /* Force to IPv4 in case the IP address was terminated with a space rather than a NULL */
        }
        if ( str_to_ip( netmask, &static_ip_settings.netmask ) != WICED_SUCCESS )
        {
            static_ip_settings.netmask.version = WICED_IPV4; /* Force to IPv4 in case the netmask was terminated with a space rather than a NULL */
        }
        if ( str_to_ip( gateway, &static_ip_settings.gateway ) != WICED_SUCCESS )
        {
            static_ip_settings.gateway.version = WICED_IPV4; /* Force to IPv4 in case the gateway address was terminated with a space rather than a NULL */
        }
        ip_settings = &static_ip_settings;
    }

    if ( wiced_network_up( WICED_STA_INTERFACE, network_config, ip_settings ) != WICED_SUCCESS )
    {
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

#ifdef PRINT_WIFI_DCT
static wiced_result_t print_wifi_config_dct( platform_dct_wifi_config_t* dct_wifi_config )
{
    WPRINT_APP_INFO( ( "\n----------------------------------------------------------------\n\n") );

    /* Wi-Fi Config Section */
    WPRINT_APP_INFO( ( "Wi-Fi Config Section \n") );
    WPRINT_APP_INFO( ( "    device_configured               : %d \n", dct_wifi_config->device_configured ) );
    WPRINT_APP_INFO( ( "    stored_ap_list[0]  (SSID)       : %.*s \n", dct_wifi_config->stored_ap_list[0].details.SSID.length, dct_wifi_config->stored_ap_list[0].details.SSID.value ) );
    WPRINT_APP_INFO( ( "    stored_ap_list[0]  (Passphrase) : %.*s \n", dct_wifi_config->stored_ap_list[0].security_key_length, dct_wifi_config->stored_ap_list[0].security_key ) );
    WPRINT_APP_INFO( ( "    soft_ap_settings   (SSID)       : %.*s \n", dct_wifi_config->soft_ap_settings.SSID.length, dct_wifi_config->soft_ap_settings.SSID.value ) );
    WPRINT_APP_INFO( ( "    soft_ap_settings   (Passphrase) : %.*s \n", dct_wifi_config->soft_ap_settings.security_key_length, dct_wifi_config->soft_ap_settings.security_key ) );
    WPRINT_APP_INFO( ( "    config_ap_settings (SSID)       : %.*s \n", dct_wifi_config->config_ap_settings.SSID.length, dct_wifi_config->config_ap_settings.SSID.value ) );
    WPRINT_APP_INFO( ( "    config_ap_settings (Passphrase) : %.*s \n", dct_wifi_config->config_ap_settings.security_key_length, dct_wifi_config->config_ap_settings.security_key ) );
    WPRINT_APP_INFO( ( "    country_code                    : %c%c%d \n", ((dct_wifi_config->country_code) >>  0) & 0xff,
                                                                            ((dct_wifi_config->country_code) >>  8) & 0xff,
                                                                            ((dct_wifi_config->country_code) >> 16) & 0xff));
    //print_mac_address( "    DCT mac_address                 :", (wiced_mac_t*)&dct_wifi_config->mac_address );

    return WICED_SUCCESS;
}
#endif
