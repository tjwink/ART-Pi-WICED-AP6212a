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
 * Security Types Application
 *
 * This application snippet demonstrates how to format and write
 * security settings to the DCT prior to joining an AP, including
 * various modes of WPA2, WPA and WEP.
 *
 * The app initialises the WICED device, writes the SSID and security
 * settings to the DCT, connects to the AP and then pings the
 * network gateway repeatedly.
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - Formatting of keys associated with different security types
 *  - Writing to the DCT
 *  - Joining a network and pinging the gateway
 *
 * Application Instructions
 *   1. Modify the value of YOUR_SSID to match the SSID of your Wi-Fi
 *      access point
 *   2. Modify the value of your_security_type to match that of your AP
 *   3. Modify the value of KEY_FORMAT_HEX_FLAG to match that of your AP
 *   4. Modify the value of the security key that will be used by
 *      your_security_type to match that of your AP. For example if using
 *      WPA2 or WPA security types then modify the value of YOUR_AP_PASSPHRASE,
 *      or if using 40 or 104 bit WEP security types modify the value of
 *      YOUR_WEP40_KEY or YOUR_WEP104_KEY respectively.
 *   5. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *
 *  After joining the AP specified by YOUR_SSID, your_security_type and
 *  the passphrase or key value provided, the application sends regular
 *  ICMP ping packets to the network forever
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define PING_INTERVAL      (1000 * MILLISECONDS)

#define YOUR_SSID           "YOUR_AP_SSID"
#define YOUR_AP_PASSPHRASE  "YOUR_AP_PASSPHRASE"

#define MAX_SSID_LENGTH        32
#define MAX_PASSPHRASE_LENGTH  64
#define MAX_KEY_LENGTH         64

#define WEP_KEY_FORMAT_ASCII
//#define WEP_KEY_FORMAT_HEX

/* WEP key is ASCII or HEX */
#if defined( WEP_KEY_FORMAT_HEX )
    #define YOUR_WEP40_KEY      "0102030405"                 /* WEP 40 HEX example key */
    #define YOUR_WEP104_KEY     "0102030405060708090a0b0c0d" /* WEP 104 example key */
    #define WEP_KEY_TYPE        WEP_KEY_HEX_FORMAT
#elif defined ( WEP_KEY_FORMAT_ASCII )
    #define YOUR_WEP40_KEY      "12345"          /* WEP 40 HEX example key */
    #define YOUR_WEP104_KEY     "123456789abcd"  /* WEP 104 example key */
    #define WEP_KEY_TYPE        WEP_KEY_ASCII_FORMAT
#else
#error No WEP40 key format selected
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    YOUR_SECURITY_OPEN,
    YOUR_SECURITY_WEP40_PSK,
    YOUR_SECURITY_WEP104_PSK,
    YOUR_SECURITY_WPA_TKIP_PSK,
    YOUR_SECURITY_WPA_AES_PSK,
    YOUR_SECURITY_WPA2_AES_PSK,
    YOUR_SECURITY_WPA2_TKIP_PSK,
    YOUR_SECURITY_WPA2_MIXED_PSK,
} your_security_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t send_ping        ( void );
static wiced_result_t set_wifi_security( your_security_t security_type );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    const your_security_t       your_security_type = YOUR_SECURITY_WEP40_PSK;
    platform_dct_wifi_config_t* wifi_config_dct_local;

    /* Initialise the WICED device */
    wiced_init();

    /* Configure security for the device and join the AP */
    if ( set_wifi_security( your_security_type ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Failed to update DCT with security type\r\n" ) );
        return;
    }

    /* Try join the network */
    if ( wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS )
    {
        /* Check if we were using WEP. It is not possible to tell WEP Open from WEP Shared in a scan so we try Open first and Shared second */
        if ( your_security_type == YOUR_SECURITY_WEP40_PSK || your_security_type == YOUR_SECURITY_WEP104_PSK )
        {
            WPRINT_APP_INFO( ("WEP with open authentication failed, trying WEP with shared authentication...\r\n") );

            /* Modify the Wi-Fi config to use Shared WEP */
            wiced_dct_read_lock( (void**) &wifi_config_dct_local, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t ) );
            wifi_config_dct_local->stored_ap_list[0].details.security = WICED_SECURITY_WEP_SHARED;
            wiced_dct_write ( (const void*)wifi_config_dct_local, DCT_WIFI_CONFIG_SECTION, 0, sizeof (platform_dct_wifi_config_t) );
            wiced_dct_read_unlock( (void*)wifi_config_dct_local, WICED_TRUE );

            /* Try join the network again */
            if ( wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS )
            {
                WPRINT_APP_INFO( ("WEP with Shared authentication failed as well\r\n") );

                /* Restore the Wi-Fi config back to the default Open WEP */
                wiced_dct_read_lock( (void**) &wifi_config_dct_local, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t ) );
                wifi_config_dct_local->stored_ap_list[0].details.security = WICED_SECURITY_WEP_PSK;
                wiced_dct_write ( (const void*)wifi_config_dct_local, DCT_WIFI_CONFIG_SECTION, 0, sizeof (platform_dct_wifi_config_t) );
                wiced_dct_read_unlock( (void*)wifi_config_dct_local, WICED_TRUE );

                /* Nothing more we can do.. Give up */
                WPRINT_APP_INFO(( "Unable to join AP\r\n" ));
                return;
            }
        }
        else
        {
            WPRINT_APP_INFO(( "Unable to join AP\r\n" ));
            return;
        }
    }

    while (1)
    {
        /* Send an ICMP ping to the gateway */
        send_ping( );

        /* Wait between pings */
        wiced_rtos_delay_milliseconds( PING_INTERVAL );
    }
}

static wiced_result_t set_wifi_security( your_security_t security_type )
{
    wiced_result_t   result;
    char             security_key[MAX_KEY_LENGTH];
    uint8_t          key_length = 0;
    wiced_security_t wiced_security_type;
    platform_dct_wifi_config_t* wifi_config_dct_local;

    /* Check if the security type permits us to simply use the default AP passphrase */
    if ( (security_type != YOUR_SECURITY_OPEN ) && ( security_type != YOUR_SECURITY_WEP40_PSK ) && (security_type != YOUR_SECURITY_WEP104_PSK ) )
    {
        memcpy(security_key, YOUR_AP_PASSPHRASE, sizeof( YOUR_AP_PASSPHRASE ));
        key_length = strlen((char*)&security_key);
    }

    switch ( security_type )
    {
        case YOUR_SECURITY_OPEN:
            memset( security_key, 0, sizeof( security_key ) );
            wiced_security_type = WICED_SECURITY_OPEN;
            break;
        case YOUR_SECURITY_WEP40_PSK:
            wiced_security_type = WICED_SECURITY_WEP_PSK;
            key_length          = strlen( YOUR_WEP40_KEY );
            format_wep_keys( security_key, YOUR_WEP40_KEY, &key_length, WEP_KEY_TYPE );
            break;
        case YOUR_SECURITY_WEP104_PSK:
            wiced_security_type = WICED_SECURITY_WEP_PSK;
            key_length          = strlen( YOUR_WEP104_KEY );
            format_wep_keys( security_key, YOUR_WEP104_KEY, &key_length, WEP_KEY_TYPE );
            break;
        case YOUR_SECURITY_WPA_TKIP_PSK:
            wiced_security_type = WICED_SECURITY_WPA_TKIP_PSK;
            break;
        case YOUR_SECURITY_WPA_AES_PSK:
            wiced_security_type = WICED_SECURITY_WPA_AES_PSK;
            break;
        case YOUR_SECURITY_WPA2_AES_PSK:
            wiced_security_type = WICED_SECURITY_WPA2_AES_PSK;
            break;
        case YOUR_SECURITY_WPA2_TKIP_PSK:
            wiced_security_type = WICED_SECURITY_WPA2_TKIP_PSK;
            break;
        case YOUR_SECURITY_WPA2_MIXED_PSK:
            wiced_security_type = WICED_SECURITY_WPA2_MIXED_PSK;
            break;
        default:
            WPRINT_APP_INFO(( "Unrecognised security type\r\n" ));
            return WICED_ERROR;
        break;
    }

    /* Read the Wi-Fi config from the DCT */
    result = wiced_dct_read_lock( (void**) &wifi_config_dct_local, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t ) );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Write the new security settings into the config */
    wifi_config_dct_local->stored_ap_list[0].details.SSID.length = strlen(YOUR_SSID);
    strncpy((char*)&wifi_config_dct_local->stored_ap_list[0].details.SSID.value, YOUR_SSID, MAX_SSID_LENGTH);
    wifi_config_dct_local->stored_ap_list[0].details.security = wiced_security_type;
    memcpy((char*)wifi_config_dct_local->stored_ap_list[0].security_key, (char*)security_key, MAX_PASSPHRASE_LENGTH);
    wifi_config_dct_local->stored_ap_list[0].security_key_length = key_length;

    /* Write the modified config back into the DCT */
    result = wiced_dct_write ( (const void*)wifi_config_dct_local, DCT_WIFI_CONFIG_SECTION, 0, sizeof (platform_dct_wifi_config_t) );
    wiced_dct_read_unlock( (void*)wifi_config_dct_local, WICED_TRUE );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    return WICED_SUCCESS;
}


static wiced_result_t send_ping( void )
{
    const uint32_t     ping_timeout = 1000;
    uint32_t           elapsed_ms;
    wiced_result_t     status;
    wiced_ip_address_t ping_target_ip;

    wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &ping_target_ip );
    status = wiced_ping( WICED_STA_INTERFACE, &ping_target_ip, ping_timeout, &elapsed_ms );

    if ( status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Ping Reply %lums\r\n", elapsed_ms ));
    }
    else if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(( "Ping timeout\r\n" ));
    }
    else
    {
        WPRINT_APP_INFO(( "Ping error\r\n" ));
    }

    return WICED_SUCCESS;
}

