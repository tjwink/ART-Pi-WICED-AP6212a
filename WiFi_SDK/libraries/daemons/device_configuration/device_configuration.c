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
 */

#include <stdlib.h>
#include <string.h>
#include "wiced.h"
#include "simple_http_server.h"
#include "dns_redirect.h"
#include <wiced_utilities.h>
#include "wiced_network.h"
#include <resources.h>
#include "wiced_framework.h"
#include "wiced_wps.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/


static const wiced_wps_device_detail_t wps_details =
{
    .device_name               = PLATFORM,
    .manufacturer              = "Cypress",
    .model_name                = PLATFORM,
    .model_number              = "2.0",
    .serial_number             = "1408248",
    .device_category           = WICED_WPS_DEVICE_COMPUTER,
    .sub_category              = 7,
    .config_methods            = WPS_CONFIG_LABEL | WPS_CONFIG_VIRTUAL_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_DISPLAY_PIN,
    .authentication_type_flags = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
    .encryption_type_flags     = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
};

#define HTTPS_PORT   443
#define HTTP_PORT    80

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

extern const wiced_http_page_t config_http_page_database[];
extern const wiced_http_page_t config_https_page_database[];

/* These are accessed by config_http_content.c */
const configuration_entry_t* app_configuration = NULL;

#ifdef USE_HTTPS
wiced_simple_https_server_t*         http_server;
#else
wiced_simple_http_server_t*          http_server;
#endif

wiced_bool_t                 config_use_wps;
char                         config_wps_pin[9];

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_configure_device(const configuration_entry_t* config)
{
    wiced_bool_t*  device_configured;
    wiced_result_t result;

    result = wiced_dct_read_lock( (void**) &device_configured, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, device_configured), sizeof(wiced_bool_t) );
    if ( result != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( *device_configured != WICED_TRUE )
    {
        dns_redirector_t dns_redirector;

        /* Configure variables */
        app_configuration = config;
        config_use_wps = WICED_FALSE;

        /* Prepare the HTTP server */
#ifdef USE_HTTPS
        http_server = MALLOC_OBJECT("http server", wiced_simple_https_server_t);
        memset( http_server, 0, sizeof(wiced_simple_https_server_t) );
#else
        http_server = MALLOC_OBJECT("http server", wiced_simple_http_server_t);
        memset( http_server, 0, sizeof(wiced_simple_http_server_t) );
#endif
        /* Start the AP */
        wiced_network_up( WICED_CONFIG_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );

        /* Start the DNS redirect server */
        wiced_dns_redirector_start( &dns_redirector, WICED_CONFIG_INTERFACE );

        /* Start the HTTP server */
#ifdef USE_HTTPS
        {
            platform_dct_security_t* dct_security = NULL;
            /* Lock the DCT to allow us to access the certificate and key */
            result = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
                return result;
            }

            wiced_simple_https_server_start( http_server, HTTPS_PORT, config_http_page_database, (const uint8_t *)dct_security->certificate, dct_security->private_key, WICED_CONFIG_INTERFACE );

            /* Finished accessing the certificates */
            wiced_dct_read_unlock( dct_security, WICED_FALSE );
        }
#else
        wiced_simple_http_server_start( http_server, HTTP_PORT, config_http_page_database, WICED_CONFIG_INTERFACE );
#endif
        /* Wait for configuration to complete */
        wiced_rtos_thread_join( &http_server->thread );

        /* Cleanup HTTP server */
#ifdef USE_HTTPS
        wiced_simple_https_server_stop(http_server);
#else
        wiced_simple_http_server_stop(http_server);
#endif
        free( http_server );


        /* Cleanup DNS server */
        wiced_dns_redirector_stop(&dns_redirector);

        /* Turn off AP */
        wiced_network_down( WICED_CONFIG_INTERFACE );

        /* Check if WPS was selected */
        if (config_use_wps == WICED_TRUE)
        {
            wiced_result_t ret;
            wiced_wps_credential_t* wps_credentials = MALLOC_OBJECT("wps",wiced_wps_credential_t);

            if (config_wps_pin[0] == '\x00')
            {
                ret = wiced_wps_enrollee(WICED_WPS_PBC_MODE, &wps_details, "00000000", wps_credentials, 1);
            }
            else
            {
                ret = wiced_wps_enrollee(WICED_WPS_PIN_MODE, &wps_details, config_wps_pin, wps_credentials, 1);
            }

            if (ret == WICED_SUCCESS)
            {
                /* Write received credentials into DCT */
                struct
                {
                    wiced_bool_t             device_configured;
                    wiced_config_ap_entry_t  ap_entry;
                } temp_config;
                memset(&temp_config, 0, sizeof(temp_config));
                memcpy(&temp_config.ap_entry.details.SSID,     &wps_credentials->ssid, sizeof(wiced_ssid_t));
                memcpy(&temp_config.ap_entry.details.security, &wps_credentials->security, sizeof(wiced_security_t));
                memcpy(temp_config.ap_entry.security_key,       wps_credentials->passphrase, wps_credentials->passphrase_length);
                temp_config.ap_entry.security_key_length = wps_credentials->passphrase_length;
                temp_config.device_configured = WICED_TRUE;
                wiced_dct_write( &temp_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(temp_config) );
            }
            else
            {
                /* TODO: WPS failed.. Do something */
            }

            free(wps_credentials);
        }

        app_configuration = NULL;
    }
    wiced_dct_read_unlock( device_configured, WICED_FALSE );

    return WICED_SUCCESS;
}

