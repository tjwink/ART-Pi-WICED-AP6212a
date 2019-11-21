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
 */

#include "ctype.h"
#include "wiced.h"
#include "command_console_dct.h"
#include "command_console.h"

/* for print_mac_address() and WiFi flags */
#include "wiced_wifi.h"

/* for OTA2 types */
#include "../../../libraries/filesystems/ota2/wiced_ota2_image.h"

/******************************************************
 *                      Macros
 ******************************************************/

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
 *               Function Declarations
 ******************************************************/
static char* get_bss_type_name(wiced_bss_type_t bss_type);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static dct_changed_callback_t registered_dct_changed_callback = NULL;
static void*                  registered_dct_changed_callback_data = NULL;

typedef struct security_lookup_s {
    char     *name;
    wiced_security_t sec_type;
} security_lookup_t;

static security_lookup_t dctrw_security_name_table[] =
{
        { "open",        WICED_SECURITY_OPEN           },
        { "none",        WICED_SECURITY_OPEN           },
        { "wep",         WICED_SECURITY_WEP_PSK        },
        { "shared",      WICED_SECURITY_WEP_SHARED     },
        { "wpa_tkip",    WICED_SECURITY_WPA_TKIP_PSK   },
        { "wpa_aes",     WICED_SECURITY_WPA_AES_PSK    },
        { "wpa_mix",     WICED_SECURITY_WPA_MIXED_PSK  },
        { "wpa_tkipent", WICED_SECURITY_WPA_TKIP_ENT   },
        { "wpa_aesent",  WICED_SECURITY_WPA_AES_ENT    },
        { "wpa_mixent",  WICED_SECURITY_WPA_MIXED_ENT  },
        { "wpa2_aes",    WICED_SECURITY_WPA2_AES_PSK   },
        { "wpa2_tkip",   WICED_SECURITY_WPA2_TKIP_PSK  },
        { "wpa2_mix",    WICED_SECURITY_WPA2_MIXED_PSK },
        { "wpa2_tkipent",WICED_SECURITY_WPA2_TKIP_ENT  },
        { "wpa2_aesent", WICED_SECURITY_WPA2_AES_ENT   },
        { "wpa2_mixent", WICED_SECURITY_WPA2_MIXED_ENT },
        { "ibss",        WICED_SECURITY_IBSS_OPEN      },
        { "wps_open",    WICED_SECURITY_WPS_OPEN       },
        { "wps_none",    WICED_SECURITY_WPS_OPEN       },
        { "wps_aes",     WICED_SECURITY_WPS_SECURE     },
        { "invalid",     WICED_SECURITY_UNKNOWN        },
};

static char* interface_type_string[] =
{
        "WICED_STA_INTERFACE",
        "WICED_AP_INTERFACE",
        "WICED_P2P_INTERFACE",
        "WICED_ETHERNET_INTERFACE",
        "WICED_THREAD_INTERFACE"    /* wiced_interface_t includes this without #ifdef WICED_USE_THREAD_INTERFACE */
};

static char* ota2_boot_type_string[OTA2_MAX_BOOT_TYPES] =
{
        "OTA2_BOOT_NEVER_RUN_BEFORE",
        "OTA2_BOOT_NORMAL",
        "OTA2_BOOT_EXTRACT_FACTORY_RESET",
        "OTA2_BOOT_EXTRACT_UPDATE",
        "OTA2_BOOT_SOFTAP_UPDATE",
        "OTA2_BOOT_LAST_KNOWN_GOOD",

        "OTA2_BOOT_FACTORY_RESET",
        "OTA2_BOOT_UPDATE",

        "OTA2_BOOT_FAILSAFE_FACTORY_RESET",
        "OTA2_BOOT_FAILSAFE_UPDATE",
};

/******************************************************
 *               Common Function Definitions
 ******************************************************/

static wiced_bss_type_t parse_bss_type_name(char* type_name )
{
    wiced_bss_type_t bss_type = WICED_BSS_TYPE_UNKNOWN;

    if ((strcasecmp(type_name, "INFRASTRUCTURE") == 0) || (strncasecmp(type_name, "INFRA", 5) == 0))
    {
        bss_type = WICED_BSS_TYPE_INFRASTRUCTURE;
    }
    else if (strcasecmp(type_name, "ADHOC") == 0)
    {
        bss_type = WICED_BSS_TYPE_ADHOC;
    }
    else if (strcasecmp(type_name, "ANY") == 0)
    {
        bss_type = WICED_BSS_TYPE_ANY;
    }

    return bss_type;
}
static char* get_bss_type_name(wiced_bss_type_t bss_type)
{
    switch (bss_type )
    {
        case WICED_BSS_TYPE_INFRASTRUCTURE:
            return "INFRASTRUCTURE";
        case WICED_BSS_TYPE_ADHOC:
            return "ADHOC";
        case WICED_BSS_TYPE_ANY:
            return "ANY";
        case WICED_BSS_TYPE_UNKNOWN:
            return "BSS_TYPE_UNKNOWN";
        default:
            break;
    }

    return "ERROR";
}

static char* get_security_type_name(wiced_security_t type)
{
    int table_index;

    for (table_index = 0; table_index < sizeof(dctrw_security_name_table)/sizeof(security_lookup_t); table_index++)
    {
        if (dctrw_security_name_table[table_index].sec_type == type)
        {
            return dctrw_security_name_table[table_index].name;
        }
    }

    return "ERROR";
}

static wiced_security_t parse_security_type_name(char* security_str)
{
    wiced_security_t security_type = WICED_SECURITY_UNKNOWN;

    int table_index;

    for (table_index = 0; table_index < sizeof(dctrw_security_name_table)/sizeof(security_lookup_t); table_index++)
    {
        if (strcasecmp(dctrw_security_name_table[table_index].name, security_str) == 0)
        {
            security_type = dctrw_security_name_table[table_index].sec_type;
            break;
        }
    }

    return security_type;
}


static wiced_bool_t convert_input_to_wiced_bool(char *input)
{
    wiced_bool_t                new_configured = WICED_FALSE;

    if (isdigit((unsigned char)input[0]))
    {
        new_configured = (atoi(input) == 0) ? WICED_FALSE : WICED_TRUE;
    }
    else if (toupper((unsigned char)input[0]) == 'T')
    {
        new_configured = WICED_TRUE;
    }

    return new_configured;
}

wiced_country_code_t convert_input_to_country_code(char* input)
{
    int         rev = 0;
    char*       slash;

    slash = strchr(input, '/');
    if (slash != NULL)
    {
        rev = strtol( slash + 1, NULL, 10 );
    }

    return MK_CNTRY(input[0], input[1], rev);
}

/* set num_octets to 6 for normal MAC address parsing
 * set num_octets to 3 for Bluetooth device type parsing
 * set num_octets to 1 for to change only the last of a 6-octet mac address
 */
static wiced_result_t parse_octets(char* in, wiced_mac_t* octet, int num_octets)
{
    char* colon;
    int octet_count;
    wiced_mac_t new_mac = {{0}};

    if ((in == NULL) || (octet == NULL))
    {
        return WICED_ERROR;
    }

    switch (num_octets)
    {
        case 1:
            if (strlen(in) != 2)
            {
                return WICED_ERROR;
            }
            break;
        case 3:
            if (strlen(in) != 8)
            {
                return WICED_ERROR;
            }
            break;
        case 6:
            if (strlen(in) != 17)
            {
                return WICED_ERROR;
            }
            break;
        default:
            return WICED_ERROR;
    }

    octet_count = 0;
    while ((in != NULL) && (*in != 0) && (octet_count < num_octets))
    {
        colon = strchr(in, ':');
        if (colon != NULL)
        {
            *colon++ = 0;
        }

        /* convert the hex data */
        new_mac.octet[octet_count++] = hex_str_to_int(in);
        in = colon;
    }

    if (octet_count == num_octets)
    {
        if (num_octets == 1)
        {
            /* only adjust the last octet of 6 */
            octet->octet[5] = new_mac.octet[0];
        }
        else
        {
            memcpy(octet, &new_mac, sizeof(wiced_mac_t));
        }

        return WICED_SUCCESS;
    }

    return WICED_ERROR;
}

/** Map channel to its band, comparing channel to max 2g channel
 *  -- this routine taken and modified from WICED/WWD/internal/wwd_wifi.c
 *
 * @param channel     : The channel to map to a band
 *
 * @return                  : WL_CHANSPEC_BAND_2G or WL_CHANSPEC_BAND_5G
 */

static wiced_802_11_band_t console_channel_to_wl_band( uint32_t channel )
{
    return ( ( (channel) <= CH_MAX_2G_CHANNEL ) ? WICED_802_11_BAND_2_4GHZ : WICED_802_11_BAND_5GHZ );
}

/******************************************************
 *               DCT Changed Callback Function
 ******************************************************/
static void console_dct_callback_to_application(console_dct_struct_type_t struct_type)
{
    if (registered_dct_changed_callback != NULL)
    {
        registered_dct_changed_callback(struct_type, registered_dct_changed_callback_data);
    }
}

void console_dct_register_callback(dct_changed_callback_t dct_changed_callback, void *app_data)
{
    registered_dct_changed_callback = dct_changed_callback;
    registered_dct_changed_callback_data = app_data;
}

/******************************************************
 *               WiFi DCT Function Definitions
 ******************************************************/
/* common print functions */
static void console_dct_configured_print(wiced_bool_t device_configured)
{
    WPRINT_APP_INFO( ( " configured  : %d %s\r\n", device_configured, (device_configured == WICED_FALSE) ? "False" : "True") );
}
static void console_dct_country_code_print(wiced_country_code_t country_code)
{
    WPRINT_APP_INFO( ( " country_code: %c%c/%d \r\n", ((country_code) >>  0) & 0xff,
                                                     ((country_code) >>  8) & 0xff,
                                                     ((country_code) >> 16) & 0xffff));
}
static void console_dct_wifi_ssid_print( wiced_ssid_t *ssid, int prefix)
{
    char                        ssid_name[ SSID_NAME_SIZE + 1 ];
    strlcpy(ssid_name, (char*)ssid->value, sizeof(ssid_name));
    if (prefix == 1)
    {
        WPRINT_APP_INFO( ( "  SSID    : (%d) ", ssid->length) );
    }
    WPRINT_APP_INFO( ( "%s\r\n",ssid_name ) );
}
static void console_dct_wifi_sec_print(wiced_security_t security, int prefix)
{
    if (prefix == 1)
    {
        WPRINT_APP_INFO( ( "  sec_type: 0x%x ", security) );
    }
    WPRINT_APP_INFO( ( "%s\r\n", get_security_type_name(security) ) );
}
static void console_dct_wifi_sec_phrase_print(char *security_key, uint8_t length, int prefix)
{
    char                        passphrase[ SECURITY_KEY_SIZE + 1 ];
    strlcpy(passphrase, (char*)security_key, sizeof(passphrase));
    if (prefix == 1)
    {
        WPRINT_APP_INFO( ( "  sec_key : (%d)", length) );
    }
    WPRINT_APP_INFO( ( "%s\r\n", passphrase ) );
}

static void console_dct_wifi_ap_print(wiced_config_soft_ap_t* settings, int argc, char* argv[])
{
    if (strcasecmp(argv[1], "ssid") == 0)
    {
        console_dct_wifi_ssid_print(&settings->SSID, 0);
    }
    else if (strncasecmp(argv[1], "sec", 3) == 0)
    {
        console_dct_wifi_sec_print(settings->security, 0);
    }
    else if ((strcasecmp(argv[1], "key") == 0) || (strncasecmp(argv[1], "pass", 4) == 0))
    {
        console_dct_wifi_sec_phrase_print(settings->security_key, settings->security_key_length, 0);
    }
    else if (strncasecmp(argv[1], "chan", 4) == 0)
    {
        WPRINT_APP_INFO( ( "%d\r\n", settings->channel ) );
    }
    else if (strcasecmp(argv[1], "valid") == 0)
    {
        WPRINT_APP_INFO( ( "0x%lx\r\n", settings->details_valid) );
    }
}

int console_dct_wifi_configured( int argc, char* argv[] )
{
    platform_dct_wifi_config_t  dct_wifi_config;
    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        dct_wifi_config.device_configured = convert_input_to_wiced_bool(argv[1]);

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "%d\r\n", dct_wifi_config.device_configured ) );
    }

    return 0;
}

int console_dct_wifi_country( int argc, char* argv[] )
{
    platform_dct_wifi_config_t  dct_wifi_config;
    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        dct_wifi_config.country_code = convert_input_to_country_code(argv[1]);

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "%c%c/%d\r\n", ((dct_wifi_config.country_code) >>  0) & 0xff,
                                                         ((dct_wifi_config.country_code) >>  8) & 0xff,
                                                         ((dct_wifi_config.country_code) >> 16) & 0xffff));
    }
    return 0;
}

int console_dct_wifi_mac( int argc, char* argv[] )
{
    platform_dct_wifi_config_t  dct_wifi_config;
    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        if ((parse_octets(argv[1], &dct_wifi_config.mac_address, 6) != WICED_SUCCESS) &&
            (parse_octets(argv[1], &dct_wifi_config.mac_address, 1) != WICED_SUCCESS))
        {
            WPRINT_APP_INFO( ( " --> unparseable mac!\r\n") );
            return -1;
        }

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        print_mac_address( (wiced_mac_t*) &dct_wifi_config.mac_address );
        WPRINT_APP_INFO( ("\r\n") );
    }
    return 0;
}


int console_dct_wifi_ap_list( int argc, char* argv[] )
{
    int                         index;
    platform_dct_wifi_config_t  dct_wifi_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    /* argv[1] must be a valid index number */
    index = atoi(argv[1]);
    if ((index < 0) || (index > CONFIG_AP_LIST_SIZE))
    {
        WPRINT_APP_INFO( ( " --> invalid index %s !\r\n", argv[1]) );
        return -1;
    }

    if (argc < 3)
    {
        WPRINT_APP_INFO( ( " --> insufficient args !\r\n") );
        return -1;
    }

    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc > 3 )
    {
        if (strcasecmp(argv[2], "ssid") == 0)
        {
            dct_wifi_config.stored_ap_list[index].details.SSID.length =
                                strlcpy((char*)&dct_wifi_config.stored_ap_list[index].details.SSID.value, (char*)argv[3], SSID_NAME_SIZE);
        }
        else if ((strcasecmp(argv[2], "bssid") == 0) || (strcasecmp(argv[2], "mac") == 0))
        {
            if ((parse_octets(argv[3], &dct_wifi_config.stored_ap_list[index].details.BSSID, 6) != WICED_SUCCESS) &&
                (parse_octets(argv[3], &dct_wifi_config.stored_ap_list[index].details.BSSID, 1) != WICED_SUCCESS))
            {
                WPRINT_APP_INFO( ( " --> unparseable mac!\r\n") );
                return -1;
            }
        }
        else if ((strcasecmp(argv[2], "type") == 0) || (strcasecmp(argv[2], "bss_type") == 0))
        {
            wiced_bss_type_t new_bss_type = parse_bss_type_name(argv[3]);
            if (new_bss_type != WICED_BSS_TYPE_UNKNOWN)
            {
                dct_wifi_config.stored_ap_list[index].details.bss_type = new_bss_type;
            }
            else
            {
                WPRINT_APP_INFO( ( " --> bss type %s unknown!\r\n", argv[3]) );
                return -1;
            }
        }
        else if (strncasecmp(argv[2], "sec", 3) == 0)
        {
            wiced_security_t new_security = parse_security_type_name(argv[3]);
            if (new_security != WICED_SECURITY_UNKNOWN)
            {
                dct_wifi_config.stored_ap_list[index].details.security = new_security;
            }
            else
            {
                WPRINT_APP_INFO( ( " --> security type %s unknown!\r\n", argv[3]) );
                return -1;
            }
        }
        else if ((strcasecmp(argv[2], "key") == 0) || (strncasecmp(argv[2], "pass", 4) == 0))
        {
            dct_wifi_config.stored_ap_list[index].security_key_length = strlcpy((char*)&dct_wifi_config.stored_ap_list[index].security_key, (char*)argv[3], SECURITY_KEY_SIZE);
        }
        else if (strncasecmp(argv[2], "chan", 4) == 0)
        {
            dct_wifi_config.stored_ap_list[index].details.channel = strtol(argv[3], NULL, 10 );
            dct_wifi_config.stored_ap_list[index].details.band = console_channel_to_wl_band(dct_wifi_config.stored_ap_list[index].details.channel);
        }
        else
        {
            WPRINT_APP_INFO( ( " --> unknown command!\r\n") );
            return -1;
        }

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else if (argc == 3)
    {
        if (strcasecmp(argv[2], "ssid") == 0)
        {
            console_dct_wifi_ssid_print(&dct_wifi_config.stored_ap_list[index].details.SSID, 0);
        }
        else if ((strcasecmp(argv[2], "bssid") == 0) || (strcasecmp(argv[2], "mac") == 0))
        {
            print_mac_address( (wiced_mac_t*) &dct_wifi_config.stored_ap_list[index].details.BSSID );
            WPRINT_APP_INFO( ("\r\n") );
        }
        else if ((strcasecmp(argv[2], "type") == 0) || (strcasecmp(argv[2], "bss_type") == 0))
        {
            WPRINT_APP_INFO( ( "%s\r\n", get_bss_type_name(dct_wifi_config.stored_ap_list[index].details.bss_type) ) );
        }
        else if ((strcasecmp(argv[2], "key") == 0) || (strncasecmp(argv[2], "sec_k", 5) == 0) || (strncasecmp(argv[2], "pass", 4) == 0))
        {
            console_dct_wifi_sec_phrase_print(dct_wifi_config.stored_ap_list[index].security_key, dct_wifi_config.stored_ap_list[index].security_key_length, 0);
        }
        else if (strncasecmp(argv[2], "sec", 3) == 0)
        {
            console_dct_wifi_sec_print(dct_wifi_config.stored_ap_list[index].details.security, 0);
        }
        else if (strncasecmp(argv[2], "chan", 4) == 0)
        {
            WPRINT_APP_INFO( ( "%d\r\n", dct_wifi_config.stored_ap_list[index].details.channel ) );
        }
        else
        {
            WPRINT_APP_INFO( ( " --> unknown command!\r\n") );
            return -1;
        }
    }
    return 0;
}

static int console_parse_soft_ap_args(wiced_config_soft_ap_t* settings, int argc, char* argv[] )
{
    if (settings == NULL)
    {
        return -1;
    }

    if (argc == 3)
    {
        if (strcasecmp(argv[1], "ssid") == 0)
        {
            settings->SSID.length = strlcpy((char*)&settings->SSID.value, (char*)argv[2], SSID_NAME_SIZE);
        }
        else if ((strcasecmp(argv[1], "key") == 0) || (strncasecmp(argv[1], "sec_k", 5) == 0) || (strncasecmp(argv[1], "pass", 4) == 0))
        {
            settings->security_key_length = strlcpy((char*)&settings->security_key, (char*)argv[2], SECURITY_KEY_SIZE);
        }
        else if (strncasecmp(argv[1], "sec", 3) == 0)
        {
            wiced_security_t new_security = parse_security_type_name(argv[2]);
            if (new_security != WICED_SECURITY_UNKNOWN)
            {
                settings->security = new_security;
            }
            else
            {
                WPRINT_APP_INFO( ( " --> security type %s unknown!\r\n", argv[2]) );
                return -1;
            }
        }
        else if (strncasecmp(argv[1], "chan", 4) == 0)
        {
            settings->channel = strtol(argv[2], NULL, 10 );
        }
        else if (strcasecmp(argv[1], "valid") == 0)
        {
            settings->details_valid = (convert_input_to_wiced_bool(argv[2]) == WICED_FALSE) ? 0 : CONFIG_VALIDITY_VALUE;
        }
        else
        {
            return -1;
        }
    }
    return 0;
}

int console_dct_wifi_soft_ap( int argc, char* argv[] )
{
    platform_dct_wifi_config_t  dct_wifi_config;

    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 3)
    {
        if (console_parse_soft_ap_args(&dct_wifi_config.soft_ap_settings, argc, argv) != 0)
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else if (argc == 2)
    {
        console_dct_wifi_ap_print(&dct_wifi_config.soft_ap_settings, argc, argv);
    }

    return 0;
}

int console_dct_wifi_config_ap( int argc, char* argv[] )
{
    platform_dct_wifi_config_t  dct_wifi_config;

    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 3)
    {
        if (console_parse_soft_ap_args(&dct_wifi_config.config_ap_settings, argc, argv) != 0)
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }

        if (wiced_dct_write( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_WIFI);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else if (argc == 2)
    {
        console_dct_wifi_ap_print(&dct_wifi_config.config_ap_settings, argc, argv);
    }
    return 0;
}
int console_dct_wifi_print( int argc, char* argv[] )
{
    int                         i;
    platform_dct_wifi_config_t  dct_wifi_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (wiced_dct_read_with_copy( &dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    WPRINT_APP_INFO( ( "--- DCT WiFi ---\r\n") );
    console_dct_configured_print(dct_wifi_config.device_configured);
    console_dct_country_code_print(dct_wifi_config.country_code);
    WPRINT_APP_INFO( ( " mac_address : ") );
    print_mac_address( (wiced_mac_t*) &dct_wifi_config.mac_address );
    WPRINT_APP_INFO( ("\r\n") );

    for ( i = 0; i < CONFIG_AP_LIST_SIZE; i++ )
    {
        if((dct_wifi_config.stored_ap_list[i].details.SSID.length > 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[0] != 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[1] != 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[2] != 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[3] != 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[4] != 0) ||
           (dct_wifi_config.stored_ap_list[i].details.BSSID.octet[5] != 0) )
        {
            WPRINT_APP_INFO( ( "ap_list %d:\r\n", i) );
            console_dct_wifi_ssid_print(&dct_wifi_config.stored_ap_list[i].details.SSID, 1);
            WPRINT_APP_INFO( ( "  BSSID   : ") );
            print_mac_address( (wiced_mac_t*) &dct_wifi_config.stored_ap_list[i].details.BSSID );
            WPRINT_APP_INFO( ("\r\n") );
            console_dct_wifi_sec_print(dct_wifi_config.stored_ap_list[i].details.security, 1);
            console_dct_wifi_sec_phrase_print(dct_wifi_config.stored_ap_list[i].security_key, dct_wifi_config.stored_ap_list[i].security_key_length, 1);
            WPRINT_APP_INFO( ( "  channel : %d\r\n", dct_wifi_config.stored_ap_list[i].details.channel ) );
            WPRINT_APP_INFO( ( "  band    : %d %s\r\n", dct_wifi_config.stored_ap_list[i].details.band,
                                                    (dct_wifi_config.stored_ap_list[i].details.band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz" ) );
            WPRINT_APP_INFO( ( "  bss_type: %d %s\r\n", dct_wifi_config.stored_ap_list[i].details.bss_type, get_bss_type_name(dct_wifi_config.stored_ap_list[i].details.bss_type) ) );
            WPRINT_APP_INFO( ( "  max_rate: %ld\r\n", dct_wifi_config.stored_ap_list[i].details.max_data_rate ) );
            WPRINT_APP_INFO( ( "  strength: %d\r\n", dct_wifi_config.stored_ap_list[i].details.signal_strength ) );
        }
    }
    WPRINT_APP_INFO( ( " soft_ap_settings\r\n"));
    console_dct_wifi_ssid_print(&dct_wifi_config.soft_ap_settings.SSID, 1);
    console_dct_wifi_sec_print(dct_wifi_config.soft_ap_settings.security, 1);
    console_dct_wifi_sec_phrase_print(dct_wifi_config.soft_ap_settings.security_key, dct_wifi_config.soft_ap_settings.security_key_length, 1);
    WPRINT_APP_INFO( ( "  channel : %d\r\n", dct_wifi_config.soft_ap_settings.channel ) );
    WPRINT_APP_INFO( ( "  valid   : 0x%lx\r\n", dct_wifi_config.soft_ap_settings.details_valid ) );

    WPRINT_APP_INFO( ( " config_ap_settings\r\n"));
    console_dct_wifi_ssid_print(&dct_wifi_config.config_ap_settings.SSID, 1);
    console_dct_wifi_sec_print(dct_wifi_config.config_ap_settings.security, 1);
    console_dct_wifi_sec_phrase_print(dct_wifi_config.config_ap_settings.security_key, dct_wifi_config.config_ap_settings.security_key_length, 1);
    WPRINT_APP_INFO( ( "  channel : %d\r\n", dct_wifi_config.config_ap_settings.channel ) );
    WPRINT_APP_INFO( ( "  valid   : 0x%lx\r\n", dct_wifi_config.config_ap_settings.details_valid ) );
    WPRINT_APP_INFO( ( "-----------\r\n") );

    return 0;
}

/******************************************************
 *               Network DCT Function Definitions
 ******************************************************/

//dct_network_iface <STA|AP|P2P|ETHER>
//dct_network_hostname <hostname>

int console_dct_network_interface( int argc, char* argv[] )
{
    platform_dct_network_config_t   dct_network_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_network_config, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        if (strncasecmp(argv[1], "sta", 3) == 0)
        {
            dct_network_config.interface = WICED_STA_INTERFACE;

        }
        else if (strncasecmp(argv[1], "ap", 2) == 0)
        {
            dct_network_config.interface = WICED_AP_INTERFACE;

        }
        else if (strncasecmp(argv[1], "p2p", 3) == 0)
        {
            dct_network_config.interface = WICED_P2P_INTERFACE;

        }
        else if (strncasecmp(argv[1], "eth", 3) == 0)
        {
            dct_network_config.interface = WICED_ETHERNET_INTERFACE;

        }
#ifdef WICED_USE_THREAD_INTERFACE
        else if (strncasecmp(argv[1], "thr", 3) == 0)
        {
            dct_network_config.interface = WICED_THREAD_INTERFACE;

        }
#endif
        else
        {
            WPRINT_APP_INFO( ( " --> invalid interface: %s !\r\n", argv[1]) );
            return -1;
        }

        if (wiced_dct_write( &dct_network_config, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_NETWORK);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        if (dct_network_config.interface < WICED_INTERFACE_MAX)
        {
            WPRINT_APP_INFO( ( "%s\r\n", interface_type_string[dct_network_config.interface]) );
        }
        else
        {
            WPRINT_APP_INFO( ( "INVALID\r\n") );
        }
    }
    return 0;
}

int console_dct_network_hostname( int argc, char* argv[] )
{
    char                            new_host_name[ HOSTNAME_SIZE + 1 ];
    int                             new_host_name_len;
    platform_dct_network_config_t   dct_network_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_network_config, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        new_host_name_len = strlcpy((char*)&new_host_name, (char*)argv[1], HOSTNAME_SIZE);
        if (new_host_name_len > HOSTNAME_SIZE)
        {
            WPRINT_APP_INFO( ( " --> invalid hostname: len:%d %s !\r\n", new_host_name_len, argv[1]) );
            return -1;
        }
        strlcpy(dct_network_config.hostname.value, new_host_name, HOSTNAME_SIZE);

        if (wiced_dct_write( &dct_network_config, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_NETWORK);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        strlcpy(new_host_name, (char*)dct_network_config.hostname.value, sizeof(new_host_name));
        WPRINT_APP_INFO( ( "%s\r\n", new_host_name) );
    }
    return 0;
}

int console_dct_network_print( int argc, char* argv[] )
{
    char                            host_name[ HOSTNAME_SIZE + 1 ];
    platform_dct_network_config_t   dct_network_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (wiced_dct_read_with_copy( &dct_network_config, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) ) != WICED_SUCCESS)
    {
        return -1;
    }

    WPRINT_APP_INFO( ( "--- DCT Network ---\r\n") );
    if (dct_network_config.interface < WICED_INTERFACE_MAX)
    {
        WPRINT_APP_INFO( ( " interface : %d %s\r\n", dct_network_config.interface, interface_type_string[dct_network_config.interface]) );
    }
    else
    {
        WPRINT_APP_INFO( ( " interface : %d INVALID\r\n", dct_network_config.interface) );
    }
    strlcpy(host_name, (char*)dct_network_config.hostname.value, sizeof(host_name));
    WPRINT_APP_INFO( ( " hostname  : %s\r\n", host_name) );
    WPRINT_APP_INFO( ( "-----------\r\n") );

    return  0;
}


/******************************************************
 *               Bluetooth DCT Function Definitions
 ******************************************************/
int console_dct_bluetooth_mac( int argc, char* argv[] )
{
    platform_dct_bt_config_t   dct_bt_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        if ((parse_octets(argv[1], (wiced_mac_t*)&dct_bt_config.bluetooth_device_address, 6) != WICED_SUCCESS) &&
            (parse_octets(argv[1], (wiced_mac_t*)&dct_bt_config.bluetooth_device_address, 1) != WICED_SUCCESS))
        {
            WPRINT_APP_INFO( ( " --> unparseable mac !\r\n") );
            return -1;
        }

        if (wiced_dct_write( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        print_mac_address( (wiced_mac_t*) &dct_bt_config.bluetooth_device_address );
        WPRINT_APP_INFO( ("\r\n") );
    }
    return 0;
}

#define BLUETOOTH_NAME_SIZE     248      /* without null termination (taken from platform_dct.h) */
int console_dct_bluetooth_name( int argc, char* argv[] )
{
    char                        new_bt_name[BLUETOOTH_NAME_SIZE + 1];
    int                         new_bt_name_len;
    platform_dct_bt_config_t    dct_bt_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        new_bt_name_len = strlcpy((char*)&new_bt_name, (char*)argv[1], BLUETOOTH_NAME_SIZE);
        if (new_bt_name_len > BLUETOOTH_NAME_SIZE)
        {
            WPRINT_APP_INFO( ( " --> invalid bluetooth name: len:%d %s !\r\n", new_bt_name_len, argv[1]) );
            return -1;
        }
        strlcpy((char*)dct_bt_config.bluetooth_device_name, new_bt_name, BLUETOOTH_NAME_SIZE);

        if (wiced_dct_write( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "%s\r\n", dct_bt_config.bluetooth_device_name ) );
    }
    return 0;
}

int console_dct_bluetooth_class( int argc, char* argv[] )
{
    wiced_mac_t                 new_class;
    platform_dct_bt_config_t    dct_bt_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        if (parse_octets(argv[1], &new_class, 3) != WICED_SUCCESS)
        {
            WPRINT_APP_INFO( ( " --> unparseable class!\r\n") );
            return -1;
        }
        /* it is parsed like a mac address, but only first 3 bytes are the class */
        memcpy(&dct_bt_config.bluetooth_device_class, &new_class, 3);

        if (wiced_dct_write( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "%02x:%02x:%02x\r\n", dct_bt_config.bluetooth_device_class[0], dct_bt_config.bluetooth_device_class[1], dct_bt_config.bluetooth_device_class[2] ) );
    }
    return 0;
}

int console_dct_bluetooth_debug( int argc, char* argv[] )
{
    platform_dct_bt_config_t   dct_bt_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        dct_bt_config.ssp_debug_mode = convert_input_to_wiced_bool(argv[1]);

        if (wiced_dct_write( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "%d\r\n", (dct_bt_config.ssp_debug_mode == WICED_TRUE) ? 1 : 0 ) );
    }
    return 0;
}

int console_dct_bluetooth_print( int argc, char* argv[] )
{
    platform_dct_bt_config_t   dct_bt_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (wiced_dct_read_with_copy( &dct_bt_config, DCT_BT_CONFIG_SECTION, 0, sizeof(dct_bt_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    WPRINT_APP_INFO( ( "--- DCT Bluetooth ---\r\n") );
    WPRINT_APP_INFO( ( " address  : ") );
    print_mac_address( (wiced_mac_t*) &dct_bt_config.bluetooth_device_address );
    WPRINT_APP_INFO( ("\r\n") );
    WPRINT_APP_INFO( ( " name     : %s\r\n", dct_bt_config.bluetooth_device_name ) );
    WPRINT_APP_INFO( ( " class    : %02x:%02x:%02x\r\n", dct_bt_config.bluetooth_device_class[0], dct_bt_config.bluetooth_device_class[1], dct_bt_config.bluetooth_device_class[2] ) );
    WPRINT_APP_INFO( ( " ssp_debug: %s\r\n", (dct_bt_config.ssp_debug_mode == WICED_TRUE) ? "True" : "False" ) );
    WPRINT_APP_INFO( ( "-----------\r\n") );

    return  0;
}



/******************************************************
 *               OTA2 DCT Function Definitions
 ******************************************************/
int console_dct_ota2_force_factory_reset( int argc, char* argv[] )
{
    platform_dct_ota2_config_t   dct_ota2_config;

    UNUSED_PARAMETER(argc);

    if (wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(dct_ota2_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc == 2)
    {
        dct_ota2_config.force_factory_reset = (convert_input_to_wiced_bool(argv[1]) == WICED_FALSE) ? 0 : 1;

        if (wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(dct_ota2_config)) == WICED_SUCCESS)
        {
            console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_OTA2);
        }
        else
        {
            WPRINT_APP_INFO( ( "  Setting failed\r\n") );
            return -1;
        }
    }
    else
    {
        WPRINT_APP_INFO( ( " force_factory_reset : %d\r\n", dct_ota2_config.force_factory_reset) );
    }
    return 0;
}


int console_dct_ota2_print( int argc, char* argv[] )
{
    platform_dct_ota2_config_t   dct_ota2_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(dct_ota2_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    WPRINT_APP_INFO( ( "--- DCT OTA2 ---\r\n") );

    WPRINT_APP_INFO( ( " update_count        : %d\r\n", dct_ota2_config.update_count) );
    WPRINT_APP_INFO( ( " boot_type           : %d %s\r\n", dct_ota2_config.boot_type, (dct_ota2_config.boot_type < OTA2_MAX_BOOT_TYPES) ? ota2_boot_type_string[dct_ota2_config.boot_type] : "ERROR") );
    WPRINT_APP_INFO( ( " force_factory_reset : %d\r\n", dct_ota2_config.force_factory_reset) );
    WPRINT_APP_INFO( ( "-----------\r\n") );

    return  0;
}

/******************************************************
 *               Misc DCT Function Definitions
 ******************************************************/

int console_dct_misc_wifi( int argc, char* argv[] )
{
    int changed_value = 0;
    platform_dct_misc_config_t   dct_misc_config;

    if (wiced_dct_read_with_copy( &dct_misc_config, DCT_MISC_SECTION, 0, sizeof(dct_misc_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    if (argc >= 3)
    {
        if ( stricmp(argv[1], "mesh") == 0)
        {
            uint32_t old_value = dct_misc_config.wifi_flags;

            if ((argc == 3) && isdigit((unsigned char)argv[2][0])) {
                dct_misc_config.wifi_flags &= ~WIFI_FLAG_MESH;
                dct_misc_config.wifi_flags |= (convert_input_to_wiced_bool(argv[2]) == WICED_FALSE) ?  0: WIFI_FLAG_MESH;
                if (old_value != dct_misc_config.wifi_flags)
                {
                    changed_value = 1;
                }
            } else if ((argc == 4) && (stricmp(argv[2], "rb") == 0)) {
                dct_misc_config.wifi_flags &= ~WIFI_FLAG_MESH_MCAST_REBROADCAST;
                dct_misc_config.wifi_flags |= (convert_input_to_wiced_bool(argv[3]) == WICED_FALSE) ?  0: WIFI_FLAG_MESH_MCAST_REBROADCAST;
                if (old_value != dct_misc_config.wifi_flags)
                {
                    changed_value = 1;
                }
            } else if ((argc == 4) && (stricmp(argv[2], "ip") == 0)) {
                if (strnicmp(argv[3], "static", 6) == 0) {
                    dct_misc_config.wifi_flags &= ~WIFI_MESH_DHCP_IP;
                } else {
                    if (strnicmp(argv[3], "dhcp", 4) == 0) {
                        dct_misc_config.wifi_flags |= WIFI_MESH_DHCP_IP;
                    } else {
                        WPRINT_APP_INFO(("Bad ip type.  Use \"dhcp\" or \"static\".\n"));
                    }
                }
                if (old_value != dct_misc_config.wifi_flags)
                {
                    changed_value = 1;
                }
            } else if (strnicmp(argv[2], "gate", 4) == 0) {
                if (argc == 4) {
                    dct_misc_config.wifi_flags &= ~WIFI_FLAG_MESH_GATEWAY;
                    dct_misc_config.wifi_flags |= (convert_input_to_wiced_bool(argv[3]) == WICED_FALSE) ?  0: WIFI_FLAG_MESH_GATEWAY;
                    if (old_value != dct_misc_config.wifi_flags)
                    {
                        changed_value = 1;
                    }
                } else {
                    WPRINT_APP_INFO(( "%s\r\n", dct_misc_config.wifi_flags & WIFI_FLAG_MESH_GATEWAY ? "On" : "Off"));
                }
            } else {
                WPRINT_APP_INFO( ( " Input argument error: %d\n", argc ));
                return -1;
            }
        }

        if (changed_value == 1)
        {
            if (wiced_dct_write( &dct_misc_config, DCT_MISC_SECTION, 0, sizeof(dct_misc_config)) == WICED_SUCCESS)
            {
                console_dct_callback_to_application(CONSOLE_DCT_STRUCT_TYPE_MESH);
            }
            else
            {
                WPRINT_APP_INFO( ( "  Setting failed\r\n") );
                return -1;
            }
        }
    }
    else if (argc == 2)
    {
        if ( stricmp(argv[1], "mesh") == 0)
        {
            WPRINT_APP_INFO( ( " Mesh                   : (0x%lx) %s\r\n", (dct_misc_config.wifi_flags & WIFI_FLAG_MESH), ((dct_misc_config.wifi_flags & WIFI_FLAG_MESH) == WIFI_FLAG_MESH) ? "On" : "Off") );
            WPRINT_APP_INFO( ( " Mesh MCAST Rebroadcast : (0x%lx) %s\r\n", (dct_misc_config.wifi_flags & WIFI_FLAG_MESH_MCAST_REBROADCAST), ((dct_misc_config.wifi_flags & WIFI_FLAG_MESH_MCAST_REBROADCAST) == WIFI_FLAG_MESH_MCAST_REBROADCAST) ? "On" : "Off") );
            WPRINT_APP_INFO( ( " Mesh Gateway           : (0x%lx) %s\r\n", (dct_misc_config.wifi_flags & WIFI_FLAG_MESH_GATEWAY), dct_misc_config.wifi_flags & WIFI_FLAG_MESH_GATEWAY ? "On" : "Off") );
            WPRINT_APP_INFO( ( " IP Addressing Type     : (0x%lx) %s\r\n", (dct_misc_config.wifi_flags & WIFI_MESH_DHCP_IP), dct_misc_config.wifi_flags & WIFI_MESH_DHCP_IP ? "DHCP" : "Static Addressing") );
        }
        else
        {
            WPRINT_APP_INFO( ( " unknown flag: %s\r\n", argv[2] ));
        }
    }
    return 0;
}

int console_dct_misc_print( int argc, char* argv[] )
{
    platform_dct_misc_config_t   dct_misc_config;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (wiced_dct_read_with_copy( &dct_misc_config, DCT_MISC_SECTION, 0, sizeof(dct_misc_config) ) != WICED_SUCCESS)
    {
        return -1;
    }

    WPRINT_APP_INFO( ( "--- DCT Misc ---\r\n") );

    WPRINT_APP_INFO( ( " WiFi Flags       : 0x%lx\r\n", dct_misc_config.wifi_flags) );
    WPRINT_APP_INFO( ( "            MESH  : 0x%lx\r\n", dct_misc_config.wifi_flags & WIFI_FLAG_MESH_ALL_FLAGS) );
    WPRINT_APP_INFO( ( "-----------\r\n") );

    return  0;
}
