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

#include "wiced.h"
#include "command_console.h"

#include "ssdp_app.h"
#include "ssdp_app_config.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define WIFI_DCT_SECURITY_KEY_DIRTY     (1 << 0)
#define WIFI_DCT_SECURITY_TYPE_DIRTY    (1 << 1)
#define WIFI_DCT_CHANNEL_DIRTY          (1 << 2)
#define WIFI_DCT_SSID_DIRTY             (1 << 3)
#define WIFI_DCT_MAC_ADDR_DIRTY         (1 << 4)

#define MAC_STR_LEN                     (18)
/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    CONFIG_CMD_NONE         = 0,
    CONFIG_CMD_HELP,
    CONFIG_CMD_MAC_ADDR,
    CONFIG_CMD_NETWORK_CHANNEL,
    CONFIG_CMD_NETWORK_NAME,
    CONFIG_CMD_NETWORK_PASSPHRASE,
    CONFIG_CMD_NETWORK_SECURITY_TYPE,
    CONFIG_CMD_SAVE,

    CONFIG_CMD_MAX,
} CONFIG_CMDS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct cmd_lookup_s {
        char *cmd;
        uint32_t event;
} cmd_lookup_t;

typedef struct security_lookup_s {
    char     *name;
    wiced_security_t sec_type;
} security_lookup_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t app_save_network_dct(application_info_t* app_info);
static wiced_result_t app_save_wifi_dct(application_info_t* app_info);
static void app_print_network_info(application_info_t* app_info);
static void app_print_wifi_info(application_info_t* app_info);
static wiced_bool_t app_get_channel_band(int channel, int* band);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* dirty flags for determining what to save */
static uint32_t net_dct_dirty = 0;
static uint32_t wifi_dct_dirty = 0;


static cmd_lookup_t config_command_lookup[] = {
        { "help",               CONFIG_CMD_HELP                 },
        { "?",                  CONFIG_CMD_HELP                 },

        { "mac_addr",           CONFIG_CMD_MAC_ADDR             },
        { "mac",                CONFIG_CMD_MAC_ADDR             },

        { "network_channel",    CONFIG_CMD_NETWORK_CHANNEL      },
        { "network_chan",       CONFIG_CMD_NETWORK_CHANNEL      },
        { "network_name",       CONFIG_CMD_NETWORK_NAME         },
        { "network_passphrase", CONFIG_CMD_NETWORK_PASSPHRASE   },
        { "network_pass",       CONFIG_CMD_NETWORK_PASSPHRASE   },
        { "network_security",   CONFIG_CMD_NETWORK_SECURITY_TYPE},
        { "network_sec",        CONFIG_CMD_NETWORK_SECURITY_TYPE},

        { "net_chan",           CONFIG_CMD_NETWORK_CHANNEL      },
        { "net_name",           CONFIG_CMD_NETWORK_NAME         },
        { "net_pass",           CONFIG_CMD_NETWORK_PASSPHRASE   },
        { "net_sec",            CONFIG_CMD_NETWORK_SECURITY_TYPE},

        { "ssid",               CONFIG_CMD_NETWORK_NAME         },
        { "pass",               CONFIG_CMD_NETWORK_PASSPHRASE   },

        { "save",               CONFIG_CMD_SAVE                 },

        { "", CONFIG_CMD_NONE },
};

static security_lookup_t app_security_name_table[] =
{
        { "open",       WICED_SECURITY_OPEN           },
        { "none",       WICED_SECURITY_OPEN           },
        { "wep",        WICED_SECURITY_WEP_PSK        },
        { "shared",     WICED_SECURITY_WEP_SHARED     },
        { "wpatkip",    WICED_SECURITY_WPA_TKIP_PSK   },
        { "wpaaes",     WICED_SECURITY_WPA_AES_PSK    },
        { "wpa2aes",    WICED_SECURITY_WPA2_AES_PSK   },
        { "wpa2tkip",   WICED_SECURITY_WPA2_TKIP_PSK  },
        { "wpa2mix",    WICED_SECURITY_WPA2_MIXED_PSK },
        { "wpa2aesent", WICED_SECURITY_WPA2_AES_ENT   },
        { "ibss",       WICED_SECURITY_IBSS_OPEN      },
        { "wpsopen",    WICED_SECURITY_WPS_OPEN       },
        { "wpsnone",    WICED_SECURITY_WPS_OPEN       },
        { "wpsaes",     WICED_SECURITY_WPS_SECURE     },
        { "invalid",    WICED_SECURITY_UNKNOWN        },
};

static uint32_t default_24g_channel_list[] =
{
        /* 2.4 GHz channels */
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static uint32_t default_5g_channel_list[] =
{
        /* 5 GHz non-DFS channels */
        36, 40, 44, 48,

        /* DFS channels */
        52, 56, 60, 64,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140,

        /* more non-DFS channels */
        149, 153, 157, 161, 165,
};


/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t app_save_network_dct(application_info_t* app_info)
{
    return wiced_dct_write( (void*) app_info->dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
}

static wiced_result_t app_save_wifi_dct(application_info_t* app_info)
{
    return wiced_dct_write( (void*) app_info->dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
}

/* validate the channel and return the band
 *
 * in - channel
 * out - band
 *
 * return WICED_TRUE or WICED_FALSE
 */

static wiced_bool_t app_get_channel_band(int channel, int* band)
{
    uint32_t i;

    if (band == NULL)
    {
        return WICED_FALSE;
    }

    /* TODO: get country code and channels in country */

    for (i = 0; i < sizeof(default_24g_channel_list)/sizeof(uint32_t) ; i++)
    {
        if (default_24g_channel_list[i] == (uint32_t)channel)
        {
            *band = WICED_802_11_BAND_2_4GHZ;
            return WICED_TRUE;
        }
    }

    for (i = 0; i < sizeof(default_5g_channel_list)/sizeof(uint32_t) ; i++)
    {
        if (default_5g_channel_list[i] == (uint32_t)channel)
        {
            *band = WICED_802_11_BAND_5GHZ;
            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

static wiced_security_t app_parse_security_type(char* security_str)
{
    int table_index;

    for (table_index = 0; table_index < sizeof(app_security_name_table)/sizeof(security_lookup_t); table_index++)
    {
        if (strcasecmp(app_security_name_table[table_index].name, security_str) == 0)
        {
            return app_security_name_table[table_index].sec_type;
        }
    }

    return WICED_SECURITY_UNKNOWN;
}

static char* app_get_security_type_name(wiced_security_t type)
{
    int table_index;

    for (table_index = 0; table_index < sizeof(app_security_name_table)/sizeof(security_lookup_t); table_index++)
    {
        if (app_security_name_table[table_index].sec_type == type)
        {
            return app_security_name_table[table_index].name;
        }
    }

    return "";
}

static wiced_bool_t app_parse_mac_addr(char* in, wiced_mac_t* mac)
{
    char* colon;
    wiced_mac_t new_mac = {{0}};
    int octet_count;
    wiced_bool_t allow_one_octet = WICED_FALSE;

    if ((in == NULL) || (mac == NULL))
    {
        return WICED_FALSE;
    }

    /* We want to allow a "shortcut" of only supplying the last octet
     *  - Only if the DCT mac is non-zero
     */
    if (memcmp(&new_mac, mac, sizeof(wiced_mac_t)) != 0)
    {
        allow_one_octet = WICED_TRUE;
    }

    octet_count = 0;
    while((in != NULL) && (*in != 0) && (octet_count < 6))
    {
        colon = strchr(in, ':');
        if (colon != NULL)
        {
            *colon++ = 0;
        }

        /* convert the hex data */
        new_mac.octet[octet_count++] = hex_str_to_int( in );

        in = colon;
    }

    if(octet_count == 6)
    {
        memcpy(mac, &new_mac, sizeof(wiced_mac_t));

        return WICED_TRUE;
    }
    else if ((allow_one_octet == WICED_TRUE) && (octet_count == 1))
    {
        /* only one octet provided! */
        mac->octet[5] = new_mac.octet[0];
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

static wiced_bool_t app_get_mac_addr_text(wiced_mac_t *mac, char* out, int out_len)
{
    if((mac == NULL) || (out == NULL) || (out_len < MAC_STR_LEN))
    {
        return WICED_FALSE;
    }

    sprintf(out, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac->octet[0], mac->octet[1], mac->octet[2], mac->octet[3], mac->octet[4], mac->octet[5]);

    return WICED_TRUE;
}

static void app_config_command_help( void )
{
    WPRINT_APP_INFO(("Config commands:\r\n"));
    WPRINT_APP_INFO(("    config                              : output current config\r\n"));
    WPRINT_APP_INFO(("    config <?|help>                     : show this list\r\n"));

    WPRINT_APP_INFO(("    config mac_addr <xx:xx:xx:xx:xx:xx> : xx:xx:xx:xx:xx:xx = new MAC address\r\n"));
    WPRINT_APP_INFO(("    config mac <xx:xx:xx:xx:xx:xx>      : Shortcut:\r\n"));
    WPRINT_APP_INFO(("    config mac <xx>                     :   enter 1 octet to change last octet\r\n"));

    WPRINT_APP_INFO(("    config network_channel <xxx>        : xxx = channel\r\n"));
    WPRINT_APP_INFO(("           net_chan <xxx>               :  (1-11,36,40,44,48,52,56,60,64,\r\n"));
    WPRINT_APP_INFO(("                                        :   100,104,108,112,116,120,124,128,\r\n"));
    WPRINT_APP_INFO(("                                        :   132,136,140,149,153,157,161,165)\r\n"));
    WPRINT_APP_INFO(("    config network_name <ssid_name>     : name of AP (max %d characters)\r\n", SSID_NAME_SIZE));
    WPRINT_APP_INFO(("           net_name <ssid_name>\r\n"));
    WPRINT_APP_INFO(("           ssid <ssid_name>\r\n"));
    WPRINT_APP_INFO(("    config network_passphrase <pass>    : passkey/password (max %d characters)\r\n", SECURITY_KEY_SIZE));
    WPRINT_APP_INFO(("           net_pass <pass>\r\n"));
    WPRINT_APP_INFO(("           pass <pass>\r\n"));
    WPRINT_APP_INFO(("    config network_security <type>      : security type is one of: \r\n"));
    WPRINT_APP_INFO(("           net_sec <type>               :   open,none,ibss,wep,wepshared,wpatkip,wpaaes,\r\n"));
    WPRINT_APP_INFO(("                                        :   wpa2tkip,wpa2aes,wpa2mix,wpsopen,wpsaes\r\n"));

    WPRINT_APP_INFO(("    config save                         : save data to flash NOTE: Changes not \r\n"));
    WPRINT_APP_INFO(("                                        :   automatically saved to flash!\r\n"));
}

void app_set_config(application_info_t* app_info, int argc, char *argv[])
{
    int i;
    CONFIG_CMDS_T cmd;

    if (argc < 2)
    {
        app_config_print_info(app_info);
        return;
    }

    cmd = CONFIG_CMD_NONE;
    for (i = 0; i < (sizeof(config_command_lookup) / sizeof(cmd_lookup_t)); ++i)
    {
        if (strcasecmp(config_command_lookup[i].cmd, argv[1]) == 0)
        {
            cmd = config_command_lookup[i].event;
            break;
        }
    }

    switch( cmd )
    {
        case CONFIG_CMD_HELP:
            app_config_command_help();
            break;

        case CONFIG_CMD_NETWORK_NAME:
        {
            char name[SSID_NAME_SIZE + 1];
            int  name_len;
            if (argc > 2)
            {
                if (strlen(argv[2]) > SSID_NAME_SIZE)
                {
                    WPRINT_APP_INFO(("Error: too long (max: %d)\r\n", SSID_NAME_SIZE));
                    break;
                }

                memset(name, 0, sizeof(name));
                memcpy(name, app_info->dct_wifi->stored_ap_list[0].details.SSID.value, app_info->dct_wifi->stored_ap_list[0].details.SSID.length);
                if(strcmp(name, argv[2])  != 0)
                {
                    WPRINT_APP_DEBUG(("name: %s -> %s\r\n", name, argv[2]));
                    memset(app_info->dct_wifi->stored_ap_list[0].details.SSID.value, 0, sizeof(app_info->dct_wifi->stored_ap_list[0].details.SSID.value));
                    name_len = strlen(argv[2]);
                    memcpy(app_info->dct_wifi->stored_ap_list[0].details.SSID.value, argv[2], name_len);
                    app_info->dct_wifi->stored_ap_list[0].details.SSID.length = name_len;
                    wifi_dct_dirty |= WIFI_DCT_SSID_DIRTY;
                }
            }
            memset(name, 0, sizeof(name));
            memcpy(name, app_info->dct_wifi->stored_ap_list[0].details.SSID.value, app_info->dct_wifi->stored_ap_list[0].details.SSID.length);
            WPRINT_APP_INFO(("Network name: (%d) %s\r\n", app_info->dct_wifi->stored_ap_list[0].details.SSID.length, name));
            break;
        }

        case CONFIG_CMD_MAC_ADDR:
        {
            char mac_str[20];
            if (argc > 2)
            {
                char new_mac_str[20];
                wiced_mac_t new_mac;

                /* start with current DCT mac value to allow one-octet shortcut */
                memcpy(&new_mac, &app_info->dct_wifi->mac_address, sizeof(wiced_mac_t));
                if (app_parse_mac_addr(argv[2], &new_mac) != WICED_TRUE)
                {
                    WPRINT_APP_INFO(("Error: %s not a valid mac.\r\n", argv[2]));
                    break;
                }

                if (memcmp(&new_mac, &app_info->dct_wifi->mac_address, sizeof(wiced_mac_t)) != 0)
                {
                    app_get_mac_addr_text(&app_info->dct_wifi->mac_address, mac_str, sizeof(mac_str));
                    app_get_mac_addr_text(&new_mac, new_mac_str, sizeof(new_mac_str));
                    WPRINT_APP_DEBUG(("mac_addr: %s -> %s\r\n", mac_str, new_mac_str));
                    memcpy(&app_info->dct_wifi->mac_address, &new_mac, sizeof(wiced_mac_t));
                    wifi_dct_dirty |= WIFI_DCT_MAC_ADDR_DIRTY;
                    WPRINT_APP_INFO(("After save, you MUST reboot for MAC address change to take effect\r\n"));
                }
            }
            app_get_mac_addr_text(&app_info->dct_wifi->mac_address, mac_str, sizeof(mac_str));
            WPRINT_APP_INFO(("MAC address:%s\r\n", mac_str));

            {
                wiced_mac_t wiced_mac;
                wwd_wifi_get_mac_address( &wiced_mac, app_info->dct_network->interface );
                app_get_mac_addr_text(&wiced_mac, mac_str, sizeof(mac_str));
                if (memcmp(&wiced_mac, &app_info->dct_wifi->mac_address, sizeof(wiced_mac_t) != 0))
                {
                    WPRINT_APP_INFO(("  WICED MAC:%s\r\n", mac_str));
                }
            }
            break;
        }

        case CONFIG_CMD_NETWORK_CHANNEL:
        {
            int new_channel, new_band;
            if (argc > 2)
            {
                new_channel = atoi(argv[2]);
                if (app_get_channel_band(new_channel, &new_band) != WICED_TRUE)
                {
                    /* TODO: get country code for output */
                    WPRINT_APP_INFO(("Error: %d not a valid channel.\r\n", new_channel));
                    break;
                }
                if ((app_info->dct_wifi->stored_ap_list[0].details.channel != new_channel) ||
                    (app_info->dct_wifi->stored_ap_list[0].details.band != new_band))
                {
                    WPRINT_APP_DEBUG(("channel: %d -> %d\r\n  Band: %d -> %d \r\n",
                            app_info->dct_wifi->stored_ap_list[0].details.channel, new_channel,
                            app_info->dct_wifi->stored_ap_list[0].details.band, new_band));
                    app_info->dct_wifi->stored_ap_list[0].details.channel = new_channel;
                    app_info->dct_wifi->stored_ap_list[0].details.band = new_band;
                    wifi_dct_dirty |= WIFI_DCT_CHANNEL_DIRTY;
                }
            }
            WPRINT_APP_INFO(("Network channel:%d  band:%s\r\n", app_info->dct_wifi->stored_ap_list[0].details.channel,
                    (app_info->dct_wifi->stored_ap_list[0].details.band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz"));
            break;
        }

        case CONFIG_CMD_NETWORK_SECURITY_TYPE:
            if (argc > 2)
            {
                wiced_security_t new_sec_type;
                new_sec_type = app_parse_security_type(argv[2]);
                if (new_sec_type == WICED_SECURITY_UNKNOWN)
                {
                    WPRINT_APP_INFO(("Error: unknown security type: %s\r\n", (argv[2][0] != 0) ? argv[2] : ""));
                    break;
                }
                if (app_info->dct_wifi->stored_ap_list[0].details.security != new_sec_type)
                {
                    WPRINT_APP_DEBUG(("network security %s -> %s\r\n",
                            app_get_security_type_name( app_info->dct_wifi->stored_ap_list[0].details.security),
                            app_get_security_type_name(new_sec_type)));
                    app_info->dct_wifi->stored_ap_list[0].details.security = new_sec_type;
                    wifi_dct_dirty |= WIFI_DCT_SECURITY_TYPE_DIRTY;
                }
            }
            WPRINT_APP_INFO(("Network security type: %s\r\n", app_get_security_type_name(app_info->dct_wifi->stored_ap_list[0].details.security)));
            break;

        case CONFIG_CMD_NETWORK_PASSPHRASE:
        {
            char pass[SECURITY_KEY_SIZE + 1];
            if (argc > 2)
            {
                if (strlen(argv[2]) > SECURITY_KEY_SIZE)
                {
                    WPRINT_APP_INFO(("Error: too long (max %d)\r\n", SECURITY_KEY_SIZE));
                    break;
                }
                memset(pass, 0, sizeof(pass));
                memcpy(pass, app_info->dct_wifi->stored_ap_list[0].security_key, app_info->dct_wifi->stored_ap_list[0].security_key_length);
                if(strcmp(pass, argv[2])  != 0)
                {
                    WPRINT_APP_DEBUG(("passphrase: %s -> %s \r\n", pass, argv[2]));
                    memset(pass, 0, sizeof(pass));
                    memcpy(pass, argv[2], SECURITY_KEY_SIZE);
                    memset(app_info->dct_wifi->stored_ap_list[0].security_key, 0, sizeof(app_info->dct_wifi->stored_ap_list[0].security_key));
                    memcpy(app_info->dct_wifi->stored_ap_list[0].security_key, argv[2], SECURITY_KEY_SIZE);
                    app_info->dct_wifi->stored_ap_list[0].security_key_length = strlen(pass);
                    wifi_dct_dirty |= WIFI_DCT_SECURITY_KEY_DIRTY;
                }
            }
            memset(pass, 0, sizeof(pass));
            memcpy(pass, app_info->dct_wifi->stored_ap_list[0].security_key, app_info->dct_wifi->stored_ap_list[0].security_key_length);
            WPRINT_APP_INFO(("Network passphrase: %s \r\n", pass));
            break;
        }

        case CONFIG_CMD_SAVE:
            if (net_dct_dirty != 0)
            {
                WPRINT_APP_INFO(("Saving Network DCT:\r\n"));
                app_save_network_dct(app_info);
                net_dct_dirty = 0;
                app_print_network_info(app_info);
            }
            if (wifi_dct_dirty != 0)
            {
                WPRINT_APP_INFO(("Saving WiFi DCT:\r\n"));
                app_save_wifi_dct(app_info);
                if ((wifi_dct_dirty & WIFI_DCT_MAC_ADDR_DIRTY) != 0)
                {
                    WPRINT_APP_INFO(("You MUST reboot for MAC address change to take effect\r\n"));
                }
                wifi_dct_dirty = 0;
                app_print_wifi_info(app_info);
            }
            break;

        default:
            WPRINT_APP_INFO(("Unrecognized config command: %s\r\n", (argv[1][0] != 0) ? argv[1] : ""));
            app_config_command_help();
            break;
    }
}

static void app_print_network_info(application_info_t* app_info)
{
    wiced_ip_address_t ip_addr;

    WPRINT_APP_INFO(("  Network DCT:\r\n"));
    WPRINT_APP_INFO(("       Interface: %s\r\n",
           (app_info->dct_network->interface == (wiced_interface_t)WWD_STA_INTERFACE)      ? "STA" :
           (app_info->dct_network->interface == (wiced_interface_t)WWD_AP_INTERFACE)       ? "AP" :
           (app_info->dct_network->interface == (wiced_interface_t)WWD_ETHERNET_INTERFACE) ? "Ethernet" :
           "Unknown"));
    wiced_ip_get_ipv4_address(app_info->dct_network->interface, &ip_addr);
    WPRINT_APP_INFO(("         IP addr: %d.%d.%d.%d\r\n",
           (int)((ip_addr.ip.v4 >> 24) & 0xFF), (int)((ip_addr.ip.v4 >> 16) & 0xFF),
           (int)((ip_addr.ip.v4 >> 8) & 0xFF),  (int)(ip_addr.ip.v4 & 0xFF)));
}

static void app_print_wifi_info(application_info_t* app_info)
{
    wiced_security_t sec;
    uint32_t channel;
    int band;
    char mac_str[MAC_STR_LEN], wiced_mac_str[MAC_STR_LEN];
    wiced_mac_t wiced_mac;

    wwd_wifi_get_mac_address( &wiced_mac, WICED_STA_INTERFACE );
    app_get_mac_addr_text(&wiced_mac, wiced_mac_str, sizeof(wiced_mac_str));
    app_get_mac_addr_text(&app_info->dct_wifi->mac_address, mac_str, sizeof(mac_str));

    if (app_info->dct_network->interface == WICED_STA_INTERFACE)
    {
        sec = app_info->dct_wifi->stored_ap_list[0].details.security;
        WPRINT_APP_INFO(("  WiFi DCT:\r\n"));

        WPRINT_APP_INFO((" WICED MAC (STA): %s\r\n", wiced_mac_str));

        WPRINT_APP_INFO(("             MAC: %s %c\r\n", mac_str,
                                                        (wifi_dct_dirty & WIFI_DCT_MAC_ADDR_DIRTY) ? '*' : ' '));
        WPRINT_APP_INFO(("            SSID: %.*s %c\r\n", app_info->dct_wifi->stored_ap_list[0].details.SSID.length,
                                                        app_info->dct_wifi->stored_ap_list[0].details.SSID.value,
                                                        (wifi_dct_dirty & WIFI_DCT_SSID_DIRTY) ? '*' : ' '));
        WPRINT_APP_INFO(("        Security: %s %c\r\n", app_get_security_type_name(sec), (wifi_dct_dirty & WIFI_DCT_SECURITY_TYPE_DIRTY) ? '*' : ' '));

        if (app_info->dct_wifi->stored_ap_list[0].details.security != WICED_SECURITY_OPEN)
        {
            WPRINT_APP_INFO(("      Passphrase: %.*s %c\r\n", app_info->dct_wifi->stored_ap_list[0].security_key_length,
                   app_info->dct_wifi->stored_ap_list[0].security_key, (wifi_dct_dirty & WIFI_DCT_SECURITY_KEY_DIRTY) ? '*' : ' '));
        }
        else
        {
            WPRINT_APP_INFO(("      Passphrase: none\r\n"));
        }

        channel = app_info->dct_wifi->stored_ap_list[0].details.channel;
        band = app_info->dct_wifi->stored_ap_list[0].details.band;
        WPRINT_APP_INFO(("         Channel: %d %c\r\n", (int)channel, (wifi_dct_dirty & WIFI_DCT_CHANNEL_DIRTY) ? '*' : ' '));
        WPRINT_APP_INFO(("            Band: %s %c\r\n", (band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz",
                                                        (wifi_dct_dirty & WIFI_DCT_CHANNEL_DIRTY) ? '*' : ' '));
    }
    else
    {
        /*
         * Nothing for AP interface yet.
         */
    }
}

static void app_print_current_info(application_info_t* app_info)
{
    uint32_t channel;

    wiced_wifi_get_channel(&channel);
    WPRINT_APP_INFO((" Current:\r\n"));
    WPRINT_APP_INFO(("         Channel: %lu\r\n            Band: %s\r\n",
                     channel, channel <= 13 ? "2.4GHz" : "5GHz"));
}

void app_config_print_info(application_info_t* app_info)
{
    WPRINT_APP_INFO(("\r\nConfig Info: * = dirty\r\n"));
    app_print_network_info(app_info);
    app_print_wifi_info(app_info);
    app_print_current_info(app_info);
    WPRINT_APP_INFO(("\r\n"));
}
