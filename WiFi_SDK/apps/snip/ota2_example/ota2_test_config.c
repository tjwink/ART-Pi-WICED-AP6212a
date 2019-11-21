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

#include <ctype.h>
#include "wiced.h"
#include "wiced_log.h"
#include "platform_audio.h"
#include "command_console.h"

#include "ota2_test.h"
#include "ota2_test_config.h"
#include "wiced_ota2_image.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define APP_DCT_OTA2_REBOOT_DIRTY   (1 << 0)
#define APP_DCT_OTA2_URI_DIRTY      (1 << 1)

/******************************************************
 *                    Constants
 ******************************************************/

#define MAC_STR_LEN                     (18)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    CONFIG_CMD_NONE         = 0,
    CONFIG_CMD_HELP,
    CONFIG_CMD_SAVE,
    CONFIG_CMD_REBOOT_AFTER_DOWNLOAD,
    CONFIG_CMD_DOWNLOAD_URI,

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
static void ota_test_print_app_info(ota2_data_t* player);
static void ota_test_print_network_info(ota2_data_t* player);
static void ota_test_print_wifi_info(ota2_data_t* player);

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint32_t app_dct_dirty;

static cmd_lookup_t config_command_lookup[] = {
        { "help",               CONFIG_CMD_HELP                  },
        { "?",                  CONFIG_CMD_HELP                  },
        { "save",               CONFIG_CMD_SAVE                  },
        { "ota2_reboot",        CONFIG_CMD_REBOOT_AFTER_DOWNLOAD },
        { "ota2_uri",           CONFIG_CMD_DOWNLOAD_URI          },
        { "", CONFIG_CMD_NONE },
};

static void ota2_config_command_help( void )
{
    wiced_log_printf( "Config commands:\r\n");
    wiced_log_printf( "    config                              : output current config\r\n");
    wiced_log_printf( "    config <?|help>                     : show this list\r\n");
    wiced_log_printf( "    ota2_reboot  <0|1|y|n>              : Reboot after successful OTA2 download\r\n");
    wiced_log_printf( "    ota2_uri  <new_URI>                 : Location of OTA2 image file\r\n");
    wiced_log_printf( "    save                                : save changes\r\n");
}

static security_lookup_t ota2_security_name_table[] =
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
 *               Function Definitions
 ******************************************************/

static char* ota2_get_security_type_name(wiced_security_t type)
{
    int table_index;

    for (table_index = 0; table_index < sizeof(ota2_security_name_table)/sizeof(security_lookup_t); table_index++)
    {
        if (ota2_security_name_table[table_index].sec_type == type)
        {
            return ota2_security_name_table[table_index].name;
        }
    }

    return "";
}

static wiced_bool_t ota2_get_mac_addr_text(wiced_mac_t *mac, char* out, int out_len)
{
    if((mac == NULL) || (out == NULL) || (out_len < MAC_STR_LEN))
    {
        return WICED_FALSE;
    }

    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac->octet[0], mac->octet[1], mac->octet[2], mac->octet[3], mac->octet[4], mac->octet[5]);

    return WICED_TRUE;
}

void ota2_set_config(ota2_data_t* player, int argc, char *argv[])
{
    int i;
    CONFIG_CMDS_T           cmd;

    if (argc < 2)
    {
        ota2_config_print_info(player);
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
            ota2_config_command_help();
            break;
        case CONFIG_CMD_SAVE:
            ota2_save_config(player);
             break;
         case CONFIG_CMD_REBOOT_AFTER_DOWNLOAD:
        {
            int ota2_reboot_after_download;
            ota2_reboot_after_download = player->dct_app->ota2_reboot_after_download;
            if (argc > 2)
            {
                if ((strcasecmp(argv[2], "no") == 0) || (tolower((int)argv[2][0]) == 'n'))
                {
                    ota2_reboot_after_download = 0;
                }
                else if ((strcasecmp(argv[2], "yes") == 0)  || (tolower((int)argv[2][0]) == 'y'))
                {
                    ota2_reboot_after_download = 1;
                }
                else
                {
                    ota2_reboot_after_download = atoi(argv[2]);
                }
                ota2_reboot_after_download = (ota2_reboot_after_download == 0) ? 0 : 1;
                if (player->dct_app->ota2_reboot_after_download != ota2_reboot_after_download)
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2 Reboot after download: %d -> %d\r\n", player->dct_app->ota2_reboot_after_download, ota2_reboot_after_download);
                    player->dct_app->ota2_reboot_after_download = ota2_reboot_after_download;
                    app_dct_dirty |= APP_DCT_OTA2_REBOOT_DIRTY;
                }
            }
            wiced_log_printf( "OTA2 Reboot after download: %d\r\n", player->dct_app->ota2_reboot_after_download);
        }
        break;
         case CONFIG_CMD_DOWNLOAD_URI:
         {
             if (argc > 2)
             {
                 int ota2_uri_len = strlen(argv[2]);
                 if (ota2_uri_len > sizeof(player->dct_app->ota2_update_uri))
                 {
                     wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Error: too long (max %d)\r\n", sizeof(player->dct_app->ota2_update_uri));
                     break;
                 }
                 if ((ota2_uri_len != strlen(player->dct_app->ota2_update_uri)) ||
                      (strncmp(player->dct_app->ota2_update_uri, argv[2], ota2_uri_len) != 0) )
                 {
                     wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2 URI: %s -> %s \r\n",
                             player->dct_app->ota2_update_uri, argv[2]);
                     strlcpy(player->dct_app->ota2_update_uri, argv[2], sizeof(player->dct_app->ota2_update_uri));

                     app_dct_dirty |= APP_DCT_OTA2_URI_DIRTY;
                 }
             }
             wiced_log_printf( "OTA2 Default Update URI: %s \r\n", player->dct_app->ota2_update_uri);
             break;
         }
         break;
        default:
            wiced_log_printf( "Unrecognized config command: %s\r\n", (argv[1][0] != 0) ? argv[1] : "");
            ota2_config_command_help();
            break;
    }
}

static void ota_test_print_app_info(ota2_data_t* player)
{
    wiced_log_printf( "OTA2 App DCT:\r\n");
    wiced_log_printf( "           reboot_count: %ld\r\n", player->dct_app->reboot_count);
    wiced_log_printf( "          version major: %d\r\n", player->dct_app->ota2_major_version);
    wiced_log_printf( "          version minor: %d\r\n", player->dct_app->ota2_minor_version);
    wiced_log_printf( "  reboot after download: %d %c\r\n", player->dct_app->ota2_reboot_after_download, ( app_dct_dirty & APP_DCT_OTA2_REBOOT_DIRTY) ? '*' : ' ');
    wiced_log_printf( "     Default Update URI: %s %c\r\n", player->dct_app->ota2_update_uri,  ( app_dct_dirty & APP_DCT_OTA2_URI_DIRTY) ? '*' : ' ');

}

static void ota2_test_print_dct_info(void)
{
    platform_dct_ota2_config_t   dct_ota2_config;

    if (wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(dct_ota2_config) ) != WICED_SUCCESS)
    {
        return;
    }

    wiced_log_printf(  "OTA2 System DCT:\r\n");

    wiced_log_printf(  "  update_count        : %d\r\n", dct_ota2_config.update_count);
    wiced_log_printf(  "  boot_type           : %d %s\r\n", dct_ota2_config.boot_type, (dct_ota2_config.boot_type < OTA2_MAX_BOOT_TYPES) ? ota2_boot_type_string[dct_ota2_config.boot_type] : "ERROR");
    wiced_log_printf(  "  force_factory_reset : %d\r\n", dct_ota2_config.force_factory_reset);
}

static void ota_test_print_network_info(ota2_data_t* player)
{
    wiced_ip_address_t ip_addr;

    wiced_log_printf( "Network DCT:\r\n");
    wiced_log_printf( "       Interface: %s\r\n",
           (player->dct_network->interface == (wiced_interface_t)WWD_STA_INTERFACE)      ? "STA" :
           (player->dct_network->interface == (wiced_interface_t)WWD_AP_INTERFACE)       ? "AP" :
           (player->dct_network->interface == (wiced_interface_t)WWD_ETHERNET_INTERFACE) ? "Ethernet" :
           "Unknown");
    wiced_log_printf( "        Hostname: %s\r\n", player->dct_network->hostname.value);
    wiced_ip_get_ipv4_address(player->dct_network->interface, &ip_addr);
    wiced_log_printf( "         IP addr: " IPV4_PRINT_FORMAT "\r\n", IPV4_SPLIT_TO_PRINT(ip_addr) );
}

static void ota_test_print_wifi_info(ota2_data_t* player)
{
    wiced_security_t sec;
    uint32_t channel;
    int band;
    char mac_str[MAC_STR_LEN], wiced_mac_str[MAC_STR_LEN];
    wiced_mac_t wiced_mac;

    wwd_wifi_get_mac_address( &wiced_mac, WICED_STA_INTERFACE );
    ota2_get_mac_addr_text(&wiced_mac, wiced_mac_str, sizeof(wiced_mac_str));
    ota2_get_mac_addr_text(&player->dct_wifi->mac_address, mac_str, sizeof(mac_str));

    if (player->dct_network->interface == WICED_STA_INTERFACE)
    {
        sec = player->dct_wifi->stored_ap_list[0].details.security;
        wiced_log_printf( "WiFi DCT:\r\n");

        wiced_log_printf( "  WICED MAC (STA): %s\r\n", wiced_mac_str);

        wiced_log_printf( "              MAC: %s\r\n", mac_str);
        wiced_log_printf( "             SSID: %.*s\r\n", player->dct_wifi->stored_ap_list[0].details.SSID.length,
                                                        player->dct_wifi->stored_ap_list[0].details.SSID.value);
        wiced_log_printf( "         Security: %s\r\n", ota2_get_security_type_name(sec));

        if (player->dct_wifi->stored_ap_list[0].details.security != WICED_SECURITY_OPEN)
        {
            wiced_log_printf( "       Passphrase: %.*s\r\n", player->dct_wifi->stored_ap_list[0].security_key_length,
                                                                           player->dct_wifi->stored_ap_list[0].security_key);
        }
        else
        {
            wiced_log_printf( "       Passphrase: none\r\n");
        }

        channel = player->dct_wifi->stored_ap_list[0].details.channel;
        band = player->dct_wifi->stored_ap_list[0].details.band;
        wiced_log_printf( "          Channel: %d\r\n", (int)channel);
        wiced_log_printf( "             Band: %s\r\n", (band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz");
    }
    else
    {
        /*
         * Nothing for AP interface yet.
         */
    }
}

static void ota_test_print_current_info(ota2_data_t* player)
{
    uint32_t channel;

    wiced_wifi_get_channel(&channel);
    wiced_log_printf( " Current:\r\n");
    wiced_log_printf( "         Channel: %lu\r\n            Band: %s\r\n",
                     channel, channel <= 13 ? "2.4GHz" : "5GHz");
}

void ota2_config_print_info(ota2_data_t* player)
{
    wiced_log_printf( "\r\nConfig Info: * = dirty\r\n");
    ota_test_print_app_info(player);
    ota2_test_print_dct_info();
    ota_test_print_network_info(player);
    ota_test_print_wifi_info(player);
    ota_test_print_current_info(player);
    wiced_log_printf( "\r\n");
}

static wiced_result_t ota2_save_app_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)player->dct_app, DCT_APP_SECTION, 0, sizeof(ota2_dct_t) );
}

static wiced_result_t ota2_save_network_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)player->dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
}

static wiced_result_t ota2_save_wifi_dct(ota2_data_t* player)
{
    return wiced_dct_write( (void*)player->dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
}

wiced_result_t ota2_save_config(ota2_data_t* player)
{
    wiced_result_t result;
    /* save all the configuration info */
    result = ota2_save_app_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_printf( "ota2_save_config() ota2_save_app_dct() failed:%d\r\n", result);
        return result;
    }
    app_dct_dirty = 0;

    result = ota2_save_network_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_printf( "ota2_save_config() ota2_save_network_dct() failed:%d\r\n", result);
        return result;
    }
    result = ota2_save_wifi_dct(player);
    if (result != WICED_SUCCESS)
    {
        wiced_log_printf( "ota2_save_config() ota2_save_wifi_dct() failed:%d\r\n", result);
        return result;
    }

    return result;
}

static wiced_result_t ota2_test_config_load_dct_wifi(ota2_data_t* player)
{
    wiced_result_t result;
    result = wiced_dct_read_lock((void**)&player->dct_wifi, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Can't get WIFi configuration!\r\n"));
    }
    return result;
}

static wiced_result_t ota2_test_config_load_dct_network(ota2_data_t* player)
{
    wiced_result_t result;
    result = wiced_dct_read_lock( (void**)&player->dct_network, WICED_TRUE, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Can't get Network configuration!\r\n"));
    }
    return result;
}

wiced_result_t ota2_test_config_init(ota2_data_t* player)
{
    wiced_result_t result;

    /* wifi */
    result = ota2_test_config_load_dct_wifi(player);

    /* network */
    result |= ota2_test_config_load_dct_network(player);

    /* App */
    result |= wiced_dct_read_lock( (void**)&player->dct_app, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( ota2_dct_t ) );

    return result;
}

static wiced_result_t ota2_test_config_unload_dct_wifi(ota2_data_t* player)
{
    wiced_result_t result = WICED_SUCCESS;

    if ((player != NULL) && (player->dct_wifi != NULL))
    {
        result = wiced_dct_read_unlock(player->dct_wifi, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            player->dct_wifi = NULL;
        }
        else
        {
            WPRINT_APP_INFO(("Can't Free/Release WiFi Configuration !\r\n"));
        }
    }
    return result;
}

static wiced_result_t ota2_test_config_unload_dct_network(ota2_data_t* player)
{
    wiced_result_t result = WICED_SUCCESS;

    if ((player != NULL) && (player->dct_network != NULL))
    {
        result = wiced_dct_read_unlock(player->dct_network, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            player->dct_network = NULL;
        }
        else
        {
            WPRINT_APP_INFO(("Can't Free/Release Network Configuration !\r\n"));
        }
    }
    return result;
}

wiced_result_t ota2_test_config_deinit(ota2_data_t* player)
{
    wiced_result_t result;

    result = wiced_dct_read_unlock(player->dct_app, WICED_TRUE);

    result |= ota2_test_config_unload_dct_network(player);
    result |= ota2_test_config_unload_dct_wifi(player);


    return result;
}

wiced_result_t ota2_test_config_reload_dct_wifi(ota2_data_t* player)
{
    wiced_result_t result;
    result =  ota2_test_config_unload_dct_wifi(player);
    result |=  ota2_test_config_load_dct_wifi(player);
    return result;
}

wiced_result_t ota2_test_config_reload_dct_network(ota2_data_t* player)
{
    wiced_result_t result;
    result =  ota2_test_config_unload_dct_network(player);
    result |=  ota2_test_config_load_dct_network(player);
    return result;
}

