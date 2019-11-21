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

/** @file Wifi Mesh test configuration routines.
 *
 */

#include <ctype.h>
#include "wiced.h"
#include "wiced_log.h"
#include "wifi_mesh_config.h"
#include "command_console.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define DIRTY_CHECK(flags,bit)                      ((flags & bit) != 0 ? '*' : ' ')


/******************************************************
 *                    Constants
 *
 ******************************************************/

#define APP_DCT_MAC_ADDRESSES_DIRTY      (1 << 0)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CONFIG_CMD_NONE         = 0,
    CONFIG_CMD_HELP,

    CONFIG_CMD_MAC_ADDR,

    CONFIG_CMD_SAVE,

    CONFIG_CMD_MAX,
} CONFIG_CMDS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    char *name;
    uint32_t value;
} lookup_table_entry_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* dirty flags for determining what to save */
static uint32_t wifi_mesh_dct_dirty;

static lookup_table_entry_t config_command_lookup[] = {
    { "help",               CONFIG_CMD_HELP                 },
    { "?",                  CONFIG_CMD_HELP                 },

    { "mac",                CONFIG_CMD_MAC_ADDR           },

    { "save",               CONFIG_CMD_SAVE                 },

    { "", CONFIG_CMD_NONE },
};


/******************************************************
 *               Function Definitions
 ******************************************************/

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

static wiced_result_t wifi_mesh_save_config(wifi_mesh_dct_t* mesh_app_dct)
{
    return wiced_dct_write((void*)mesh_app_dct, DCT_APP_SECTION, 0, sizeof(wifi_mesh_dct_t));
}

static void wifi_mesh_config_command_help(void)
{
    wiced_log_printf("Config commands:\r\n");
    wiced_log_printf("    config                                    : output current config\r\n");
    wiced_log_printf("    config <?|help>                           : show this list\r\n");

    wiced_log_printf("    config mac <0-%d> xx:xx:xx:xx:xx:xx  : change mesh mac addr entry\r\n", (WIFI_MESH_MAC_ADDRESSES - 1));

    wiced_log_printf("    config save                               : save data to flash NOTE: Changes not \r\n");
    wiced_log_printf("                                              :   automatically saved to flash!\r\n");
}

void wifi_mesh_config_set(wifi_mesh_dct_t* mesh_app_dct, int argc, char *argv[])
{
    int i;
    CONFIG_CMDS_T cmd;

    if (argc < 2)
    {
        wifi_mesh_print_config(mesh_app_dct);
        return;
    }

    cmd = CONFIG_CMD_NONE;
    for (i = 0; i < (sizeof(config_command_lookup) / sizeof(config_command_lookup[0])); ++i)
    {
        if (strcasecmp(config_command_lookup[i].name, argv[1]) == 0)
        {
            cmd = config_command_lookup[i].value;
            break;
        }
    }

    /*
     * Make sure the notices about setting changes get displayed.
     */

    switch(cmd)
    {
        case CONFIG_CMD_HELP:
            wifi_mesh_config_command_help();
            break;

        case CONFIG_CMD_MAC_ADDR:
            if (argc == 4)
            {
                wiced_mac_t new_mac;
                int idx = -1;

                idx = atoi(argv[2]);
                if (idx > WIFI_MESH_MAC_ADDRESSES)
                {
                    wifi_mesh_config_command_help();
                    break;
                }

                memcpy(&new_mac, &mesh_app_dct->mac_address[idx], sizeof(mesh_app_dct->mac_address[idx]));
                if ((parse_octets(argv[3], &new_mac, 6) != WICED_SUCCESS) &&
                    (parse_octets(argv[3], &new_mac, 1) != WICED_SUCCESS))
                {
                    wiced_log_printf("mac index: %d mac:", idx);
                    print_mac_address( (wiced_mac_t*) &mesh_app_dct->mac_address[idx] );
                    wiced_log_printf(" %c\r\n", DIRTY_CHECK(wifi_mesh_dct_dirty, APP_DCT_MAC_ADDRESSES_DIRTY));
                    return;
                }
                else
                {
                    wiced_log_printf("mac index: %d changed from: ", idx);
                    print_mac_address( (wiced_mac_t*) &mesh_app_dct->mac_address[idx] );
                    wiced_log_printf(" to: ");
                    print_mac_address( (wiced_mac_t*) &new_mac );
                    memcpy( &mesh_app_dct->mac_address[idx], &new_mac, sizeof(mesh_app_dct->mac_address[idx]));
                    wifi_mesh_dct_dirty |= APP_DCT_MAC_ADDRESSES_DIRTY;
                    wiced_log_printf(" %c\r\n", DIRTY_CHECK(wifi_mesh_dct_dirty, APP_DCT_MAC_ADDRESSES_DIRTY));
                }
            }
            else if (argc == 3)
            {
                int idx;
                idx = atoi(argv[2]);
                if (idx > WIFI_MESH_MAC_ADDRESSES)
                {
                    wifi_mesh_config_command_help();
                    break;
                }
                wiced_log_printf("mac index: %d mac:", idx);
                print_mac_address( (wiced_mac_t*) &mesh_app_dct->mac_address[idx] );
                wiced_log_printf(" %c\r\n", DIRTY_CHECK(wifi_mesh_dct_dirty, APP_DCT_MAC_ADDRESSES_DIRTY));
            }
            else
            {
                wifi_mesh_print_config(mesh_app_dct);            }
            break;

        case CONFIG_CMD_SAVE:
            if (wifi_mesh_dct_dirty != 0)
            {
                wiced_log_msg(WLF_DEF, WICED_LOG_NOTICE, "Saving Wifi Mesh App DCT:\r\n");

                wifi_mesh_save_config(mesh_app_dct);
                wifi_mesh_dct_dirty = 0;
                wifi_mesh_print_config(mesh_app_dct);
            }
            break;

        default:
            wiced_log_msg(WLF_DEF, WICED_LOG_ERR, "Unrecognized config command: %s\r\n", (argv[1][0] != 0) ? argv[1] : "");
            wifi_mesh_config_command_help();
            break;
    }
}

void wifi_mesh_print_config(wifi_mesh_dct_t* mesh_app_dct)
{
    int i;
    wiced_log_printf("  Wifi Mesh app DCT: (* = dirty)\r\n");

    wiced_log_printf("       MAC list:\r\n");
    for (i=0; i < WIFI_MESH_MAC_ADDRESSES; i++ )
    {
        wiced_log_printf("   %d: %02x:%02x:%02x:%02x:%02x:%02x  %c\r\n", i,
                mesh_app_dct->mac_address[i].octet[0], mesh_app_dct->mac_address[i].octet[1], mesh_app_dct->mac_address[i].octet[2],
                mesh_app_dct->mac_address[i].octet[3], mesh_app_dct->mac_address[i].octet[4], mesh_app_dct->mac_address[i].octet[5],
                DIRTY_CHECK(wifi_mesh_dct_dirty, APP_DCT_MAC_ADDRESSES_DIRTY) );
    }
}


wiced_result_t wifi_mesh_config_init(wifi_mesh_dct_t** mesh_app_dct)
{
    wiced_result_t result;

    /* Get mesh configuration */
    result = wiced_dct_read_lock((void **)mesh_app_dct, WICED_TRUE, DCT_APP_SECTION, 0, sizeof(wifi_mesh_dct_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_printf("Can't get mesh mac configuration!\r\n");
    }

    return result;
}


wiced_result_t wifi_mesh_config_deinit(wifi_mesh_dct_t** mesh_app_dct)
{
    wiced_result_t result = WICED_SUCCESS;

    if (mesh_app_dct != NULL)
    {
        result = wiced_dct_read_unlock(*mesh_app_dct, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            *mesh_app_dct = NULL;
        }
        else
        {
            wiced_log_printf("Can't free/release mesh configuration data !\r\n");
        }
    }
    return result;
}
