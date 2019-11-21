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

/** @file Apollo configuration routines.
 *
 */

#include <ctype.h>
#include "wiced.h"
#include "wiced_log.h"
#include "platform_audio.h"
#include "command_console.h"

#include "audio_render.h"

#include "apollocore.h"
#include "apollo_config.h"
#include "apollo_streamer.h"

#if defined(OTA2_SUPPORT)
#include "apollo_context.h"
#include "apollo_ota2_support.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define PRINT_MAC(mac)                              mac.octet[0],mac.octet[1],mac.octet[2],mac.octet[3],mac.octet[4],mac.octet[5]

#define TABLE_ENTRY_GET_TEXT(table, value)          apollo_lookup_table_get_text(table, sizeof(table)/sizeof(lookup_table_entry_t), value)
#define TABLE_ENTRY_GET_VALUE(table, str, value)    apollo_lookup_table_get_value(table, sizeof(table)/sizeof(lookup_table_entry_t), str, value)

#define DIRTY_CHECK(flags,bit)                      ((flags & bit) != 0ULL ? '*' : ' ')

#define GET_ON_OFF_VALUE(idx, var)                                                                          \
    if (argv[idx][0] == '1' || (argv[idx][0] == 'o' && argv[idx][1] == 'n'))                                \
    {                                                                                                       \
        var = 1;                                                                                            \
    }                                                                                                       \
    else if (argv[idx][0] == '0' || (argv[idx][0] == 'o' && argv[idx][1] == 'f' && argv[idx][1] == 'f'))    \
    {                                                                                                       \
        var = 0;                                                                                            \
    }                                                                                                       \
    else                                                                                                    \
    {                                                                                                       \
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bad value for on/off setting: %s\n", argv[idx]);           \
        break;                                                                                              \
    }


/******************************************************
 *                    Constants
 *
 ******************************************************/

#define NSECS_PER_MSEC                      (1000000)
#define APOLLO_THRESHOLD_MS_MIN             (1)
#define APOLLO_THRESHOLD_MS_MAX             (APOLLO_THRESHOLD_NS_MAX/NSECS_PER_MSEC)

#define APP_DCT_SPEAKER_NAME_DIRTY          (1ULL << 0)
#define APP_DCT_SPEAKER_CHANNEL_DIRTY       (1ULL << 1)
#define APP_DCT_AUTO_START_DIRTY            (1ULL << 2)
#define APP_DCT_BUFF_MS_DIRTY               (1ULL << 3)
#define APP_DCT_THRESH_NS_DIRTY             (1ULL << 4)
#define APP_DCT_CLOCK_ENABLE_DIRTY          (1ULL << 5)
#define APP_DCT_VOLUME_DIRTY                (1ULL << 6)
#define APP_DCT_PAYLOAD_SIZE_DIRTY          (1ULL << 7)
#define APP_DCT_BURST_LENGTH_DIRTY          (1ULL << 8)
#define APP_DCT_SHUFFLE_LENGTH_DIRTY        (1ULL << 9)
#define APP_DCT_SOURCE_TYPE_DIRTY           (1ULL << 10)
#define APP_DCT_AUDIO_DEVICE_RX_DIRTY       (1ULL << 11)
#define APP_DCT_AUDIO_DEVICE_TX_DIRTY       (1ULL << 12)
#define APP_DCT_APOLLO_ROLE_DIRTY           (1ULL << 13)
#define APP_DCT_CLIENTADDR_DIRTY            (1ULL << 14)
#define APP_DCT_RTP_PORT_DIRTY              (1ULL << 15)
#define APP_DCT_LOG_LEVEL_DIRTY             (1ULL << 16)
#define APP_DCT_IS_CONFIGURED_DIRTY         (1ULL << 17)
#define APP_DCT_PLL_TUNING_ENABLE_DIRTY     (1ULL << 18)
#define APP_DCT_INPUT_SAMPLE_RATE_DIRTY     (1ULL << 19)
#define APP_DCT_PLL_PPM_MAX_DIRTY           (1ULL << 20)
#define APP_DCT_PLL_PPM_MIN_DIRTY           (1ULL << 21)

#define APP_DCT_RMC_SSID_DIRTY              (1ULL << 22)
#define APP_DCT_RMC_SECURITY_KEY_DIRTY      (1ULL << 23)
#define APP_DCT_RMC_BSS_TYPE_DIRTY          (1ULL << 24)
#define APP_DCT_RMC_CHANNEL_DIRTY           (1ULL << 25)
#define APP_DCT_RMC_SECURITY_TYPE_DIRTY     (1ULL << 26)
#define APP_DCT_RMC_EXTERNAL_AP_DIRTY       (1ULL << 27)

#if defined(OTA2_SUPPORT)
#define APP_DCT_OTA2_STOP_PLAYBACK_DIRTY    (1ULL << 28)
#define APP_DCT_OTA2_REBOOT_DIRTY           (1ULL << 29)
#define APP_DCT_OTA2_URI_DIRTY              (1ULL << 30)
#endif

#define APP_DCT_SEND_DUPS_DIRTY             (1ULL << 31)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CONFIG_CMD_NONE         = 0,
    CONFIG_CMD_HELP,
    CONFIG_CMD_AUTO_START,
    CONFIG_CMD_BUFFERING_MS,
    CONFIG_CMD_CLOCK_ENABLE,
    CONFIG_CMD_SPEAKER_CHANNEL,
    CONFIG_CMD_SPEAKER_NAME,
    CONFIG_CMD_THRESHOLD_MS,
    CONFIG_CMD_THRESHOLD_NS,
    CONFIG_CMD_VOLUME,
    CONFIG_CMD_PAYLOAD_SIZE,
    CONFIG_CMD_BURST_LENGTH,
    CONFIG_CMD_SHUFFLE_LENGTH,
    CONFIG_CMD_SOURCE_TYPE,
    CONFIG_CMD_AUDIO_DEVICE_RX,
    CONFIG_CMD_AUDIO_DEVICE_TX,
    CONFIG_CMD_APOLLO_ROLE,
    CONFIG_CMD_CLIENTADDR,
    CONFIG_CMD_RTP_PORT,
    CONFIG_CMD_LOG_LEVEL,
    CONFIG_CMD_IS_CONFIGURED,
    CONFIG_CMD_PLL_TUNING_ENABLE,
    CONFIG_CMD_INPUT_SAMPLE_RATE,
    CONFIG_CMD_PLL_PPM_MAX,
    CONFIG_CMD_PLL_PPM_MIN,
    CONFIG_CMD_RMC_SSID,
    CONFIG_CMD_RMC_PASSPHRASE,
    CONFIG_CMD_RMC_BSS_TYPE,
    CONFIG_CMD_RMC_CHANNEL,
    CONFIG_CMD_RMC_SECURITY,
    CONFIG_CMD_RMC_EXTERNAL_AP,
#if defined(OTA2_SUPPORT)
    CONFIG_CMD_OTA2_STOP_PLAYBACK_TO_UPDATE,
    CONFIG_CMD_OTA2_REBOOT_AFTER_DOWNLOAD,
    CONFIG_CMD_OTA2_URI,
#endif
    CONFIG_CMD_SEND_DUPS,
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

static wiced_result_t apollo_save_app_dct(apollo_dct_collection_t* dct_tables);
static void apollo_print_app_info(apollo_dct_collection_t* dct_tables);
static void apollo_print_network_info(apollo_dct_collection_t* dct_tables);
static void apollo_print_wifi_info(apollo_dct_collection_t* dct_tables);
static wiced_bool_t apollo_encode_speaker_channel(int argc, char *argv[], APOLLO_CHANNEL_MAP_T* channel);
static wiced_bool_t apollo_get_channel_band(int channel, int* band);
static wiced_result_t apollo_save_wifi_dct(apollo_dct_collection_t* dct_tables);

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
static void apollo_print_bt_info(apollo_dct_collection_t* dct_tables);
static wiced_result_t apollo_save_bt_dct(apollo_dct_collection_t* dct_tables);
#endif


/******************************************************
 *               Variables Definitions
 ******************************************************/

/* dirty flags for determining what to save */
static uint64_t app_dct_dirty;

static lookup_table_entry_t config_command_lookup[] = {
    { "help",               CONFIG_CMD_HELP                 },
    { "?",                  CONFIG_CMD_HELP                 },
    { "auto_start",         CONFIG_CMD_AUTO_START           },
    { "auto",               CONFIG_CMD_AUTO_START           },

    { "apollo_role",        CONFIG_CMD_APOLLO_ROLE          },
    { "role",               CONFIG_CMD_APOLLO_ROLE          },

    { "buffering_ms",       CONFIG_CMD_BUFFERING_MS         },
    { "buff_ms",            CONFIG_CMD_BUFFERING_MS         },

    { "clock",              CONFIG_CMD_CLOCK_ENABLE         },
    { "pll_tuning",         CONFIG_CMD_PLL_TUNING_ENABLE    },
    { "pll",                CONFIG_CMD_PLL_TUNING_ENABLE    },
    { "pll_ppm_max",        CONFIG_CMD_PLL_PPM_MAX          },
    { "ppm_max",            CONFIG_CMD_PLL_PPM_MAX          },
    { "pll_ppm_min",        CONFIG_CMD_PLL_PPM_MIN          },
    { "ppm_min",            CONFIG_CMD_PLL_PPM_MIN          },

    { "rmc_ssid",           CONFIG_CMD_RMC_SSID             },
    { "rmc_bss",            CONFIG_CMD_RMC_BSS_TYPE         },
    { "rmc_pass",           CONFIG_CMD_RMC_PASSPHRASE       },
    { "rmc_chan",           CONFIG_CMD_RMC_CHANNEL          },
    { "rmc_sec",            CONFIG_CMD_RMC_SECURITY         },
    { "rmc_ext_ap",         CONFIG_CMD_RMC_EXTERNAL_AP      },

    { "ssid",               CONFIG_CMD_RMC_SSID             },
    { "bss",                CONFIG_CMD_RMC_BSS_TYPE         },
    { "pass",               CONFIG_CMD_RMC_PASSPHRASE       },
    { "chan",               CONFIG_CMD_RMC_CHANNEL          },
    { "sec",                CONFIG_CMD_RMC_SECURITY         },
    { "ext_ap",             CONFIG_CMD_RMC_EXTERNAL_AP      },

    { "speaker_name",       CONFIG_CMD_SPEAKER_NAME         },
    { "speaker_channel",    CONFIG_CMD_SPEAKER_CHANNEL      },
    { "speaker_chan",       CONFIG_CMD_SPEAKER_CHANNEL      },

    { "spkr_name",          CONFIG_CMD_SPEAKER_NAME         },
    { "spkr_chan",          CONFIG_CMD_SPEAKER_CHANNEL      },

    { "threshold_ms",       CONFIG_CMD_THRESHOLD_MS         },
    { "thresh_ms",          CONFIG_CMD_THRESHOLD_MS         },

    { "threshold_ns",       CONFIG_CMD_THRESHOLD_NS         },
    { "thresh_ns",          CONFIG_CMD_THRESHOLD_NS         },

    { "volume",             CONFIG_CMD_VOLUME               },
    { "vol",                CONFIG_CMD_VOLUME               },

    { "payload_size",       CONFIG_CMD_PAYLOAD_SIZE         },

    { "burst_length",       CONFIG_CMD_BURST_LENGTH         },
    { "bl",                 CONFIG_CMD_BURST_LENGTH         },
    { "shuffle_length",     CONFIG_CMD_SHUFFLE_LENGTH       },
    { "sl",                 CONFIG_CMD_SHUFFLE_LENGTH       },

    { "source_type",        CONFIG_CMD_SOURCE_TYPE          },
    { "src_t",              CONFIG_CMD_SOURCE_TYPE          },
    { "audio_device_rx",    CONFIG_CMD_AUDIO_DEVICE_RX      },
    { "ad_rx",              CONFIG_CMD_AUDIO_DEVICE_RX      },
    { "audio_device_tx",    CONFIG_CMD_AUDIO_DEVICE_TX      },
    { "ad_tx",              CONFIG_CMD_AUDIO_DEVICE_TX      },
    { "input_sample_rate",  CONFIG_CMD_INPUT_SAMPLE_RATE    },
    { "sr_rx",              CONFIG_CMD_INPUT_SAMPLE_RATE    },

    { "clientaddr",         CONFIG_CMD_CLIENTADDR           },
    { "addr",               CONFIG_CMD_CLIENTADDR           },

    { "rtp_port",           CONFIG_CMD_RTP_PORT             },
    { "port",               CONFIG_CMD_RTP_PORT             },

    { "log_level",          CONFIG_CMD_LOG_LEVEL            },
    { "log",                CONFIG_CMD_LOG_LEVEL            },
    { "is_configured",      CONFIG_CMD_IS_CONFIGURED        },
    { "is_conf",            CONFIG_CMD_IS_CONFIGURED        },

    { "slc_send_dups",      CONFIG_CMD_SEND_DUPS            },
    { "dups",               CONFIG_CMD_SEND_DUPS            },

#if defined(OTA2_SUPPORT)
    { "ota2_stop_to_update", CONFIG_CMD_OTA2_STOP_PLAYBACK_TO_UPDATE },
    { "ota2_reboot",         CONFIG_CMD_OTA2_REBOOT_AFTER_DOWNLOAD   },
    { "ota2_uri",            CONFIG_CMD_OTA2_URI                     },
#endif
    { "save",               CONFIG_CMD_SAVE                 },

    { "", CONFIG_CMD_NONE },
};

static lookup_table_entry_t speaker_channel_name[] =
{
    { "NONE",   CHANNEL_MAP_NONE    },  /* None or undefined    */
    { "FL",     CHANNEL_MAP_FL      },  /* Front Left           */
    { "FR",     CHANNEL_MAP_FR      },  /* Front Right          */
    { "FC",     CHANNEL_MAP_FC      },  /* Front Center         */
    { "LFE1",   CHANNEL_MAP_LFE1    },  /* LFE-1                */
    { "BL",     CHANNEL_MAP_BL      },  /* Back Left            */
    { "BR",     CHANNEL_MAP_BR      },  /* Back Right           */
    { "FLC",    CHANNEL_MAP_FLC     },  /* Front Left Center    */
    { "FRC",    CHANNEL_MAP_FRC     },  /* Front Right Center   */
    { "BC",     CHANNEL_MAP_BC      },  /* Back Center          */
    { "LFE2",   CHANNEL_MAP_LFE2    },  /* LFE-2                */
    { "SIL",    CHANNEL_MAP_SIL     },  /* Side Left            */
    { "SIR",    CHANNEL_MAP_SIR     },  /* Side Right           */
    { "TPFL",   CHANNEL_MAP_TPFL    },  /* Top Front Left       */
    { "TPFR",   CHANNEL_MAP_TPFR    },  /* Top Front Right      */
    { "TPFC",   CHANNEL_MAP_TPFC    },  /* Top Front Center     */
    { "TPC",    CHANNEL_MAP_TPC     },  /* Top Center           */
    { "TPBL",   CHANNEL_MAP_TPBL    },  /* Top Back Left        */
    { "TPBR",   CHANNEL_MAP_TPBR    },  /* Top Back Right       */
    { "TPSIL",  CHANNEL_MAP_TPSIL   },  /* Top Side Left        */
    { "TPSIR",  CHANNEL_MAP_TPSIR   },  /* Top Side Right       */
    { "TPBC",   CHANNEL_MAP_TPBC    },  /* Top Back Center      */
    { "BTFC",   CHANNEL_MAP_BTFC    },  /* Bottom Front Center  */
    { "BTFL",   CHANNEL_MAP_BTFL    },  /* Bottom Front Left    */
    { "BTFR",   CHANNEL_MAP_BTFR    },  /* Bottom Front Right   */
    { "TPLS",   CHANNEL_MAP_TPLS    },  /* Top Left Surround    */
    { "TPRS",   CHANNEL_MAP_TPRS    },  /* Top Right Surround   */
    { "LS",     CHANNEL_MAP_LS      },  /* Middle Left Surround */
    { "RS",     CHANNEL_MAP_RS      },  /* Middle Right Surround*/
    { "BLC",    CHANNEL_MAP_BLC     },  /* Back Left Center     */
    { "BRC",    CHANNEL_MAP_BRC     },  /* Back Right Center    */
    { "VIRT",   CHANNEL_VIRTUAL     },  /* Virtual channel FLAG */
};

static lookup_table_entry_t apollo_security_name_table[] =
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

static lookup_table_entry_t apollo_source_type_table[] =
{
    { "capture",    APOLLO_AUDIO_SOURCE_CAPTURE      },
    { "bt",         APOLLO_AUDIO_SOURCE_BT           },
    { "ext",        APOLLO_AUDIO_SOURCE_EXTERNAL     },
};

static lookup_table_entry_t apollo_role_type_table[] =
{
    { "source",     APOLLO_ROLE_SOURCE      },
    { "sink",       APOLLO_ROLE_SINK        },
};

static lookup_table_entry_t apollo_bss_type_table[] =
{
    { "infra",      WICED_BSS_TYPE_INFRASTRUCTURE   }, /**< Denotes infrastructure network                  */
    { "adhoc",      WICED_BSS_TYPE_ADHOC            }, /**< Denotes an 802.11 ad-hoc IBSS network           */
    { "any",        WICED_BSS_TYPE_ANY              }, /**< Denotes either infrastructure or ad-hoc network */
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t apollo_save_app_dct(apollo_dct_collection_t* dct_tables)
{
    return wiced_dct_write((void*)dct_tables->dct_app, DCT_APP_SECTION, 0, sizeof(apollo_dct_t));
}

static wiced_result_t apollo_save_wifi_dct(apollo_dct_collection_t* dct_tables)
{
    return wiced_dct_write((void*)dct_tables->dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
}

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
static wiced_result_t apollo_save_bt_dct(apollo_dct_collection_t* dct_tables)
{
    return wiced_dct_write((void*)dct_tables->dct_bt, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t));
}
#endif

#if defined(OTA2_SUPPORT)
static wiced_result_t apollo_save_network_dct(apollo_dct_collection_t* dct_tables)
{
    return wiced_dct_write((void*)dct_tables->dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t));
}
#endif  /* OTA2 */

static char* apollo_lookup_table_get_text(lookup_table_entry_t *table, int table_entries, uint32_t value)
{
    int table_index;

    for (table_index = 0; table_index < table_entries; table_index++)
    {
        if (table[table_index].value == value)
        {
            return table[table_index].name;
        }
    }

    return "";
}

static wiced_result_t apollo_lookup_table_get_value(lookup_table_entry_t *table, int table_entries, char* str, uint32_t *value)
{
    wiced_result_t result = WICED_ERROR;
    int table_index;

    for (table_index = 0; table_index < table_entries; table_index++)
    {
        if (strcasecmp(table[table_index].name, str) == 0)
        {
            *value = table[table_index].value;
            result = WICED_SUCCESS;
            break;
        }
    }

    return result;
}

/* Prints null-terminated speaker channel type in the provided buffer */
static void apollo_channel_name_get_text(uint32_t channel, char *buff, int buff_len)
{
    int table_index, outdex;

    if ((buff == NULL) || (buff_len < 3))
    {
        return;
    }

    buff[0] = 0;
    for (table_index = 0; table_index < sizeof(speaker_channel_name)/sizeof(lookup_table_entry_t); table_index++)
    {
        if (speaker_channel_name[table_index].value & channel)
        {
            outdex = strlen(buff);
            if (strlen(speaker_channel_name[table_index].name) < (buff_len - (outdex + 1)))
            {
                strlcat(buff, speaker_channel_name[table_index].name, buff_len);
                strlcat(buff, " ", buff_len);
            }
            else
            {
                return;
            }
        }
    }
    if ((strlen(buff) == 0) && (buff_len > strlen("Invalid")))
    {
        strlcpy(buff, "Invalid", buff_len);
    }
}

static wiced_bool_t apollo_encode_speaker_channel(int argc, char *argv[], APOLLO_CHANNEL_MAP_T* channel)
{
    APOLLO_CHANNEL_MAP_T new_speaker_channel_map;
    int table_index, arg_index;

    if ((argc < 2) || (argv == NULL) || (channel == NULL))
    {
        return WICED_FALSE;
    }

    /* go through all argv's after 2 (config speaker_channel FL FR FC) and OR in the bit(s) */
    new_speaker_channel_map = 0;
    for (arg_index = 2; arg_index < argc; arg_index++)
    {
        for (table_index = 0; table_index < sizeof(speaker_channel_name)/sizeof(lookup_table_entry_t); table_index++)
        {
            if (strcasecmp(speaker_channel_name[table_index].name, argv[arg_index]) == 0)
            {
                new_speaker_channel_map |= speaker_channel_name[table_index].value;
            }
        }
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "\r\n   chan: 0x%08x\r\n", new_speaker_channel_map);
    }
    *channel = new_speaker_channel_map;

    return WICED_TRUE;
}

static wiced_bool_t apollo_get_channel_band(int channel, int* band)
{
    if (band != NULL)
    {
        *band = (channel < 34) ? WICED_802_11_BAND_2_4GHZ : WICED_802_11_BAND_5GHZ;
        return WICED_TRUE;
    }

    return WICED_FALSE;
}


static wiced_security_t apollo_security_type_parse(char* security_str)
{
    wiced_security_t security_type;

    if (TABLE_ENTRY_GET_VALUE(apollo_security_name_table, security_str, (uint32_t*)&security_type) != WICED_SUCCESS)
    {
        security_type = WICED_SECURITY_UNKNOWN;
    }

    return security_type;
}
static char* apollo_security_type_get_name(wiced_security_t type)
{
    return TABLE_ENTRY_GET_TEXT(apollo_security_name_table, type);
}

static wiced_result_t apollo_source_type_get(char* source_type_str, uint32_t* source_type)
{
    return TABLE_ENTRY_GET_VALUE(apollo_source_type_table, source_type_str, source_type);
}

char* apollo_source_type_get_text(int source_type)
{
    return TABLE_ENTRY_GET_TEXT(apollo_source_type_table, source_type);
}

static wiced_result_t apollo_role_type_get(char* apollo_role_str, uint32_t* apollo_role)
{
    return TABLE_ENTRY_GET_VALUE(apollo_role_type_table, apollo_role_str, apollo_role);
}

static char* apollo_role_type_get_text(uint32_t apollo_role)
{
    return TABLE_ENTRY_GET_TEXT(apollo_role_type_table, apollo_role);
}

static wiced_result_t apollo_bss_type_get(char* apollo_bss_type_str, wiced_bss_type_t *bss_type)
{
    uint32_t bss_type_32 = WICED_BSS_TYPE_UNKNOWN;
    if (TABLE_ENTRY_GET_VALUE(apollo_bss_type_table, apollo_bss_type_str, &bss_type_32) == WICED_SUCCESS)
    {
        *bss_type = (wiced_bss_type_t)bss_type_32;
        return WICED_SUCCESS;
    }
    return WICED_BADARG;
}

static char *apollo_bss_type_get_text(wiced_bss_type_t bss_type)
{
    return TABLE_ENTRY_GET_TEXT(apollo_bss_type_table, bss_type);

}

static platform_audio_sample_rates_t apollo_sample_rate_type_get(int sample_rate)
{
    platform_audio_sample_rates_t sr = (platform_audio_sample_rates_t)0;

    switch(sample_rate)
    {
        case 8000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_8KHZ;
            break;
        case 12000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_12KHZ;
            break;
        case 16000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_16KHZ;
            break;
        case 24000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_24KHZ;
            break;
        case 32000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_32KHZ;
            break;
        case 48000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_48KHZ;
            break;
        case 64000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_64KHZ;
            break;
        case 96000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_96KHZ;
            break;
        case 128000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_128KHZ;
            break;
        case 192000:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_192KHZ;
            break;

        case 11025:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_11_025KHZ;
            break;
        case 22050:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_22_05KHZ;
            break;
        case 44100:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ;
            break;
        case 88200:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_88_2KHZ;
            break;
        case 176400:
            sr = PLATFORM_AUDIO_SAMPLE_RATE_176_4KHZ;
            break;

        default:
            break;
    }

    return sr;
}


static void apollo_config_command_help(void)
{
    wiced_log_printf("Config commands:\r\n");
    wiced_log_printf("    config                              : output current config\r\n");
    wiced_log_printf("    config <?|help>                     : show this list\r\n");
    wiced_log_printf("    config auto_start <0|off|1|on>      : 0 = auto start off, 1 = auto start on\r\n");
    wiced_log_printf("           auto       <0|off|1|on>\r\n");
    wiced_log_printf("    config buffering_ms <xxx>           : xxx = milliseconds\r\n");
    wiced_log_printf("           buff_ms <xxx>                :       (range:%d <= xx <= %d)\r\n",
                                                                APOLLO_BUFFERING_MS_MIN, APOLLO_BUFFERING_MS_MAX);
    wiced_log_printf("    config clock <0|disable|1|enable>   : 0 = disable AS clock, 1 = enable\r\n");
    wiced_log_printf("    config pll   <0|disable|1|enable>   : 0 = disable audio PLL tuning, 1 = enable\r\n");
    wiced_log_printf("    config pll_tuning <0|disable|1|enable>\r\n");
    wiced_log_printf("    config pll_ppm_max <xxx>            : xxx = parts-per-million (PPM)\r\n");
    wiced_log_printf("           ppm_max <xxx>                        (range: %d <= xxx <= %d)\r\n",
                                                                APOLLO_PLL_TUNING_PPM_MAX_LOW, APOLLO_PLL_TUNING_PPM_MAX_HIGH);
    wiced_log_printf("    config pll_ppm_min <xxx>            : xxx = parts-per-million (PPM)\r\n");
    wiced_log_printf("           ppm_min <xxx>                        (range: %d <= xxx <= %d)\r\n",
                                                                APOLLO_PLL_TUNING_PPM_MIN_LOW, APOLLO_PLL_TUNING_PPM_MIN_HIGH);

    wiced_log_printf("    config speaker_name <name>          : speaker name (max %d characters)\r\n", APOLLO_SPEAKER_NAME_LENGTH);
    wiced_log_printf("           spkr_name <name>\r\n");
    wiced_log_printf("    config speaker_channel <ch> [ch]... : channel mix - all will be OR'ed together\r\n");
    wiced_log_printf("           spkr_chan <ch> [ch]...       :    FL,FR,FC,LFE1,BL,BR,FLC,FRC,BC,LFE2,\r\n");
    wiced_log_printf("                                        :    SIL,SIR,TPFL,TPFR,TPFC,TPC,TPBL,TPBR,\r\n");
    wiced_log_printf("                                        :    TPSIL,TPSIR,TPBC,BTFC,BTFL,BTFR\r\n");

    wiced_log_printf("    config threshold_ms <xx>            : xx = milliseconds\r\n");
    wiced_log_printf("           thresh_ms <xx>               :      (range:%d <= xx <= %d)\r\n",
                                                                APOLLO_THRESHOLD_MS_MIN, APOLLO_THRESHOLD_MS_MAX);
    wiced_log_printf("    config threshold_ns <xx>            : xx = nanoseconds\r\n");
    wiced_log_printf("           thresh_ns <xx>               :      (range:%d <= xx <= %d)\r\n",
                                                                APOLLO_THRESHOLD_NS_MIN, APOLLO_THRESHOLD_NS_MAX);

    wiced_log_printf("    config volume <xx>                  : xx = volume level\r\n");
    wiced_log_printf("           vol <xx>                     :      (range:%d <= xx <= %d)\r\n", APOLLO_VOLUME_MIN, APOLLO_VOLUME_MAX);

    wiced_log_printf("    config payload_size <size_in_bytes> : from %d to %d bytes\r\n", RTP_PACKET_MIN_DATA, RTP_PACKET_MAX_DATA);
    wiced_log_printf("    config burst_length <count> | auto  : from %d to %d\r\n", 0, RTP_AUDIO_BL_MAX_LENGTH);
    wiced_log_printf("    config bl           <packet_count>\r\n" );
    wiced_log_printf("    config shuffle_length <count>       : from %d to %d\r\n", 0, RTP_AUDIO_BL_MAX_LENGTH);
    wiced_log_printf("    config sl           <packet_count>\r\n" );

    wiced_log_printf("    config source_type  <string>        : \"bt\" = BT A2DP audio, \"capture\" = using local ADC, \"ext\" = using external source\r\n");
    wiced_log_printf("    config src_t        <string> \r\n");

    wiced_log_printf("    config audio_device_rx <device_X>   : enter X as in WICED_AUDIO_X, X starts at 0 for all WICED platforms\r\n");
    wiced_log_printf("    config ad_rx           <device_X>\r\n");
    wiced_log_printf("    config audio_device_tx <device_X>   : enter X as in WICED_AUDIO_X, X starts at 0 for all WICED platforms\r\n");
    wiced_log_printf("    config ad_tx           <device_X>\r\n");
    wiced_log_printf("    config input_sample_rate <in_Hz>    : enter input sample rate in Hertz for the RX/capture audio device\r\n");
    wiced_log_printf("    config sr_rx             <in_Hz>\r\n");

    wiced_log_printf("    config apollo_role <source | sink>  : Configure as a source or a sink\r\n");
    wiced_log_printf("    config role <source | sink>\r\n");

    wiced_log_printf("    config clientaddr <IP address>      : Client IP address for sender to use\r\n");
    wiced_log_printf("    config addr <IP address>\r\n");

    wiced_log_printf("    config rmc_ssid <SSID_name>         : RMC Network Name\r\n");
    wiced_log_printf("    config rmc_pass <Passphrase>        : RMC Network Passphrase\r\n");
    wiced_log_printf("    config rmc_bss <bss_type>           : RMC Network BSS Type  <infra|adhoc|any>\r\n");
    wiced_log_printf("    config rmc_chan <Channel>           : RMC Network Channel\r\n");
    wiced_log_printf("    config rmc_sec  <security_type>     : RMC Security Type <open|wep|shared|wpa_aes|wpa_tkip|wpa_mix|wpa2_aes|\r\n");
    wiced_log_printf("                                        :                    wpa2_tkip|wpa2_mix|wpa2_aesent|ibss|wps_open|wps_aes>\r\n");
    wiced_log_printf("    config rmc_ext_ap <yes|no|1|0>      : RMC Network Use External AP\r\n");
    wiced_log_printf("    config ssid <SSID_name>             : RMC Network Name\r\n");
    wiced_log_printf("    config pass <Passphrase>            : RMC Network Passphrase\r\n");
    wiced_log_printf("    config bss <bss_type>               : RMC Network BSS Type  <infra|adhoc|any>\r\n");
    wiced_log_printf("    config chan <Channel>               : RMC Network Channel\r\n");
    wiced_log_printf("    config sec  <security_type>         : RMC Security Type <open|wep|shared|wpa_aes|wpa_tkip|wpa_mix|wpa2_aes|\r\n");
    wiced_log_printf("                                        :                    wpa2_tkip|wpa2_mix|wpa2_aesent|ibss|wps_open|wps_aes>\r\n");
    wiced_log_printf("    config ext_ap <yes|no|1|0>          : RMC Network Use External AP\r\n");

    wiced_log_printf("    config rtp_port <port number>       : RTP port number\r\n");
    wiced_log_printf("    config port <port number>\r\n");

    wiced_log_printf("    config log_level <level>            : Set the default application logging level\r\n");
    wiced_log_printf("    config log <level>\r\n");

    wiced_log_printf("    config slc_send_dups <0|off|1|on>   : Send duplicate packets when SLC is enabled\r\n");
    wiced_log_printf("    config dups <0|off|1|on>\r\n");

    wiced_log_printf("    config is_configured <0|no|1|yes>   : Set to 0 to force BT GATT configuration\r\n");
    wiced_log_printf("    config is_conf       <0|no|1|yes>\r\n");
#if defined(OTA2_SUPPORT)
    wiced_log_printf("    config ota2_stop_to_update <0|no|1|yes>   : Stop playback to start OTA2 download\r\n");
    wiced_log_printf("    config ota2_reboot   <0|no|1|yes>         : Reboot after successful OTA2 download\r\n");
    wiced_log_printf("    config ota2_uri <new_URI>                 : Location of OTA2 update file\r\n");
#endif
    wiced_log_printf("    config save                         : save data to flash NOTE: Changes not \r\n");
    wiced_log_printf("                                        :   automatically saved to flash!\r\n");
}

void apollo_set_config(apollo_dct_collection_t* dct_tables, int argc, char *argv[])
{
    int i, auto_start, ms, clock_enable, pll_enable;
    uint32_t nsec;
    int volume;
    int port;
    int level;
    int is_configured;
    int ppm_max_min;
    int arg;
    APOLLO_CHANNEL_MAP_T new_speaker_channel;
    WICED_LOG_LEVEL_T log_save;
    CONFIG_CMDS_T cmd;

    if (argc < 2)
    {
        apollo_config_print_info(dct_tables);
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

    log_save = wiced_log_get_facility_level(WLF_AUDIO);
    if (log_save < WICED_LOG_NOTICE)
    {
        wiced_log_set_facility_level(WLF_AUDIO, WICED_LOG_NOTICE);
    }

    switch(cmd)
    {
        case CONFIG_CMD_HELP:
            apollo_config_command_help();
            break;

        case CONFIG_CMD_AUTO_START:
            auto_start = dct_tables->dct_app->auto_start;
            if (argc > 2)
            {
                if (strcasecmp(argv[2], "off") == 0)
                {
                    auto_start = 0;
                }
                else if (strcasecmp(argv[2], "on") == 0)
                {
                    auto_start = 1;
                }
                else
                {
                    auto_start = atoi(argv[2]);
                }
                auto_start = (auto_start == 0) ? 0 : 1;
                if (dct_tables->dct_app->auto_start != auto_start)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "auto start: %d -> %d\r\n", dct_tables->dct_app->auto_start, auto_start);
                    dct_tables->dct_app->auto_start = auto_start;
                    app_dct_dirty |= APP_DCT_AUTO_START_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Auto start: %d\r\n", dct_tables->dct_app->auto_start);
            break;

        case CONFIG_CMD_BUFFERING_MS:
            if (argc > 2)
            {
                ms = atoi(argv[2]);
                if ((ms < APOLLO_BUFFERING_MS_MIN) || (ms > APOLLO_BUFFERING_MS_MAX))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range min:%d  max:%d\r\n", APOLLO_BUFFERING_MS_MIN, APOLLO_BUFFERING_MS_MAX);
                    break;
                }
                ms = MAX(ms, APOLLO_BUFFERING_MS_MIN);
                ms = MIN(ms, APOLLO_BUFFERING_MS_MAX);
                if (dct_tables->dct_app->buffering_ms != ms)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "buffering: %d -> %d (ms)\r\n", dct_tables->dct_app->buffering_ms, ms);
                    dct_tables->dct_app->buffering_ms = ms;
                    app_dct_dirty |= APP_DCT_BUFF_MS_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Buffering: %d ms\r\n", dct_tables->dct_app->buffering_ms);
            break;

        case CONFIG_CMD_CLOCK_ENABLE:
            clock_enable = dct_tables->dct_app->clock_enable;
            if (argc > 2)
            {
                if (strcasecmp(argv[2], "disable") == 0)
                {
                    clock_enable = 0;
                }
                else if (strcasecmp(argv[2], "enable") == 0)
                {
                    clock_enable = 1;
                }
                else
                {
                    clock_enable = atoi(argv[2]);
                }
                clock_enable = (clock_enable == 0) ? 0 : 1;
                if (dct_tables->dct_app->clock_enable != clock_enable)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "clock enable: %d -> %d\r\n", dct_tables->dct_app->clock_enable, clock_enable);
                    dct_tables->dct_app->clock_enable = clock_enable;
                    app_dct_dirty |= APP_DCT_CLOCK_ENABLE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Clock enable: %d\r\n", dct_tables->dct_app->clock_enable);
            break;

        case CONFIG_CMD_PLL_TUNING_ENABLE:
            pll_enable = dct_tables->dct_app->pll_tuning_enable;
            if (argc > 2)
            {
                if (strcasecmp(argv[2], "disable") == 0)
                {
                    pll_enable = 0;
                }
                else if (strcasecmp(argv[2], "enable") == 0)
                {
                    pll_enable = 1;
                }
                else
                {
                    pll_enable = atoi(argv[2]);
                }
                pll_enable = (pll_enable == 0) ? 0 : 1;
                if (dct_tables->dct_app->pll_tuning_enable != pll_enable)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "audio PLL tuning enable: %d -> %d\r\n", dct_tables->dct_app->pll_tuning_enable, pll_enable);
                    dct_tables->dct_app->pll_tuning_enable = pll_enable;
                    app_dct_dirty |= APP_DCT_PLL_TUNING_ENABLE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio PLL tuning enable: %d\r\n", dct_tables->dct_app->pll_tuning_enable);
            break;

        case CONFIG_CMD_PLL_PPM_MAX:
            if (argc > 2)
            {
                ppm_max_min = atoi(argv[2]);
                /* validity check */
                if (ppm_max_min < APOLLO_PLL_TUNING_PPM_MAX_LOW || ppm_max_min > APOLLO_PLL_TUNING_PPM_MAX_HIGH)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range lowest: %d  highest: %d\r\n", APOLLO_PLL_TUNING_PPM_MAX_LOW, APOLLO_PLL_TUNING_PPM_MAX_HIGH);
                    break;
                }
                if (dct_tables->dct_app->pll_tuning_ppm_max != ppm_max_min)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "CONFIG_CMD_PLL_PPM_MAX: %d -> %d\r\n", dct_tables->dct_app->pll_tuning_ppm_max, ppm_max_min);
                    dct_tables->dct_app->pll_tuning_ppm_max = ppm_max_min;
                    app_dct_dirty |= APP_DCT_PLL_PPM_MAX_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "PLL PPM max: %d\r\n", dct_tables->dct_app->pll_tuning_ppm_max);
            break;

        case CONFIG_CMD_PLL_PPM_MIN:
            if (argc > 2)
            {
                ppm_max_min = atoi(argv[2]);
                /* validity check */
                if (ppm_max_min < APOLLO_PLL_TUNING_PPM_MIN_LOW || ppm_max_min > APOLLO_PLL_TUNING_PPM_MIN_HIGH)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range lowest: %d  highest: %d\r\n",
                                  APOLLO_PLL_TUNING_PPM_MIN_LOW, APOLLO_PLL_TUNING_PPM_MIN_HIGH);
                    break;
                }
                if (dct_tables->dct_app->pll_tuning_ppm_min != ppm_max_min)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "CONFIG_CMD_PLL_PPM_MIN: %d -> %d\r\n",
                                  dct_tables->dct_app->pll_tuning_ppm_min, ppm_max_min);
                    dct_tables->dct_app->pll_tuning_ppm_min = ppm_max_min;
                    app_dct_dirty |= APP_DCT_PLL_PPM_MIN_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "PLL PPM MIN: %d\r\n", dct_tables->dct_app->pll_tuning_ppm_min);
            break;

        case CONFIG_CMD_RMC_SSID:
        {
            if (argc > 2)
            {
                int  ssid_len = strlen(argv[2]);
                if (ssid_len > sizeof(dct_tables->dct_app->rmc_info.ssid_name))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: too long (max: %d)\r\n", sizeof(dct_tables->dct_app->rmc_info.ssid_name));
                    break;
                }
                if ((ssid_len != dct_tables->dct_app->rmc_info.ssid_length) ||
                    strncmp((char*)dct_tables->dct_app->rmc_info.ssid_name, argv[2], dct_tables->dct_app->rmc_info.ssid_length)  != 0)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "ssid: %.*s -> %s\r\n", dct_tables->dct_app->rmc_info.ssid_length, dct_tables->dct_app->rmc_info.ssid_name, argv[2]);
                    memset(dct_tables->dct_app->rmc_info.ssid_name, 0, sizeof(dct_tables->dct_app->rmc_info.ssid_name));
                    memcpy(dct_tables->dct_app->rmc_info.ssid_name, argv[2], ssid_len);
                    dct_tables->dct_app->rmc_info.ssid_length = ssid_len;
                    app_dct_dirty |= APP_DCT_RMC_SSID_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RMC Network name: (%d) %.*s\r\n", dct_tables->dct_app->rmc_info.ssid_length,
                          dct_tables->dct_app->rmc_info.ssid_length, dct_tables->dct_app->rmc_info.ssid_name);
            break;
        }
        case CONFIG_CMD_RMC_PASSPHRASE:
        {
            if (argc > 2)
            {
                int pass_len = strlen(argv[2]);
                if (pass_len > sizeof(dct_tables->dct_app->rmc_info.security_key))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: too long (max %d)\r\n", sizeof(dct_tables->dct_app->rmc_info.security_key));
                    break;
                }
                if ((pass_len != dct_tables->dct_app->rmc_info.security_key_length) ||
                     (strncmp(dct_tables->dct_app->rmc_info.security_key, argv[2], dct_tables->dct_app->rmc_info.security_key_length) != 0) )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "passphrase: %.*s -> %s \r\n",
                                  dct_tables->dct_app->rmc_info.security_key_length, dct_tables->dct_app->rmc_info.security_key, argv[2]);
                    memset(dct_tables->dct_app->rmc_info.security_key, 0, sizeof(dct_tables->dct_app->rmc_info.security_key));
                    memcpy(dct_tables->dct_app->rmc_info.security_key, argv[2], pass_len);
                    dct_tables->dct_app->rmc_info.security_key_length = pass_len;
                    app_dct_dirty |= APP_DCT_RMC_SECURITY_KEY_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RMC Network passphrase: %.*s \r\n",
                          dct_tables->dct_app->rmc_info.security_key_length, dct_tables->dct_app->rmc_info.security_key);
            break;
        }

        case CONFIG_CMD_RMC_BSS_TYPE:
        {
            wiced_result_t result = WICED_ERROR;
            wiced_bss_type_t bss_type;
            if (argc > 2)
            {
                result = apollo_bss_type_get(argv[2], &bss_type);
                if (result == WICED_SUCCESS && (bss_type != dct_tables->dct_app->rmc_info.bss_type))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "RMC BSS type: %d -> %d\r\n", dct_tables->dct_app->rmc_info.bss_type, bss_type);
                    dct_tables->dct_app->rmc_info.bss_type = bss_type;
                    app_dct_dirty |= APP_DCT_RMC_BSS_TYPE_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RMC BSS type: %s\r\n", apollo_bss_type_get_text(dct_tables->dct_app->rmc_info.bss_type));
            break;
        }

        case CONFIG_CMD_RMC_CHANNEL:
        {
            int new_channel, new_band;
            if (argc > 2)
            {
                new_channel = atoi(argv[2]);
                if (apollo_get_channel_band(new_channel, &new_band) != WICED_TRUE)
                {
                    /* TODO: get country code for output */
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: %d not a valid channel.\r\n", new_channel);
                    break;
                }
                if ((dct_tables->dct_app->rmc_info.channel != new_channel) ||
                    (dct_tables->dct_app->rmc_info.band != new_band))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "channel: %d -> %d\r\n  Band: %d -> %d \r\n",
                            dct_tables->dct_app->rmc_info.channel, new_channel,
                            dct_tables->dct_app->rmc_info.band, new_band);
                    dct_tables->dct_app->rmc_info.channel = new_channel;
                    dct_tables->dct_app->rmc_info.band = new_band;
                    app_dct_dirty |= APP_DCT_RMC_CHANNEL_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RMC Network channel:%d  band:%s\r\n", dct_tables->dct_app->rmc_info.channel,
                          (dct_tables->dct_app->rmc_info.band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz");
            break;
        }
        case CONFIG_CMD_RMC_SECURITY:
            if (argc > 2)
            {
                wiced_security_t new_sec_type;

                new_sec_type = apollo_security_type_parse(argv[2]);
                if (new_sec_type == WICED_SECURITY_UNKNOWN)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: unknown security type: %s\r\n", (argv[2][0] != 0) ? argv[2] : "");
                    break;
                }
                if (dct_tables->dct_app->rmc_info.security != new_sec_type)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "network security %s -> %s\r\n",
                            apollo_security_type_get_name(dct_tables->dct_app->rmc_info.security),
                            apollo_security_type_get_name(new_sec_type));
                    dct_tables->dct_app->rmc_info.security = new_sec_type;
                    app_dct_dirty |= APP_DCT_RMC_SECURITY_TYPE_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RMC Network security type: %s\r\n", apollo_security_type_get_name(dct_tables->dct_app->rmc_info.security));
            break;

        case CONFIG_CMD_RMC_EXTERNAL_AP:
        {
            int use_external_ap;
            use_external_ap = dct_tables->dct_app->rmc_info.use_external_ap;
            if (argc > 2)
            {
                if ((strcasecmp(argv[2], "no") == 0) || (tolower((int)argv[2][0]) == 'n'))
                {
                    use_external_ap = 0;
                }
                else if ((strcasecmp(argv[2], "yes") == 0) || (tolower((int)argv[2][0]) == 'y'))
                {
                    use_external_ap = 1;
                }
                else
                {
                    use_external_ap = atoi(argv[2]);
                }
                use_external_ap = (use_external_ap == 0) ? 0 : 1;
                if (dct_tables->dct_app->rmc_info.use_external_ap != use_external_ap)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "External AP: %d -> %d\r\n", dct_tables->dct_app->rmc_info.use_external_ap, use_external_ap);
                    dct_tables->dct_app->rmc_info.use_external_ap = use_external_ap;
                    app_dct_dirty |= APP_DCT_RMC_EXTERNAL_AP_DIRTY;
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "WARNING: If RMC connection is up, you must reboot after changing RMC configuration.\r\n");
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "External AP: %d\r\n", dct_tables->dct_app->rmc_info.use_external_ap);
        }
        break;

        case CONFIG_CMD_SPEAKER_NAME:
        {
            char name[APOLLO_SPEAKER_NAME_LENGTH + 1];
            if (argc > 2)
            {
                if (strlen(argv[2]) >= APOLLO_SPEAKER_NAME_LENGTH)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: too long (max %d)\r\n", APOLLO_SPEAKER_NAME_LENGTH);
                    break;
                }
                memset(name, 0, sizeof(name));
                memcpy(name, dct_tables->dct_app->speaker_name, APOLLO_SPEAKER_NAME_LENGTH);
                if (strcmp(name, argv[2]) != 0)
                {
                    int speaker_name_len = sizeof(dct_tables->dct_app->speaker_name);

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Speaker name: %s -> %s\r\n", name, argv[2]);
                    memset(dct_tables->dct_app->speaker_name, 0, speaker_name_len);
                    strlcpy(dct_tables->dct_app->speaker_name, argv[2], speaker_name_len);
                    app_dct_dirty |= APP_DCT_SPEAKER_NAME_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Speaker name: %s\r\n", dct_tables->dct_app->speaker_name);
            break;
        }

        case CONFIG_CMD_SPEAKER_CHANNEL:
        {
            char buff[10], buff2[10];
            if (argc > 2)
            {
                /* convert channel string to the bit fields */
                if (apollo_encode_speaker_channel(argc, argv, &new_speaker_channel) == WICED_TRUE)
                {
                    apollo_channel_name_get_text(dct_tables->dct_app->speaker_channel, buff, sizeof(buff));
                    apollo_channel_name_get_text(new_speaker_channel, buff2, sizeof(buff2));
                    if (dct_tables->dct_app->speaker_channel != new_speaker_channel)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Speaker channel: (0x%08x) %s -> (0x%08x) %s\r\n",
                                      dct_tables->dct_app->speaker_channel, buff, new_speaker_channel, buff2);
                        dct_tables->dct_app->speaker_channel = new_speaker_channel;
                        app_dct_dirty |= APP_DCT_SPEAKER_CHANNEL_DIRTY;
                    }
                }
            }
            apollo_channel_name_get_text(dct_tables->dct_app->speaker_channel, buff, sizeof(buff));
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Speaker channel: (0x%08x) %s\r\n", dct_tables->dct_app->speaker_channel, buff);
            break;
        }

        case CONFIG_CMD_THRESHOLD_MS:
            if (argc > 2)
            {
                ms = strtoul(argv[2], NULL, 0);
                /* validity check */
                if ((ms < APOLLO_THRESHOLD_MS_MIN) || (ms > APOLLO_THRESHOLD_MS_MAX))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range min: %d  max: %d\r\n", APOLLO_THRESHOLD_MS_MIN, APOLLO_THRESHOLD_MS_MAX);
                    break;
                }
                ms = MAX(ms, APOLLO_THRESHOLD_MS_MIN);
                ms = MIN(ms, APOLLO_THRESHOLD_MS_MAX);
                if ((dct_tables->dct_app->threshold_ns / NSECS_PER_MSEC) != ms)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "CONFIG_CMD_THRESHOLD_MS: %lu -> %lu\r\n", dct_tables->dct_app->threshold_ns / NSECS_PER_MSEC, ms);
                    dct_tables->dct_app->threshold_ns = (ms * NSECS_PER_MSEC);
                    app_dct_dirty |= APP_DCT_THRESH_NS_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Threshold: %lu nsec\r\n", dct_tables->dct_app->threshold_ns);
            break;

        case CONFIG_CMD_THRESHOLD_NS:
            if (argc > 2)
            {
                nsec = strtoul(argv[2], NULL, 0);
                /* validity check */
                if ((nsec < (uint32_t)APOLLO_THRESHOLD_NS_MIN) || (nsec > (uint32_t)APOLLO_THRESHOLD_NS_MAX))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range min: %d  max: %d\r\n", APOLLO_THRESHOLD_NS_MIN, APOLLO_THRESHOLD_NS_MAX);
                    break;
                }
                nsec = MAX(nsec, APOLLO_THRESHOLD_NS_MIN);
                nsec = MIN(nsec, APOLLO_THRESHOLD_NS_MAX);
                if (dct_tables->dct_app->threshold_ns != nsec)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "CONFIG_CMD_THRESHOLD_NS: %lu -> %lu\r\n", dct_tables->dct_app->threshold_ns, nsec);
                    dct_tables->dct_app->threshold_ns = nsec;
                    app_dct_dirty |= APP_DCT_THRESH_NS_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Threshold: %lu nsec\r\n", dct_tables->dct_app->threshold_ns);
            break;

        case CONFIG_CMD_VOLUME:
            if (argc > 2)
            {
                volume = atoi(argv[2]);
                /* validity check */
                if (volume < APOLLO_VOLUME_MIN || volume > APOLLO_VOLUME_MAX)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: out of range min: %d  max: %d\r\n", APOLLO_VOLUME_MIN, APOLLO_VOLUME_MAX);
                    break;
                }
                if (dct_tables->dct_app->volume != volume)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "CONFIG_CMD_VOLUME: %d -> %d\r\n", dct_tables->dct_app->volume, volume);
                    dct_tables->dct_app->volume = volume;
                    app_dct_dirty |= APP_DCT_VOLUME_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Volume: %d\r\n", dct_tables->dct_app->volume);
            break;

        case CONFIG_CMD_PAYLOAD_SIZE:
        {
            int payload_size = dct_tables->dct_app->payload_size;
            if (argc > 2)
            {
                payload_size = atoi(argv[2]);
                if ((payload_size >= RTP_PACKET_MIN_DATA) && (payload_size <= RTP_PACKET_MAX_DATA) && (dct_tables->dct_app->payload_size != payload_size))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Payload size: %d -> %d\r\n", dct_tables->dct_app->payload_size, payload_size);
                    dct_tables->dct_app->payload_size = payload_size;
                    app_dct_dirty |= APP_DCT_PAYLOAD_SIZE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Payload size: %d\r\n", dct_tables->dct_app->payload_size);
        }
            break;

        case CONFIG_CMD_BURST_LENGTH:
        {
            int burst_length = dct_tables->dct_app->burst_length;
            if (argc > 2)
            {
                if (!strncasecmp(argv[2], "auto", 4))
                {
                    burst_length = APOLLO_STREAMER_BURST_AUTO_SLC;
                }
                else
                {
                    burst_length = atoi(argv[2]);
                    if (burst_length < 0 || burst_length > RTP_AUDIO_BL_MAX_LENGTH)
                    {
                        burst_length = dct_tables->dct_app->burst_length;
                    }
                }
                if (dct_tables->dct_app->burst_length != burst_length)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Burst length: %d -> %d\r\n", dct_tables->dct_app->burst_length, burst_length);
                    dct_tables->dct_app->burst_length = burst_length;
                    app_dct_dirty |= APP_DCT_BURST_LENGTH_DIRTY;
                }
            }
            if (dct_tables->dct_app->burst_length != APOLLO_STREAMER_BURST_AUTO_SLC)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Burst length: %d\r\n", dct_tables->dct_app->burst_length);
            }
            else
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Burst length: auto\r\n");
            }
            break;
        }

        case CONFIG_CMD_SHUFFLE_LENGTH:
        {
            int shuffle_length = dct_tables->dct_app->shuffle_length;
            if (argc > 2)
            {
                shuffle_length = atoi(argv[2]);
                if ((shuffle_length >= 0) && (shuffle_length <= RTP_AUDIO_BL_MAX_LENGTH) && (dct_tables->dct_app->shuffle_length != shuffle_length))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Shuffle length: %d -> %d\r\n", dct_tables->dct_app->shuffle_length, shuffle_length);
                    dct_tables->dct_app->shuffle_length = shuffle_length;
                    app_dct_dirty |= APP_DCT_SHUFFLE_LENGTH_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Shuffle length: %d\r\n", dct_tables->dct_app->shuffle_length);
            break;
        }

        case CONFIG_CMD_SOURCE_TYPE:
        {
            wiced_result_t result = WICED_ERROR;
            uint32_t source_type = dct_tables->dct_app->source_type;
            if (argc > 2)
            {
                result = apollo_source_type_get(argv[2], &source_type);
                if ((result == WICED_SUCCESS) && (source_type != dct_tables->dct_app->source_type))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Audio source type: %d -> %d\r\n", dct_tables->dct_app->source_type, source_type);
                    dct_tables->dct_app->source_type = source_type;
                    app_dct_dirty |= APP_DCT_SOURCE_TYPE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio source type: %s\r\n", apollo_source_type_get_text(dct_tables->dct_app->source_type));
            break;
        }

        case CONFIG_CMD_AUDIO_DEVICE_RX:
        {
            platform_audio_device_id_t device = dct_tables->dct_app->audio_device_rx;
            if (argc > 2)
            {
                int device_index = atoi(argv[2]);
                if ((device_index >= 0) && (device_index < PLATFORM_AUDIO_NUM_INPUTS))
                {
                    device = platform_audio_input_devices[device_index].device_id;
                    if (device != dct_tables->dct_app->audio_device_rx)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio device RX ID: 0x%x -> 0x%x\r\n", dct_tables->dct_app->audio_device_rx,
                                      device, platform_audio_input_devices[device_index].device_name);
                        dct_tables->dct_app->audio_device_rx = device;
                        app_dct_dirty |= APP_DCT_AUDIO_DEVICE_RX_DIRTY;
                    }
                }
            }
            platform_audio_print_device_list(dct_tables->dct_app->audio_device_rx, (app_dct_dirty & APP_DCT_AUDIO_DEVICE_RX_DIRTY),
                                             AUDIO_DEVICE_ID_NONE, 0, 1);
            break;
        }

        case CONFIG_CMD_AUDIO_DEVICE_TX:
        {
            platform_audio_device_id_t device = dct_tables->dct_app->audio_device_tx;
            if (argc > 2)
            {
                int device_index = atoi(argv[2]);
                if ((device_index >= 0) && (device_index < PLATFORM_AUDIO_NUM_OUTPUTS))
                {
                    device = platform_audio_output_devices[device_index].device_id;
                    if (device != dct_tables->dct_app->audio_device_tx)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Audio device TX ID: 0x%x -> 0x%x %s\r\n", dct_tables->dct_app->audio_device_tx,
                                      device, platform_audio_output_devices[device_index].device_name);
                        dct_tables->dct_app->audio_device_tx = device;
                        app_dct_dirty |= APP_DCT_AUDIO_DEVICE_TX_DIRTY;
                    }
                }
            }
            platform_audio_print_device_list(AUDIO_DEVICE_ID_NONE, WICED_FALSE,
                                             dct_tables->dct_app->audio_device_tx, (app_dct_dirty & APP_DCT_AUDIO_DEVICE_RX_DIRTY),
                                             1);
            break;
        }

        case CONFIG_CMD_INPUT_SAMPLE_RATE:
        {
            platform_audio_sample_rates_t sample_rate = dct_tables->dct_app->input_sample_rate;
            if (argc > 2)
            {
                sample_rate = apollo_sample_rate_type_get(atoi(argv[2]));
                if ( (sample_rate > 0) && (sample_rate != dct_tables->dct_app->input_sample_rate) )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Input sample rate: %s KHz-> %s KHz\r\n",
                                  platform_audio_device_get_sample_rates_string(dct_tables->dct_app->input_sample_rate),
                                  platform_audio_device_get_sample_rates_string(sample_rate));
                    dct_tables->dct_app->input_sample_rate = sample_rate;
                    app_dct_dirty |= APP_DCT_INPUT_SAMPLE_RATE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Input sample rate: %s KHz\r\n", platform_audio_device_get_sample_rates_string(dct_tables->dct_app->input_sample_rate));
            break;
        }

        case CONFIG_CMD_APOLLO_ROLE:
        {
            wiced_result_t result = WICED_ERROR;
            uint32_t apollo_role = dct_tables->dct_app->apollo_role;
            if (argc > 2)
            {
                result = apollo_role_type_get(argv[2], &apollo_role);
                if (result == WICED_SUCCESS && (apollo_role != dct_tables->dct_app->apollo_role))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Apollo role: %d -> %d\r\n", dct_tables->dct_app->apollo_role, apollo_role);
                    dct_tables->dct_app->apollo_role = apollo_role;
                    app_dct_dirty |= APP_DCT_APOLLO_ROLE_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Apollo role type: %s\r\n", apollo_role_type_get_text(dct_tables->dct_app->apollo_role));
            break;
        }

        case CONFIG_CMD_CLIENTADDR:
        {
            wiced_ip_address_t address;

            if (argc > 2)
            {
                if (str_to_ip(argv[2], &address) == 0 && dct_tables->dct_app->clientaddr.ip.v4 != address.ip.v4)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Apollo client IP address: %d.%d.%d.%d -> %d.%d.%d.%d\r\n",
                            (dct_tables->dct_app->clientaddr.ip.v4 >> 24) & 0xFF, (dct_tables->dct_app->clientaddr.ip.v4 >> 16) & 0xFF,
                            (dct_tables->dct_app->clientaddr.ip.v4 >> 8) & 0xFF, dct_tables->dct_app->clientaddr.ip.v4 & 0xFF,
                            (address.ip.v4 > 24) & 0xFF, (address.ip.v4 > 16) & 0xFF, (address.ip.v4 > 8) & 0xFF, address.ip.v4 & 0xFF);
                    dct_tables->dct_app->clientaddr.ip.v4 = address.ip.v4;
                    app_dct_dirty |= APP_DCT_CLIENTADDR_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Apollo client IP address: %d.%d.%d.%d\r\n",
                          (dct_tables->dct_app->clientaddr.ip.v4 >> 24) & 0xFF, (dct_tables->dct_app->clientaddr.ip.v4 >> 16) & 0xFF,
                          (dct_tables->dct_app->clientaddr.ip.v4 >> 8) & 0xFF, dct_tables->dct_app->clientaddr.ip.v4 & 0xFF);
            break;
        }

        case CONFIG_CMD_RTP_PORT:
            if (argc > 2)
            {
                port = atoi(argv[2]);
                /* validity check */
                if (port < 0)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: port %d invalid\r\n", port);
                    break;
                }
                if (dct_tables->dct_app->rtp_port != port)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "rtp_port: %d -> %d\r\n", dct_tables->dct_app->rtp_port, port);
                    dct_tables->dct_app->rtp_port = port;
                    app_dct_dirty |= APP_DCT_RTP_PORT_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "RTP port: %d\r\n", dct_tables->dct_app->rtp_port);
            break;

        case CONFIG_CMD_LOG_LEVEL:
            if (argc > 2)
            {
                level = atoi(argv[2]);
                /* validity check */
                if (level < 0)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: log level %d invalid\r\n", level);
                    break;
                }
                if (dct_tables->dct_app->log_level != level)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "log_level: %d -> %d\r\n", dct_tables->dct_app->log_level, level);
                    dct_tables->dct_app->log_level = level;
                    app_dct_dirty |= APP_DCT_LOG_LEVEL_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Log level: %d\r\n", dct_tables->dct_app->log_level);
            break;

        case CONFIG_CMD_IS_CONFIGURED:
            is_configured = dct_tables->dct_app->is_configured;
            if (argc > 2)
            {
                if ((strcasecmp(argv[2], "no") == 0)  || (tolower((int)argv[2][0]) == 'n'))
                {
                    is_configured = 0;
                }
                else if ((strcasecmp(argv[2], "yes") == 0) || (tolower((int)argv[2][0]) == 'y'))
                {
                    is_configured = 1;
                }
                else
                {
                    is_configured = atoi(argv[2]);
                }
                is_configured = (is_configured == 0) ? 0 : 1;
                if (dct_tables->dct_app->is_configured != is_configured)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "is configured: %d -> %d\r\n", dct_tables->dct_app->is_configured, is_configured);
                    dct_tables->dct_app->is_configured = is_configured;
                    app_dct_dirty |= APP_DCT_IS_CONFIGURED_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Is configured: %d\r\n", dct_tables->dct_app->is_configured);
            break;

        case CONFIG_CMD_SEND_DUPS:
            if (argc > 2)
            {
                GET_ON_OFF_VALUE(2, arg);
                if (dct_tables->dct_app->slc_send_duplicates != arg)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "slc_send_dups: %d -> %d\n", dct_tables->dct_app->slc_send_duplicates, arg);
                    dct_tables->dct_app->slc_send_duplicates = arg;
                    app_dct_dirty |= APP_DCT_SEND_DUPS_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "slc_send_duplicates: %d\n", dct_tables->dct_app->slc_send_duplicates);
            break;

#if defined(OTA2_SUPPORT)
        case CONFIG_CMD_OTA2_STOP_PLAYBACK_TO_UPDATE:
        {
            int ota2_stop_playback_to_update;
            ota2_stop_playback_to_update = dct_tables->dct_app->ota2_stop_playback_to_update;
            if (argc > 2)
            {
                if ((strcasecmp(argv[2], "no") == 0) || (tolower((int)argv[2][0]) == 'n'))
                {
                    ota2_stop_playback_to_update = 0;
                }
                else if ((strcasecmp(argv[2], "yes") == 0)  || (tolower((int)argv[2][0]) == 'y'))
                {
                    ota2_stop_playback_to_update = 1;
                }
                else
                {
                    ota2_stop_playback_to_update = atoi(argv[2]);
                }
                ota2_stop_playback_to_update = (ota2_stop_playback_to_update == 0) ? 0 : 1;
                if (dct_tables->dct_app->ota2_stop_playback_to_update != ota2_stop_playback_to_update)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "OTA2 Stop playback to update: %d -> %d\r\n", dct_tables->dct_app->ota2_stop_playback_to_update, ota2_stop_playback_to_update);
                    dct_tables->dct_app->ota2_stop_playback_to_update = ota2_stop_playback_to_update;
                    app_dct_dirty |= APP_DCT_OTA2_STOP_PLAYBACK_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "OTA2 Stop playback to update: %d\r\n", dct_tables->dct_app->ota2_stop_playback_to_update);
        }
        break;

        case CONFIG_CMD_OTA2_REBOOT_AFTER_DOWNLOAD:
        {
            int ota2_reboot_after_download;
            ota2_reboot_after_download = dct_tables->dct_app->ota2_reboot_after_download;
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
                if (dct_tables->dct_app->ota2_reboot_after_download != ota2_reboot_after_download)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "OTA2 Reboot after download: %d -> %d\r\n", dct_tables->dct_app->ota2_reboot_after_download, ota2_reboot_after_download);
                    dct_tables->dct_app->ota2_reboot_after_download = ota2_reboot_after_download;
                    app_dct_dirty |= APP_DCT_OTA2_REBOOT_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "OTA2 Reboot after download: %d\r\n", dct_tables->dct_app->ota2_reboot_after_download);
        }
        break;
        case CONFIG_CMD_OTA2_URI:
        {
            if (argc > 2)
            {
                int ota2_uri_len = strlen(argv[2]);
                if (ota2_uri_len > sizeof(dct_tables->dct_app->ota2_default_update_uri))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error: too long (max %d)\r\n", sizeof(dct_tables->dct_app->ota2_default_update_uri));
                    break;
                }
                if ((ota2_uri_len != strlen(dct_tables->dct_app->ota2_default_update_uri)) ||
                     (strncmp(dct_tables->dct_app->ota2_default_update_uri, argv[2], ota2_uri_len) != 0) )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "OTA2 URI: %s -> %s \r\n",
                                  dct_tables->dct_app->ota2_default_update_uri, argv[2]);
                    strlcpy(dct_tables->dct_app->ota2_default_update_uri, argv[2], sizeof(dct_tables->dct_app->ota2_default_update_uri));
                    app_dct_dirty |= APP_DCT_OTA2_URI_DIRTY;
                }
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "OTA2 Default Update URI: %s \r\n", dct_tables->dct_app->ota2_default_update_uri);
            break;
        }
        break;
#endif
        case CONFIG_CMD_SAVE:
            if (app_dct_dirty != 0)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Saving App DCT:\r\n");
                apollo_save_app_dct(dct_tables);
                app_dct_dirty = 0;
                apollo_print_app_info(dct_tables);
            }
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unrecognized config command: %s\r\n", (argv[1][0] != 0) ? argv[1] : "");
            apollo_config_command_help();
            break;
    }

    /*
     * Restore the original log level.
     */

    wiced_log_set_facility_level(WLF_AUDIO, log_save);
}

static void apollo_print_app_info(apollo_dct_collection_t* dct_tables)
{
    char buf[10];

    apollo_channel_name_get_text(dct_tables->dct_app->speaker_channel, buf, sizeof(buf));

    wiced_log_printf("  Apollo app DCT:\r\n");
    wiced_log_printf("     is configured: %s %c\r\n", dct_tables->dct_app->is_configured == 0 ? "no" : "yes", DIRTY_CHECK(app_dct_dirty, APP_DCT_IS_CONFIGURED_DIRTY));
    wiced_log_printf("       apollo role: %s %c\r\n", apollo_role_type_get_text(dct_tables->dct_app->apollo_role), DIRTY_CHECK(app_dct_dirty, APP_DCT_APOLLO_ROLE_DIRTY));
    wiced_log_printf("      speaker name: %s %c\r\n", dct_tables->dct_app->speaker_name, DIRTY_CHECK(app_dct_dirty, APP_DCT_SPEAKER_NAME_DIRTY));
    wiced_log_printf("           channel: (0x%08x) %s %c\r\n", dct_tables->dct_app->speaker_channel, buf, DIRTY_CHECK(app_dct_dirty, APP_DCT_SPEAKER_CHANNEL_DIRTY));
    wiced_log_printf("         buffering: %d ms %c\r\n", dct_tables->dct_app->buffering_ms, DIRTY_CHECK(app_dct_dirty, APP_DCT_BUFF_MS_DIRTY));
    wiced_log_printf("         threshold: %lu nsec %c\r\n", dct_tables->dct_app->threshold_ns, DIRTY_CHECK(app_dct_dirty, APP_DCT_THRESH_NS_DIRTY));
    wiced_log_printf("        auto_start: %s %c\r\n", dct_tables->dct_app->auto_start == 0 ? "off" : "on", DIRTY_CHECK(app_dct_dirty, APP_DCT_AUTO_START_DIRTY));
    wiced_log_printf("     slc_send_dups: %s %c\r\n", dct_tables->dct_app->slc_send_duplicates == 0 ? "off" : "on", DIRTY_CHECK(app_dct_dirty, APP_DCT_SEND_DUPS_DIRTY));
    wiced_log_printf("      clock enable: %s %c\r\n", dct_tables->dct_app->clock_enable == 0 ? "disable" : "enable", DIRTY_CHECK(app_dct_dirty, APP_DCT_CLOCK_ENABLE_DIRTY));
    wiced_log_printf(" PLL tuning enable: %s %c\r\n", dct_tables->dct_app->pll_tuning_enable == 0 ? "disable" : "enable", DIRTY_CHECK(app_dct_dirty, APP_DCT_PLL_TUNING_ENABLE_DIRTY));
    wiced_log_printf("       PLL PPM max: %d %c\r\n", dct_tables->dct_app->pll_tuning_ppm_max, DIRTY_CHECK(app_dct_dirty, APP_DCT_PLL_PPM_MAX_DIRTY));
    wiced_log_printf("       PLL PPM min: %d %c\r\n", dct_tables->dct_app->pll_tuning_ppm_min, DIRTY_CHECK(app_dct_dirty, APP_DCT_PLL_PPM_MIN_DIRTY));
    wiced_log_printf("            volume: %d %c\r\n", dct_tables->dct_app->volume, DIRTY_CHECK(app_dct_dirty, APP_DCT_VOLUME_DIRTY));
    wiced_log_printf("       source type: %s %c\r\n", apollo_source_type_get_text(dct_tables->dct_app->source_type), DIRTY_CHECK(app_dct_dirty, APP_DCT_SOURCE_TYPE_DIRTY));
    wiced_log_printf("      payload size: %d %c\r\n", dct_tables->dct_app->payload_size, DIRTY_CHECK(app_dct_dirty, APP_DCT_PAYLOAD_SIZE_DIRTY));
    if (dct_tables->dct_app->burst_length != APOLLO_STREAMER_BURST_AUTO_SLC)
    {
        wiced_log_printf("      burst length: %d %c\r\n", dct_tables->dct_app->burst_length, DIRTY_CHECK(app_dct_dirty, APP_DCT_BURST_LENGTH_DIRTY));
    }
    else
    {
        wiced_log_printf("      burst length: auto %c\r\n", DIRTY_CHECK(app_dct_dirty, APP_DCT_BURST_LENGTH_DIRTY));
    }
    wiced_log_printf("    shuffle length: %d %c\r\n", dct_tables->dct_app->shuffle_length, DIRTY_CHECK(app_dct_dirty, APP_DCT_SHUFFLE_LENGTH_DIRTY));
    platform_audio_print_device_list(dct_tables->dct_app->audio_device_rx, (app_dct_dirty & APP_DCT_AUDIO_DEVICE_RX_DIRTY),
                                     dct_tables->dct_app->audio_device_tx, (app_dct_dirty & APP_DCT_AUDIO_DEVICE_TX_DIRTY), 0);
    wiced_log_printf(" Input sample rate: %s KHz %c\r\n", platform_audio_device_get_sample_rates_string(dct_tables->dct_app->input_sample_rate),
            DIRTY_CHECK(app_dct_dirty, APP_DCT_INPUT_SAMPLE_RATE_DIRTY));
    wiced_log_printf(" client IP address: %d.%d.%d.%d %c\r\n",
            (dct_tables->dct_app->clientaddr.ip.v4 >> 24) & 0xFF, (dct_tables->dct_app->clientaddr.ip.v4 >> 16) & 0xFF,
            (dct_tables->dct_app->clientaddr.ip.v4 >> 8) & 0xFF, dct_tables->dct_app->clientaddr.ip.v4 & 0xFF, DIRTY_CHECK(app_dct_dirty, APP_DCT_CLIENTADDR_DIRTY));
    wiced_log_printf("          RTP port: %d %c\r\n", dct_tables->dct_app->rtp_port, DIRTY_CHECK(app_dct_dirty, APP_DCT_RTP_PORT_DIRTY));
    wiced_log_printf("         log level: %d (currently %d) %c\r\n", dct_tables->dct_app->log_level, wiced_log_get_facility_level(WLF_AUDIO), DIRTY_CHECK(app_dct_dirty, APP_DCT_LOG_LEVEL_DIRTY));

#if defined(OTA2_SUPPORT)
    wiced_log_printf( "  OTA2 Details:\r\n");
    wiced_log_printf( "         Major Version: %d\r\n", dct_tables->dct_app->ota2_major_version);
    wiced_log_printf( "         Minor Version: %d\r\n", dct_tables->dct_app->ota2_minor_version);
    wiced_log_printf( "        Stop to update: %s %c\r\n", ((dct_tables->dct_app->ota2_stop_playback_to_update == 0) ? "no" : "yes"),  DIRTY_CHECK(app_dct_dirty, APP_DCT_OTA2_STOP_PLAYBACK_DIRTY));
    wiced_log_printf( "   Reboot after update: %s %c\r\n", ((dct_tables->dct_app->ota2_reboot_after_download == 0) ? "no" : "yes"),  DIRTY_CHECK(app_dct_dirty, APP_DCT_OTA2_REBOOT_DIRTY));
    wiced_log_printf( "    Default Update URI: %s %c\r\n", dct_tables->dct_app->ota2_default_update_uri,  DIRTY_CHECK(app_dct_dirty, APP_DCT_OTA2_URI_DIRTY));
#endif

    wiced_log_printf("  RMC Network Details:\r\n");
    wiced_log_printf("              SSID: (%d) %.*s %c\r\n", dct_tables->dct_app->rmc_info.ssid_length,
                            dct_tables->dct_app->rmc_info.ssid_length, dct_tables->dct_app->rmc_info.ssid_name,
                            DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_SSID_DIRTY));
    wiced_log_printf("          Security:(0x%lx) %s %c \r\n", dct_tables->dct_app->rmc_info.security,
                                                              apollo_security_type_get_name(dct_tables->dct_app->rmc_info.security),
                                                              DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_SECURITY_TYPE_DIRTY));
    if (dct_tables->dct_app->rmc_info.security != WICED_SECURITY_OPEN)
    {
        wiced_log_printf("        Passphrase: (%d) %.*s %c\r\n", dct_tables->dct_app->rmc_info.security_key_length,
                              dct_tables->dct_app->rmc_info.security_key_length, dct_tables->dct_app->rmc_info.security_key,
                              DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_SECURITY_KEY_DIRTY));
    }
    else
    {
        wiced_log_printf("        Passphrase: none %c\r\n", DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_SECURITY_KEY_DIRTY));
    }
    wiced_log_printf("   use_external_ap: %s %c\r\n", dct_tables->dct_app->rmc_info.use_external_ap == 0 ? "no" : "yes", DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_EXTERNAL_AP_DIRTY));
    wiced_log_printf("          bss_type: %d %s %c\r\n", dct_tables->dct_app->rmc_info.bss_type, apollo_bss_type_get_text(dct_tables->dct_app->rmc_info.bss_type), DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_BSS_TYPE_DIRTY));
    wiced_log_printf("           Channel: %d %c\r\n", dct_tables->dct_app->rmc_info.channel, DIRTY_CHECK(app_dct_dirty, APP_DCT_RMC_CHANNEL_DIRTY));
    wiced_log_printf("              Band: %s\r\n", (dct_tables->dct_app->rmc_info.band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz");

}

static void apollo_print_network_info(apollo_dct_collection_t* dct_tables)
{
    wiced_ip_address_t ip_addr;
    int32_t rssi;

    wiced_log_printf("  Network DCT:\r\n");
    wiced_log_printf("         Interface: %s\r\n",
           (dct_tables->dct_network->interface == (wiced_interface_t)WWD_STA_INTERFACE)      ? "STA" :
           (dct_tables->dct_network->interface == (wiced_interface_t)WWD_AP_INTERFACE)       ? "AP" :
           (dct_tables->dct_network->interface == (wiced_interface_t)WWD_ETHERNET_INTERFACE) ? "Ethernet" :
           "Unknown");
    wiced_log_printf("          Hostname: %s\r\n", dct_tables->dct_network->hostname.value);
    wiced_ip_get_ipv4_address(dct_tables->dct_network->interface, &ip_addr);
    wiced_log_printf("           IP addr: %d.%d.%d.%d\r\n",
           (int)((ip_addr.ip.v4 >> 24) & 0xFF), (int)((ip_addr.ip.v4 >> 16) & 0xFF),
           (int)((ip_addr.ip.v4 >> 8) & 0xFF),  (int)(ip_addr.ip.v4 & 0xFF));
    wwd_wifi_get_rssi(&rssi);
    wiced_log_printf("              RSSI: %d dBm\r\n", rssi);
}

static void apollo_print_wifi_info(apollo_dct_collection_t* dct_tables)
{
    wiced_security_t sec;
    uint32_t channel;
    int band;
    wiced_mac_t wiced_mac;

    wwd_wifi_get_mac_address(&wiced_mac, WICED_STA_INTERFACE);

    if (dct_tables->dct_network->interface == WICED_STA_INTERFACE)
    {
        sec = dct_tables->dct_wifi->stored_ap_list[0].details.security;
        wiced_log_printf("  WiFi DCT:\r\n");

        wiced_log_printf("   WICED MAC (STA): %02x:%02x:%02x:%02x:%02x:%02x\r\n", PRINT_MAC(wiced_mac));

        wiced_log_printf("               MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", PRINT_MAC(dct_tables->dct_wifi->mac_address));
        wiced_log_printf("              SSID: %.*s\r\n", dct_tables->dct_wifi->stored_ap_list[0].details.SSID.length,
                                                         dct_tables->dct_wifi->stored_ap_list[0].details.SSID.value);
        wiced_log_printf("          Security: (0x%lx) %s \r\n", sec, apollo_security_type_get_name(sec));

        if (dct_tables->dct_wifi->stored_ap_list[0].details.security != WICED_SECURITY_OPEN)
        {
            wiced_log_printf("        Passphrase: %.*s\r\n", dct_tables->dct_wifi->stored_ap_list[0].security_key_length,
                    dct_tables->dct_wifi->stored_ap_list[0].security_key);
        }
        else
        {
            wiced_log_printf("        Passphrase: none\r\n");
        }

        channel = dct_tables->dct_wifi->stored_ap_list[0].details.channel;
        band    = dct_tables->dct_wifi->stored_ap_list[0].details.band;
        wiced_log_printf("           Channel: %d\r\n", (int)channel);
        wiced_log_printf("              Band: %s\r\n", (band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz");
    }
    else
    {
        /*
         * Nothing for AP interface yet.
         */
    }
}


#ifdef WICED_DCT_INCLUDE_BT_CONFIG
static void apollo_print_bt_info(apollo_dct_collection_t* dct_tables)
{
    wiced_log_printf("  BT DCT:\r\n");

    wiced_log_printf("    BT device name: %s\r\n", dct_tables->dct_bt->bluetooth_device_name);
    wiced_log_printf("            BT MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", dct_tables->dct_bt->bluetooth_device_address[0],
                     dct_tables->dct_bt->bluetooth_device_address[1], dct_tables->dct_bt->bluetooth_device_address[2],
                     dct_tables->dct_bt->bluetooth_device_address[3], dct_tables->dct_bt->bluetooth_device_address[4],
                     dct_tables->dct_bt->bluetooth_device_address[5]);
    wiced_log_printf("   BT device class: %02x:%02x:%02x\r\n", dct_tables->dct_bt->bluetooth_device_class[0],
                     dct_tables->dct_bt->bluetooth_device_class[1], dct_tables->dct_bt->bluetooth_device_class[2]);
}
#endif


static void apollo_print_current_info(apollo_dct_collection_t* dct_tables)
{
    uint32_t channel;
    wiced_wifi_get_channel(&channel);
    wiced_log_printf(" Current:\r\n");
    wiced_log_printf("           Channel: %lu\r\n              Band: %s\r\n",
                     channel, channel <= 13 ? "2.4GHz" : "5GHz");
}

void apollo_config_print_info(apollo_dct_collection_t* dct_tables)
{
    wiced_log_printf("\r\nConfig Info: * = dirty\r\n");
    apollo_print_app_info(dct_tables);

    apollo_print_network_info(dct_tables);
    apollo_print_wifi_info(dct_tables);
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    apollo_print_bt_info(dct_tables);
#endif

    apollo_print_current_info(dct_tables);
    wiced_log_printf("\r\n");
}

static wiced_result_t apollo_config_load_dct_wifi(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    /* Get WiFi configuration */
    result = wiced_dct_read_lock((void **)&dct_tables->dct_wifi, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS || dct_tables->dct_wifi->device_configured != WICED_TRUE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get WiFi configuration!\r\n");
    }
    return result;
}

static wiced_result_t apollo_config_load_dct_network(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    /* Get network configuration */
    result = wiced_dct_read_lock((void **)&dct_tables->dct_network, WICED_TRUE, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t));
    if (result != WICED_SUCCESS || (dct_tables->dct_network->interface != WICED_STA_INTERFACE && dct_tables->dct_network->interface != WICED_AP_INTERFACE))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get network configuration!\r\n");
    }
    return result;
}

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
static wiced_result_t apollo_config_load_dct_bluetooth(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result;
    /* Get BT configuration */
    result = wiced_dct_read_lock((void **)&dct_tables->dct_bt, WICED_TRUE, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't get BT configuration!\r\n");
    }
    return result;
}
#endif


wiced_result_t apollo_config_init(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    /* Get WiFi configuration */
    result = apollo_config_load_dct_wifi(dct_tables);

    /* Get network configuration */
    result |= apollo_config_load_dct_network(dct_tables);

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Get BT configuration */
    result |= apollo_config_load_dct_bluetooth(dct_tables);
#endif

    /* Get app specific data from Non Volatile DCT */
    result |= wiced_dct_read_lock((void **)&dct_tables->dct_app, WICED_TRUE, DCT_APP_SECTION, 0, sizeof(apollo_dct_t));

#if defined(OTA2_SUPPORT)
    {
        wiced_bool_t save_dct_tables = WICED_FALSE;
        result = apollo_ota2_check_boot_type_and_restore_DCT(dct_tables, &save_dct_tables);
        if ((result == WICED_SUCCESS) && (save_dct_tables == WICED_TRUE))
        {
            apollo_config_save(dct_tables);
        }
    }
#endif

    return result;
}

static wiced_result_t apollo_config_unload_dct_wifi(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if ((dct_tables != NULL) && (dct_tables->dct_wifi != NULL))
    {
        result = wiced_dct_read_unlock(dct_tables->dct_wifi, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_wifi = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't free/release WiFi configuration !\r\n");
        }
    }
    return result;
}

static wiced_result_t apollo_config_unload_dct_network(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if ((dct_tables != NULL) && (dct_tables->dct_network != NULL))
    {
        result = wiced_dct_read_unlock(dct_tables->dct_network, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_network = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't free/release network configuration !\r\n");
        }
    }
    return result;
}

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
static wiced_result_t apollo_config_unload_dct_bluetooth(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if ((dct_tables != NULL) && (dct_tables->dct_bt != NULL))
    {
        result = wiced_dct_read_unlock(dct_tables->dct_bt, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_bt = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't free/release BT configuration !\r\n");
        }
    }
    return result;
}
#endif

wiced_result_t apollo_config_deinit(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    result = apollo_config_unload_dct_wifi(dct_tables);
    result |= apollo_config_unload_dct_network(dct_tables);

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    result |= apollo_config_unload_dct_bluetooth(dct_tables);
#endif

    if (dct_tables->dct_app != NULL)
    {
        result |= wiced_dct_read_unlock(dct_tables->dct_app, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_app = NULL;
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't free/release app configuration !\r\n");
        }
    }

    return result;
}

wiced_result_t apollo_config_save(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = apollo_save_app_dct( dct_tables );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_save_app_dct() failed !\r\n");
    }

    result |= apollo_save_wifi_dct( dct_tables );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_save_wifi_dct() failed !\r\n");
    }

#if defined(OTA2_SUPPORT)
    result |= apollo_save_network_dct( dct_tables );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_save_network_dct() failed !\r\n");
    }
#endif

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    result |= apollo_save_bt_dct( dct_tables );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_save_bt_dct() failed !\r\n");
    }
#endif

    return result;
}

wiced_result_t apollo_config_reload_dct_wifi(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t  result;
    result = apollo_config_unload_dct_wifi(dct_tables);
    result |= apollo_config_load_dct_wifi(dct_tables);
    return result;
}

wiced_result_t apollo_config_reload_dct_network(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t  result;
    result = apollo_config_unload_dct_network(dct_tables);
    result |= apollo_config_load_dct_network(dct_tables);
    return result;
}

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
wiced_result_t apollo_config_reload_dct_bluetooth(apollo_dct_collection_t* dct_tables)
{
    wiced_result_t  result;
    result = apollo_config_unload_dct_bluetooth(dct_tables);
    result |= apollo_config_load_dct_bluetooth(dct_tables);
    return result;
}
#endif

