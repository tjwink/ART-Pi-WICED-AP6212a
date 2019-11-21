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
 * Wi-Fi MESH demo application
 *
 * This application snippet demonstrates how to use
 * the WICED Wi-Fi MESH interface and build a self-organizing MESH network
 *
 * Features demonstrated
 *  - Wi-Fi MESH client mode (to send a regular ICMP ping to an another MESH node)
 *  - Wi-Fi MESH bridge mode (to forward traffic between MESH peers)
 *  - Wi-FI MESH Routing
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Modify the SOFT_AP_SSID/SOFT_AP_PASSPHRASE Wi-Fi credentials
 *     as desired
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Sets up a MESH network with one node. Ping results are
 *    printed to the UART and appear on the terminal
  *
 * To connect another MESH peer, build another instance of this application and
 * FLASH anotehr board with the same SSID configured in wifi_config_dct.h
 *
 * TROUBLESHOOTING
 *  - TBD
 *
 */

#include "wiced.h"
#include "resources.h"
#ifdef __IAR_SYSTEMS_ICC__
  #include "iar_unistd.h"
#else
  #include "unistd.h"
#endif
#include "wiced_log.h"
#include "command_console_commands.h"
#include "dct/command_console_dct.h"
#include "internal/wiced_internal_api.h"
#include "wwd_buffer_interface.h"
#include "internal/wwd_sdpcm.h"
#include "iperf.h"

#include "wifi_mesh_config.h"
#include "include/wwd_events.h"

#define ETHER_ISNULLADDR(ea)    ((((const uint8_t *)(ea))[0] |        \
                  ((const uint8_t *)(ea))[1] |        \
                  ((const uint8_t *)(ea))[2] |        \
                  ((const uint8_t *)(ea))[3] |        \
                  ((const uint8_t *)(ea))[4] |        \
                  ((const uint8_t *)(ea))[5]) == 0)

typedef struct
{
    char *cmd;
    uint32_t event;
} cmd_lookup_t;

int mesh_console_command(int argc, char *argv[]);
#define MESH_COMMANDS \
    { (char*) "mesh_status",  mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Get mesh status" }, \
    { (char*) "mesh_block",   mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Block packets from specific mac address, used for development" }, \
    { (char*) "mesh_route",   mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Dump routing table" }, \
    { (char*) "mesh_gateway_status", mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Dump gateway info"}, \
    { (char*) "iperf",  iperf,        0, NULL, NULL, NULL, "Run iperf --help for usage."}, \
    { (char*) "config",       mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Display / change config values" }, \
    { (char*) "mesh_migrate",  mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Sniff AP's channel and migrate mesh network to that channel." }, \
    { (char*) "mesh_start_STA",  mesh_console_command, 0, NULL, NULL, (char *)"", (char *)"Start the STA interface." }, \

typedef enum
{
    MESH_CONSOLE_CMD_STATUS = 0,
    MESH_CONSOLE_CMD_BLOCK,
    MESH_CONSOLE_CMD_ROUTE_DUMP,
    MESH_CONSOLE_CMD_GATEWAY_STATUS,
    MESH_CONSOLE_CMD_CONFIG,
    MESH_CONSOLE_CMD_MIGRATE,
    MESH_CONSOLE_CMD_START_STA,
    MESH_CONSOLE_CMD_MAX,
} MESH_CONSOLE_CMDS_T;

typedef enum
{
    MESH_AUTH_PROTO_NONE = 0,
    MESH_AUTH_PROTO_SAE,
    MESH_AUTH_PROTO_8021x
} MESH_AUTH_PROTO_T;

static cmd_lookup_t command_lookup[MESH_CONSOLE_CMD_MAX] =
{
    { "mesh_status",   0  },
    { "mesh_block",    0  },
    { "mesh_route",    0  },
    { "mesh_gateway_status",  0  },
    { "config",        MESH_CONSOLE_CMD_CONFIG  },
    { "mesh_migrate",  0  },
    { "mesh_start_STA",0  },
};

/******************************************************
 *                      Macros
 ******************************************************/

#define MESH_PEERING_STATE_STRINGS \
    {"IDLE  ", "OPNSNT", "CNFRCV", "OPNRCV", "ESTAB ", "HOLDNG"}

#define MAX_MESH_SELF_PEER_ENTRY_RETRIES    3
#define MESH_SELF_PEER_ENTRY_STATE_ACTIVE   1
#define MESH_SELF_PEER_ENTRY_STATE_TIMEDOUT 2

#define MACDBG "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STRDBG(ea) (ea)[0], (ea)[1], (ea)[2], (ea)[3], (ea)[4], (ea)[5]
#define MACADDR_SZ 6

#define CHECK_RETURN( expr )  { wiced_result_t check_res = (expr); if ( check_res != WICED_SUCCESS ) { WPRINT_APP_INFO( ("Command %s failed \n", #expr) ); } }

#define IS_GATE(x) (x & WIFI_FLAG_MESH_GATEWAY)

/******************************************************
 *                    Constants
 ******************************************************/
#define MESH_DEFAULT_CHANNEL    36
#define BUFSZ                   1024

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

static void mesh_print_peer_info(mesh_peer_info_ext_t *mpi_ext, uint32_t peer_results_count);
static void* mesh_events_handler( const wwd_event_header_t* event_header, const uint8_t* event_data, void* handler_user_data );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_ip_setting_t mesh_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  1,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  1,  1 ) ),
};

static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static const command_t commands[] =
{
    ALL_COMMANDS
    MESH_COMMANDS
    WL_COMMANDS
    DCT_CONSOLE_COMMANDS
    CMD_TABLE_END
};

static const wwd_event_num_t mesh_events[] = {WLC_E_MESH_PAIRED, WLC_E_MESH_DHCP_SUCCESS, WLC_E_NONE};
static volatile wiced_bool_t dhcp_requested = WICED_FALSE;
static volatile uint8_t dhcp_success = 0;

/* mesh application DCT info */

wifi_mesh_dct_t     *wifi_mesh_app_dct_data;     /* there's only 10 mac addresses, so this will be small */

/******************************************************
 *               Function Definitions
 ******************************************************/

static int render_log_output(WICED_LOG_LEVEL_T level, char *logmsg)
{
    UNUSED_PARAMETER(level);

    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
int g_channel;
static wiced_semaphore_t scan_semaphore;

static int send_csa(int channel);
static int read_AP_SSID_from_DCT(wiced_ssid_t *ssid);
static int scan_for_ssid( wiced_interface_t interface, wiced_ssid_t *ssid, int *ap_channel );
static wiced_result_t mesh_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
static wiced_result_t migrate_second_STA(int chan);
#endif


void application_start(void)
{
    platform_dct_wifi_config_t*     dct_wifi_config          = NULL;
    platform_dct_misc_config_t*     misc_dct                 = NULL;
    wiced_ssid_t mesh_ssid = {
        .length = 7,
        .value = {'m', 'y', '_', 'm', 'e', 's', 'h'},
    };
    uint8_t channel = MESH_DEFAULT_CHANNEL;
    uint32_t wifi_flags;
    wiced_result_t          err;
    wiced_mac_t mac;
    int i;
    char *stream_argv[] = {NULL, "start"}; /* Used to enable wlog_streaming */
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    wiced_ssid_t  ap_ssid;
    int ap_channel;
#endif
    int use_wlog_stream = 0;  /* Change this to a non-zero value to enable continuous FW logging by default */

    /* Initialise the device */
    wiced_init();

    wiced_log_init(WICED_LOG_INFO, render_log_output, NULL);
    wiced_log_msg(WLF_DEF, WICED_LOG_INFO, "wiced logging system is initialized\n");

    if (use_wlog_stream) {
        wl_continuous(2, stream_argv);
    }

    /* read in the application DCT */
    wifi_mesh_config_init(&wifi_mesh_app_dct_data);

    if ( wiced_wifi_get_mac_address( &mac ) != WICED_SUCCESS ) {
        WPRINT_APP_INFO( ( "Failed to read device MAC!\n") );
        return;
    }

    /* get the wi-fi config section for modifying, any memory allocation required would be done inside wiced_dct_read_lock() */
    if ( wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( *dct_wifi_config ) ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Failed to read DCT Wifi config!\n") );
        return;
    }

    /* Wi-Fi Config Section */
    /* If the dct info is valid, adopt it. */
    if ( dct_wifi_config->device_configured &&
         dct_wifi_config->config_ap_settings.SSID.length != 0 &&
         dct_wifi_config->config_ap_settings.channel != 0 ) {

        mesh_ssid.length = dct_wifi_config->config_ap_settings.SSID.length;
        memcpy(mesh_ssid.value, dct_wifi_config->config_ap_settings.SSID.value, mesh_ssid.length);
        channel = dct_wifi_config->config_ap_settings.channel;
        mesh_ip_settings.ip_address.ip.v4 &= 0xFFFFFF00;
        mesh_ip_settings.ip_address.ip.v4 |= mac.octet[5];
    } else {
        WPRINT_APP_INFO( ( "Invalid DCT Wifi config!\n") );
        goto out;
    }

    /* get the Misc config section for modifying, any memory allocation required would be done inside wiced_dct_read_lock() */
    wiced_dct_read_lock( (void**) &misc_dct, WICED_TRUE, DCT_MISC_SECTION, 0, sizeof( *misc_dct ) );

    /* Set mesh value */
    misc_dct->wifi_flags |= WIFI_FLAG_MESH;
    wwd_wifi_set_flags( &misc_dct->wifi_flags, 0 );

    /* write it out */
    wiced_dct_write( (const void*) misc_dct, DCT_MISC_SECTION, 0, sizeof(*misc_dct) );

    /* release the read lock */
    wiced_dct_read_unlock( misc_dct, WICED_TRUE );

    /* Run the console ... */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );

    if (wwd_management_set_event_handler(mesh_events, mesh_events_handler, NULL, WWD_STA_INTERFACE) != WWD_SUCCESS) {
        WPRINT_APP_INFO(("Error setting MESH event handler\n"));
    }

    /* Read in blocked mac addresses (mostly for testing puproses) */
    for (i=0; i < WIFI_MESH_MAC_ADDRESSES; i++ ) {
        if (!ETHER_ISNULLADDR(wifi_mesh_app_dct_data->mac_address[i].octet)) {
            wwd_mesh_filter(&wifi_mesh_app_dct_data->mac_address[i], WICED_STA_INTERFACE);
            WPRINT_APP_INFO( ( "Block MAC %i: "MACDBG"\n", i, MAC2STRDBG(wifi_mesh_app_dct_data->mac_address[i].octet) ) );
        }
    }

    /* Get flags from DCT */
    wwd_wifi_get_flags(&wifi_flags, 0);

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    /* Not quite ready for this yet.
     * this section does in fact work fine...however if the DCT specifies one channel and
     * the GATEWAY AP is on another, when the non-gateway mesh peers boot up they will go to the
     * DCT channel leaving the Gateway STA all alone on the GATEWAY AP's channel.
     *
     * FOR THE MOMENT: DCT entry for mesh channel must match the channel of the Gateway AP.
     *
     */
    if (0) {
    /* Scan for the GATEWAY AP and get its channel
     * so we can start the mesh network on that same channel.
     */
    WPRINT_APP_INFO(("\n"));
    if (IS_GATE(wifi_flags)) {
        read_AP_SSID_from_DCT(&ap_ssid);
        scan_for_ssid(WICED_STA_INTERFACE, &ap_ssid, &ap_channel);
        if (ap_channel) {
            WPRINT_APP_INFO(("Gateway: Found %s on channel %d.\n", ap_ssid.value, ap_channel));
            channel = ap_channel;
        } else {
            WPRINT_APP_INFO(("\nGateway: Can't find AP, using default channel %d\n", channel));
        }
    }
    } /* if (0) */
#endif


    if ( dct_wifi_config->config_ap_settings.security == WICED_SECURITY_WPA2_AES_PSK )
    {
        wwd_set_mesh_auth_proto(MESH_AUTH_PROTO_8021x, WICED_STA_INTERFACE);
    }

    /* Bring up mesh interface */
    wwd_set_mesh_channel(channel, WICED_STA_INTERFACE); //XXX WHy special routine, what about wwd_wifi_set_channel()?
    wwd_join_mesh(&mesh_ssid, dct_wifi_config->config_ap_settings.security,
            (const uint8_t*)dct_wifi_config->config_ap_settings.security_key,
        dct_wifi_config->config_ap_settings.security_key_length, WICED_STA_INTERFACE);

    /*
     * Bring up the MESH STA interface.
     * If this is a gateway, bring up interface with a dhcp server or static depending on DHCP bit.
     * If non-gateway (e.g. peer), bring up interface with dhcp client or static depending on DHCP bit.
     * Static is default for the moment.
     */
    if (IS_GATE(wifi_flags)) {
        if (!(wifi_flags & WIFI_MESH_DHCP_IP)) {
            /* Let any scans from the previous join_mesh() complete */
            wiced_rtos_delay_milliseconds(2000);

            WPRINT_APP_INFO( ( "Gateway: Using Static addresses: To enable DHCP server do 'dct_misc_wifi mesh ip dhcp'\n") );
            if ((err = wiced_ip_up( WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &mesh_ip_settings )) != WICED_SUCCESS) {
                WPRINT_APP_INFO(( "Gateway: Failed to start mesh interface with static IP, Error = %d\n", err));
                goto out;
            }
        }
    } else {
        /* Not a gateway...just a regular mesh peer. */
        wwd_set_mesh_auto_peer(1, WICED_STA_INTERFACE); /* Start the peering process */
        WPRINT_APP_INFO( ( "\nMesh peer: To enable Gateway do 'dct_misc_wifi mesh gate 1'\n") );
        if (wifi_flags & WIFI_MESH_DHCP_IP) {
            WPRINT_APP_INFO( ( "Using DHCP addressing: To use static IP addressing do 'dct_misc_wifi mesh ip static'\n") );
            WPRINT_APP_INFO( ( "Waiting for mesh peering ...") );
            while ((dhcp_requested == WICED_FALSE)) {
                wiced_rtos_delay_milliseconds(2000);
                WPRINT_APP_INFO( ( ".") );
            }
            WPRINT_APP_INFO( ( "Peered.  \n") );
            WPRINT_APP_INFO( ( "Attempt to get DHCP address.....") );
            err = wiced_ip_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        } else {
            WPRINT_APP_INFO( ( "Using Static IP Addressing: To enable DHCP do 'dct_misc_wifi mesh ip dhcp'\n") );
            err = wiced_ip_up( WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &mesh_ip_settings );
        }
        if (err != WICED_SUCCESS) {
            WPRINT_APP_INFO( ( "Failed to get IP address! Error = %d\n", err) );
            goto out;
        }
    }

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    /* If this is a gateway, bring up second STA to connect to AP */
    if (IS_GATE(wifi_flags)) {

        /* Wait for beacons to come to establish tsf stream */
        wiced_rtos_delay_milliseconds(1000);

        /* Enable Gateway Announcements */
        CHECK_RETURN( wwd_wifi_set_iovar_value( "mesh_gateway", 1, WWD_STA_INTERFACE ) );
        WPRINT_APP_INFO(("\nGateway: Starting STA connection to AP.\n"));
        wiced_network_up(WICED_AP_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

        if (wifi_flags & WIFI_MESH_DHCP_IP) {
           WPRINT_APP_INFO( ( "MESH attempts DHCP with AP, waiting for AP connection ...\n") );
            while ((dhcp_success == 0)) {
                wiced_rtos_delay_milliseconds(2000);
                WPRINT_APP_INFO( ( ".") );
            }

            /* In gateway case, Wait for STA to join AP before enabling peering so that the external AP becomes the tsfmaster */
            wiced_rtos_delay_milliseconds(2000);
            /* Bring up mesh interface after infra is up */
            wwd_set_mesh_channel(channel, WICED_STA_INTERFACE); //XXX WHy special routine, what about wwd_wifi_set_channel()?
            wwd_join_mesh(&mesh_ssid, dct_wifi_config->config_ap_settings.security,
                    (const uint8_t*)dct_wifi_config->config_ap_settings.security_key,
               dct_wifi_config->config_ap_settings.security_key_length, WICED_STA_INTERFACE);
            WPRINT_APP_INFO( ( "Waiting for DHCP for MESH interface...\n") );
            if ((err = wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL)) != WICED_SUCCESS) {
                WPRINT_APP_INFO(( "Gateway: Failed to start mesh interface with DHCP IP, Error = %d\n", err));
                goto out;
            }

            while ((dhcp_success == 1)) {
                wiced_rtos_delay_milliseconds(2000);
                WPRINT_APP_INFO( ( ".") );
            }
        }

        /* In gateway case, Wait for STA to join AP before enabling peering so that the external AP becomes the tsfmaster */
      wwd_set_mesh_auto_peer(1, WICED_STA_INTERFACE);
    }
#endif
out:
    /* release the read lock */
    wiced_dct_read_unlock( dct_wifi_config, WICED_FALSE );
}

static void* mesh_events_handler( const wwd_event_header_t* event_header, const uint8_t* event_data, void* handler_user_data )
{
    UNUSED_PARAMETER(event_data);

    switch (event_header->event_type) {
        case WLC_E_MESH_PAIRED:
            dhcp_requested = WICED_TRUE;
            WPRINT_APP_INFO(("%s: Receieved MESH PAIRED event\n", __FUNCTION__));
            break;
        case WLC_E_MESH_DHCP_SUCCESS:
            dhcp_success++;
            WPRINT_APP_INFO(("%s: Receieved DHCP event (%d total)\n", __FUNCTION__, dhcp_success));
            break;
        default:
            WPRINT_APP_INFO(("%s: Unrecognized event: %d\n", __FUNCTION__, event_header->event_type));
            break;
    }
    return handler_user_data;
}

static wwd_result_t
mesh_route_dump( char *result_buf, uint16_t result_buf_sz )
{
    wiced_buffer_t buffer, response;
    char *data;
    wwd_result_t   result;

    data = wwd_sdpcm_get_iovar_buffer( &buffer, result_buf_sz, "mesh_route" );
    if (data == NULL) {
        WPRINT_APP_INFO(("%s: Alloc failed\n", __FUNCTION__));
        return WWD_BUFFER_ALLOC_FAIL;
    }

    result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WICED_STA_INTERFACE);
    if (result != WWD_SUCCESS)
    {
        host_buffer_release( response, WWD_NETWORK_RX );
        WPRINT_APP_INFO(("%s: send_iovar failed\n", __FUNCTION__));
        return WWD_BUFFER_ALLOC_FAIL;
    }

    data = (char*)host_buffer_get_current_piece_data_pointer( response );
    memcpy( result_buf, data, result_buf_sz );
    host_buffer_release( response, WWD_NETWORK_RX );

    return result;
}

static int
ether_atoe(const char *a, wiced_mac_t *n)
{
    char *c = NULL;
    int i = 0;

    memset(n, 0, ETHER_ADDR_LEN);
    for (;;) {
        n->octet[i++] = (uint8_t)strtoul(a, &c, 16);
        if (!*c++ || i == ETHER_ADDR_LEN)
            break;
        a = c;
    }
    return (i == ETHER_ADDR_LEN);
}

/* Handle console commands */
int
mesh_console_command(int argc, char *argv[])
{
    int i;
    for (i = 0; i < MESH_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= MESH_CONSOLE_CMD_MAX)
    {
        WPRINT_APP_INFO(("%s: Unrecognized command: %s\n", __FUNCTION__, argv[0]));
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case MESH_CONSOLE_CMD_STATUS:
        {
            mesh_peer_info_ext_t *mpi_ext;
            mesh_peer_info_dump_t *peer_results;
            char *result_buf;

            result_buf = malloc(BUFSZ);
            if(result_buf == NULL) {
                WPRINT_APP_INFO(("Malloc failed\n"));
                break;
            }

            if (wwd_mesh_status( result_buf, BUFSZ ) == WWD_SUCCESS) {
                peer_results = (mesh_peer_info_dump_t *)result_buf;
                mpi_ext = (mesh_peer_info_ext_t *)peer_results->mpi_ext;
                mesh_print_peer_info(mpi_ext, peer_results->count);
            } else {
                WPRINT_APP_INFO(("%s: mesh_status failed\n", __FUNCTION__));
            }
            free(result_buf);
            break;
        }

        case MESH_CONSOLE_CMD_ROUTE_DUMP:
        {
            char *result_buf;

            result_buf = malloc(BUFSZ);
            if(result_buf == NULL) {
                WPRINT_APP_INFO(("Malloc failed\n"));
                break;
            }
            if (mesh_route_dump( result_buf, BUFSZ ) == WWD_SUCCESS) {
                WPRINT_APP_INFO(("%s\n", result_buf));
            } else {
                WPRINT_APP_INFO(("Booh\n"));
            }
            free(result_buf);
            break;
        }

        case MESH_CONSOLE_CMD_BLOCK:
        {
            wiced_mac_t ea;
            wwd_result_t result;

            if (argc < 2) {
                WPRINT_APP_INFO(("Usage: mesh_block mac_addr\n"));
                break;
            }

            if (!ether_atoe(argv[1], &ea)) {
                WPRINT_APP_INFO(("Mac address parse failed, Format 00:11:22:33:44:55\n"));
                break;
            }

            result = wwd_mesh_filter(&ea, WICED_STA_INTERFACE);
            if (result != WWD_SUCCESS) {
                WPRINT_APP_INFO(("Unable to add filter, result %d\n", result));
                break;
            }
            WPRINT_APP_DEBUG(("Added filter for "MACDBG" \n", MAC2STRDBG(ea.octet)));

            break;
        }
        case MESH_CONSOLE_CMD_GATEWAY_STATUS:
        {
            /* Who is our gateway? */
            uint32_t wifi_flags;
            wwd_wifi_get_flags(&wifi_flags, 0);
            if (IS_GATE(wifi_flags)) {
                WPRINT_APP_INFO(("This mesh peer is a Mesh Gateway. Use 'status' command for more info.\n"));
            } else {
                uint8_t buf[120];
                WPRINT_APP_INFO(("The Mesh Gateway info for this peer:\n"));
                CHECK_RETURN( wwd_wifi_get_iovar_buffer( "mesh_gateway_list", buf, sizeof(buf), WWD_STA_INTERFACE ) );
                WPRINT_APP_INFO(("%s\n", buf));
            }
            break;
        }
        case MESH_CONSOLE_CMD_CONFIG:
        {
            wifi_mesh_config_set(wifi_mesh_app_dct_data, argc, argv);
            break;
        }
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
        case MESH_CONSOLE_CMD_MIGRATE:
        {
            migrate_second_STA(argc >=2 ? atoi(argv[1]) : 0);
            break;
        }
        case MESH_CONSOLE_CMD_START_STA:
        {
            wiced_network_up(WICED_AP_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
            break;
        }
#endif
    }
    return 0;
}

static void
mesh_print_peer_info(mesh_peer_info_ext_t *mpi_ext, uint32_t peer_results_count)
{
    char *peering_map[] = MESH_PEERING_STATE_STRINGS;
    uint32_t count = 0;

    WPRINT_APP_INFO(("no: ------addr------ : l.aid : state : p.aid : mppid :llid : plid : entry_state : rssi\n"));
    for (count = 0; count < peer_results_count; count++) {
        if (mpi_ext->entry_state != MESH_SELF_PEER_ENTRY_STATE_TIMEDOUT)
            WPRINT_APP_INFO(("%ld: "MACDBG" : 0x%x : %s : 0x%x : %4d : %4d : %4d : %s : %ld\n",
                count, MAC2STRDBG(mpi_ext->ea.octet), mpi_ext->local_aid,
                peering_map[mpi_ext->peer_info.state],
                mpi_ext->peer_info.peer_aid,
                mpi_ext->peer_info.mesh_peer_prot_id,
                mpi_ext->peer_info.local_link_id,
                mpi_ext->peer_info.peer_link_id,
                (mpi_ext->entry_state == MESH_SELF_PEER_ENTRY_STATE_ACTIVE) ?
                "ACTIVE" :
                "EXTERNAL",
                mpi_ext->rssi));
        else
            WPRINT_APP_INFO(("%ld: "MACDBG" : %s : %s : %s : %s : %s : %s : %s : %s\n",
                count, MAC2STRDBG(mpi_ext->ea.octet), "NA     ",
                "NA    ",
                "NA    ",
                "NA    ",
                "NA    ",
                "NA    ",
                "TIMEDOUT",
                "NA    "));

        mpi_ext++;
    }
}

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
#define WL_CHANSPEC_CHAN_MASK        0x00ff
#define CHSPEC_CHANNEL(chspec)    ((uint8_t)((chspec) & WL_CHANSPEC_CHAN_MASK))
#define CH20MHZ_CHSPEC(channel)    (chanspec_t)((chanspec_t)(channel) | WL_CHANSPEC_BW_20 | \
                WL_CHANSPEC_CTL_SB_NONE | (((channel) <= CH_MAX_2G_CHANNEL) ? \
                WL_CHANSPEC_BAND_2G : WL_CHANSPEC_BAND_5G))
#define NUM_CSA_BCN         8  /* Send this many CSA beacons, usually 100 ms per beacon */

/* Send Channel Switch Announcments */
static int send_csa(int channel)
{
    wwd_result_t err = WWD_SUCCESS;
    wiced_chan_switch_t cs;
    memset(&cs, 0, sizeof(wiced_chan_switch_t));
    cs.chspec = CH20MHZ_CHSPEC(channel);
    cs.count  = NUM_CSA_BCN;
    cs.mode   = 0;

    if ((err = wwd_wifi_send_csa(&cs, WWD_STA_INTERFACE)) != WWD_SUCCESS) {
        WPRINT_APP_INFO(("wwd_wifi_send_csa to %d failed\n", channel));
    }
    return 0;
}

/* The Gateway SSID is stored in DCT array index[1].
 * (The Mesh SSID is stored in DCT array[0].)
 */
static int read_AP_SSID_from_DCT(wiced_ssid_t  *g_ssid)
{
    unsigned int a = 1;
    wiced_config_ap_entry_t* ap;

    CHECK_RETURN( wiced_dct_read_lock( (void**) &ap, WICED_FALSE, DCT_WIFI_CONFIG_SECTION,
        (uint32_t) ( OFFSETOF(platform_dct_wifi_config_t,stored_ap_list) + a * sizeof(wiced_config_ap_entry_t) ), sizeof(wiced_config_ap_entry_t) ) );
    if ( ap->details.SSID.length ) {
        *g_ssid = ap->details.SSID;
    }
    wiced_dct_read_unlock( (wiced_config_ap_entry_t*) ap, WICED_FALSE );

    return 0;
}

/* Return the channel of the given SSID.  Use given interface to do the scan */
static int scan_for_ssid( wiced_interface_t interface, wiced_ssid_t *ssid, int *ap_channel )
{
    wiced_result_t result;

    /* Initialise the semaphore that will tell us when the scan is complete */
    wiced_rtos_init_semaphore(&scan_semaphore);

    if ( ( result = wiced_wifi_scan_networks_ex(mesh_scan_result_handler, NULL, WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_ANY,
                         ssid, NULL, NULL, NULL, interface) ) == WICED_SUCCESS ) {
        wiced_rtos_get_semaphore(&scan_semaphore, WICED_WAIT_FOREVER);
    } else {
        WPRINT_APP_INFO( ( "Error starting scan! Error=%d\n", result ) );
    }
    wiced_rtos_deinit_semaphore(&scan_semaphore);

    *ap_channel = g_channel;

    return ERR_CMD_OK;
}

static wiced_result_t mesh_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    if ( malloced_scan_result != NULL ) {
        malloc_transfer_to_curr_thread( malloced_scan_result );

        if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE ) {
            wiced_scan_result_t* record = &malloced_scan_result->ap_details;
            if (( record->bss_type == WICED_BSS_TYPE_INFRASTRUCTURE ) || ( record->bss_type == WICED_BSS_TYPE_ADHOC ) ) {
                ;
            } else {
                WPRINT_APP_INFO(("Bad bss_type\n"));
            }

            WPRINT_APP_DEBUG(( "Found %s on channel %d\n", record->SSID.value, record->channel));
            g_channel = record->channel;
        } else {
            wiced_rtos_set_semaphore(&scan_semaphore);
        }
        free( malloced_scan_result );
        malloced_scan_result = NULL;
    }
    return WICED_SUCCESS;
}


/* If user specifies a channel use it, otherwise scan for the SSID in the DCT and use that channel. */
static wiced_result_t migrate_second_STA(int chan)
{
    wiced_ssid_t  ap_ssid;
    int ap_channel;
    /* Migrate mesh network over to wherever AP is */
    if (!chan) {
        read_AP_SSID_from_DCT(&ap_ssid);
        scan_for_ssid(WICED_AP_INTERFACE, &ap_ssid, &ap_channel);
    } else {
        ap_channel = chan;
    }
    WPRINT_APP_INFO(("Not really: Pause while migrating Mesh network over to channel %d\n", ap_channel));
    return WICED_SUCCESS;

    send_csa(ap_channel);
    wiced_rtos_delay_milliseconds((NUM_CSA_BCN+1) * 100);
    return WICED_SUCCESS;
}
#endif
