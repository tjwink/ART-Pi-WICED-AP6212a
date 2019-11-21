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

#include "wiced.h"
#include "wiced_duktape.h"
#include "platform_config.h"
#include "internal/wiced_internal_api.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "duk:mod:wifi"
#define LOG_DEBUG_ENABLE    0

#define MAC_STR_FMT         "%02x:%02x:%02x:%02x:%02x:%02x"
#define MIN_PASSPHRASE_LEN  (8)
#define MAX_PASSPHRASE_LEN  (64)

/* Global stash properties (not accessible from Ecmascript) */
#define GLOBAL_PROPERTY_SCAN_CALLBACK       "wifi_object_scan_callback"
#define GLOBAL_PROPERTY_CONNECT_SSID        "wifi_object_connect_ssid"
#define GLOBAL_PROPERTY_CONNECT_PASSPHRASE  "wifi_object_connect_passphrase"
#define GLOBAL_PROPERTY_CONNECT_SECURITY    "wifi_object_connect_security"
#define GLOBAL_PROPERTY_CONNECT_CALLBACK    "wifi_object_connect_callback"

#define GLOBAL_PROPERTY_LAST_STA_SSID       "wifi_object_last_sta_ssid"
#define GLOBAL_PROPERTY_LAST_STA_PASSPHRASE "wifi_object_last_sta_passphrase"
#define GLOBAL_PROPERTY_LAST_STA_STATUS     "wifi_object_last_sta_status"

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    AUTH_MODE_OPEN = 0,
    AUTH_MODE_WEP,
    AUTH_MODE_WPA_PSK,
    AUTH_MODE_WPA2_PSK,
    AUTH_MODE_WPA_WPA2_PSK,
    AUTH_MODE_MAX
} auth_mode_t;

typedef enum {
    STA_STATUS_OFF = 0,
    STA_STATUS_CONNECTING,
    STA_STATUS_WRONG_PASSWORD,
    STA_STATUS_NO_AP_FOUND,
    STA_STATUS_CONNECT_FAIL,
    STA_STATUS_CONNECTED
} sta_status_t;

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

/******************************************************
 *               Function Definitions
 ******************************************************/

static auth_mode_t wifi_module_security_type_to_enum( wiced_security_t type )
{
    switch ( type )
    {
        case WICED_SECURITY_OPEN:
            return AUTH_MODE_OPEN;
        case WICED_SECURITY_WEP_PSK:
        case WICED_SECURITY_WEP_SHARED:
            return AUTH_MODE_WEP;
        case WICED_SECURITY_WPA_TKIP_PSK:
        case WICED_SECURITY_WPA_AES_PSK:
        case WICED_SECURITY_WPA_MIXED_PSK:
            return AUTH_MODE_WPA_PSK;
        case WICED_SECURITY_WPA2_AES_PSK:
        case WICED_SECURITY_WPA2_TKIP_PSK:
        case WICED_SECURITY_WPA2_MIXED_PSK:
            return AUTH_MODE_WPA2_PSK;
        default:
            return AUTH_MODE_MAX;
    }
}

static int wifi_module_bssid_to_str( wiced_mac_t* bssid, char* str )
{
    if ( bssid == NULL || str == NULL )
    {
        return -1;
    }

    sprintf( str, MAC_STR_FMT, bssid->octet[0], bssid->octet[1],
             bssid->octet[2], bssid->octet[3], bssid->octet[4],
             bssid->octet[5] );
    return 0;
}

static duk_ret_t wifi_module_scan_callback( duk_context* ctx )
{
    wiced_scan_result_t*    entry;
    wiced_scan_result_t*    next;
    int num_ap = 0;

     /* Arguments:
     * 0 = callback (required)
     */

    duk_require_pointer( ctx, 0 );
    entry = duk_get_pointer( ctx, 0 );

    /* Load the scan callback function from global stash */
    duk_push_global_stash( ctx ); /* -> [global] */
    duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_SCAN_CALLBACK );
    /* -> [global cb] */
    duk_push_this( ctx ); /* -> [global cb this] */

    while ( entry != NULL )
    {
        auth_mode_t auth_mode;
        char mac_str[strlen(MAC_STR_FMT) + 1];

        next = entry->next;

        /* Ignore APs with unsupported security types (for now) */
        auth_mode = wifi_module_security_type_to_enum(entry->security);
        if (auth_mode == AUTH_MODE_MAX)
        {
            goto next;
        }

        /* New object for each AP */
        duk_push_object(ctx); /* -> [... obj] */

        duk_push_int(ctx, entry->signal_strength); /* -> [... obj rssi] */
        duk_put_prop_string( ctx, -2, "rssi" ); /* -> [... obj] */

        duk_push_int(ctx, entry->channel); /* -> [... obj channel] */
        duk_put_prop_string( ctx, -2, "channel" ); /* -> [... obj] */

        duk_push_int(ctx, auth_mode ); /* -> [... obj authMode] */
        duk_put_prop_string( ctx, -2, "authMode" ); /* -> [... obj] */

        duk_push_boolean(ctx, !strlen((char *)entry->SSID.value));
        /* -> [... obj isHidden] */
        duk_put_prop_string( ctx, -2, "isHidden" ); /* -> [... obj] */

        duk_push_sprintf( ctx, "%s", (char *)entry->SSID.value );
        /* -> [... obj ssid] */
        duk_put_prop_string( ctx, -2, "ssid" ); /* -> [... obj] */

        wifi_module_bssid_to_str(&entry->BSSID, mac_str);
        duk_push_sprintf( ctx, "%s", mac_str ); /* -> [... obj mac] */
        duk_put_prop_string( ctx, -2, "mac" ); /* -> [... obj] */

        num_ap++;
next:
        free(entry);
        entry = next;
    }

    /* -> [global cb this obj ...] */

    LOGD("Found %u APs", num_ap);

    /* Call the callback */
    if ( duk_pcall_method( ctx, num_ap ) == DUK_EXEC_ERROR )
    {
        /* -> [global err] */

        LOGE( "Failed to call WiFi scan callback (%s)",
              duk_safe_to_string( ctx, -1 ));
    }
    /* -> [global retval/err] */

    duk_pop( ctx ); /* -> [global] */

    /* Remove scan callback from global stash */
    duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_SCAN_CALLBACK );

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static wiced_result_t wifi_module_scan_result_handler( wiced_scan_handler_result_t* malloced_result )
{
    static wiced_scan_result_t* results = NULL;
    wiced_result_t              ret;

    if ( malloced_result == NULL )
    {
        return WICED_ERROR;
    }

    malloc_transfer_to_curr_thread( malloced_result );

    if ( malloced_result->status == WICED_SCAN_INCOMPLETE )
    {
        wiced_scan_result_t*    entry;

        entry = malloc( sizeof( wiced_scan_result_t ));
        if ( entry == NULL )
        {
            LOGE( "Failed to allocate memory for scanned results" );
            ret = WICED_ERROR;
            goto out;
        }

        memcpy( entry, &malloced_result->ap_details,
                sizeof( wiced_scan_result_t ));

        /* Add it to the head of the list since ordering does not matter */
        entry->next = results;
        results = entry;
    }
    else
    {
        if ( wiced_duktape_schedule_work( malloced_result->user_data,
                                          wifi_module_scan_callback,
                                          results ) != WICED_SUCCESS )
        {
            LOGE( "Failed to schedule scan callback" );
            free( results );
        }

        /* Reset the results list for next time */
        results = NULL;
    }

    ret = WICED_SUCCESS;
out:
    /* It's our job to free the memory */
    free( malloced_result );

    return ret;
}

static duk_ret_t wifi_module_scan( duk_context* ctx )
{
    /* Arguments:
     * 0 = callback (required)
     */

    duk_require_function( ctx, 0 );

    /* A scan callback in the global stash means there is a scan in progress */
    duk_push_global_stash( ctx ); /* -> [global] */
    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_SCAN_CALLBACK ))
    {
        duk_pop( ctx ); /* -> [] */

        LOGE( "A scan is already in progress" );
        return DUK_RET_ERROR;
    }

    /* Save the scan callback into the global stash */
    duk_dup( ctx, 0 ); /* -> [global cb] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_SCAN_CALLBACK );
    /* -> [global] */
    duk_pop( ctx ); /* -> [] */

    duk_push_this( ctx ); /* -> [this] */
    if ( wiced_wifi_scan_networks( wifi_module_scan_result_handler,
                                   duk_get_heapptr( ctx, -1 )) != WICED_SUCCESS )
    {
        duk_pop( ctx ); /* -> [] */

        LOGE( "Failed to scan for WiFi networks" );
        return DUK_RET_ERROR;
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t wifi_module_connect_work( duk_context* ctx )
{
    wiced_ap_info_t details;
    const char*     ssid;
    duk_size_t      ssid_len;
    const char*     passphrase;

    memset( &details, 0, sizeof( details ));

    /* Get the SSID, passphrase, and security details from global stash */

    duk_push_global_stash( ctx ); /* -> [global] */

    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SSID ) != 1 ||
         duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_PASSPHRASE ) != 1 ||
         duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SECURITY ) != 1 )
    {
        LOGE( "Missing WiFi connect parameters- aboring Wifi connect" );
        goto cleanup;
    }

    duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SSID );
    /* -> [global ssid] */
    ssid = duk_to_lstring( ctx, -1, &ssid_len );
    memcpy( details.SSID.value, duk_to_string( ctx, -1 ), MIN( ssid_len, 32 ));
    details.SSID.length = ssid_len;

    duk_get_prop_string( ctx, -2, GLOBAL_PROPERTY_CONNECT_PASSPHRASE );
    /* -> [global ssid passphrase] */
    passphrase = duk_to_string( ctx, -1 );

    duk_get_prop_string( ctx, -3, GLOBAL_PROPERTY_CONNECT_SECURITY );
    /* -> [global ssid passphrase security] */
    details.security = duk_to_int( ctx, -1 );

    if ( wiced_join_ap_specific( &details, strlen( passphrase ),
                                 passphrase ) != WICED_SUCCESS )
    {
        LOGE( "Failed to join AP '%s'", ssid );
        duk_pop_3( ctx ); /* -> [global] */
        duk_push_int( ctx, STA_STATUS_CONNECT_FAIL ); /* -> [global status] */
        duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
        /* -> [global] */

        goto cleanup;
    }

    /* Success! Save the AP and passphrase for reference */
    duk_pop( ctx );  /* -> [global ssid passphrase] */
    duk_put_prop_string( ctx, -3, GLOBAL_PROPERTY_LAST_STA_PASSPHRASE );
    /* -> [global ssid] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_SSID );
    /* -> [global] */

    LOGI("Requesting IP from DHCP server");
    if ( wiced_ip_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER,
              NULL ) != WICED_SUCCESS )
    {
        LOGE( "Failed to obtain IP address" );
        duk_push_int( ctx, STA_STATUS_CONNECT_FAIL ); /* -> [global status] */
        duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
        /* -> [global] */

        goto cleanup;
    }

    duk_push_int( ctx, STA_STATUS_CONNECTED ); /* -> [global status] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
    /* -> [global] */

    /* Call the callback */
    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_CALLBACK ))
    {
        duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_CALLBACK );
        /* -> [global cb] */
        duk_push_this( ctx ); /* -> [global cb this] */

        LOGD( "Calling WiFi connect callback" );
        if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
        {
            /* -> [global err] */

            LOGE( "Failed to call WiFi connect callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [global retval/err] */

        duk_pop( ctx ); /* -> [global] */
    }

    /* -> [global] */

cleanup:
    /* Remove SSID, passphrase, security details, callback from global stash */
    duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SSID );
    duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_PASSPHRASE );
    duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SECURITY );
    duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_CALLBACK );
    duk_pop( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t wifi_module_connect( duk_context* ctx )
{
    wiced_scan_result_t ap_info;
    const char*         ssid;
    duk_size_t          ssid_len;
    char                passphrase[MAX_PASSPHRASE_LEN+1] = "";

    /* Arguments:
     * 0 = SSID     (required)
     * 1 = options  (object, optional)
     * 2 = callback (optional)
     */

    /* A WiFi connect SSID property in the global stash means there is already
     * a connect in progress */
    duk_push_global_stash( ctx ); /* -> [global] */
    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SSID ))
    {
        duk_pop( ctx ); /* -> [] */

        LOGE( "A WiFi connect is already in progress" );
        return DUK_RET_ERROR;
    }
    duk_pop( ctx ); /* -> [] */

    /* SSID is a requirement- can't do much without it */
    ssid = duk_require_lstring( ctx, 0, &ssid_len );
    if ( ssid_len > SSID_NAME_SIZE )
    {
        LOGE( "SSID length too long (> %u characters long)",
              SSID_NAME_SIZE );
        return DUK_RET_ERROR;
    }

    if ( duk_has_prop_string( ctx, 1, "password" ))
    {
        duk_get_prop_string( ctx, 1, "password" ); /* -> [password] */
        if ( strlen( duk_to_string( ctx, -1 )) > MAX_PASSPHRASE_LEN )
        {
            LOGE( "Password length too long (> %u characters long)",
                  MAX_PASSPHRASE_LEN );
            duk_pop( ctx ); /* -> [] */
            return DUK_RET_ERROR;
        }

        strcpy( passphrase, duk_to_string( ctx, -1 ));
        duk_pop( ctx ); /* -> [] */
    }

    LOGD( "WiFi connect SSID='%s' password='%s'", ssid, passphrase );

    /* Check if we are already connected to an AP */
    if ( wwd_wifi_is_ready_to_transceive( WWD_STA_INTERFACE ) == WWD_SUCCESS )
    {
        duk_push_global_stash( ctx ); /* -> [global] */

        if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_SSID ) &&
             duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_PASSPHRASE ))
        {
            const char* last_sta_ssid;
            const char* last_sta_passphrase;

            duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_SSID );
            /* -> [global ssid] */
            last_sta_ssid = duk_to_string( ctx, -1 );
            duk_get_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_PASSPHRASE );
            /* -> [global ssid passphrase] */
            last_sta_passphrase = duk_to_string( ctx, -1 );

            if (( strcmp( ssid, last_sta_ssid ) == 0 ) &&
                ( strcmp( passphrase, last_sta_passphrase ) == 0 ))
            {
                LOGI( "Already connected to this AP" );

                /* Call the callback if specified */
                if ( duk_is_function( ctx, 2 ))
                {
                    duk_dup( ctx, 2 ); /* -> [... cb] */
                    duk_push_this( ctx ); /* -> [... cb this] */
                    if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
                    {
                        /* -> [... err] */

                        LOGE( "Failed to call WiFi connect callback (%s)",
                              duk_safe_to_string( ctx, -1 ));
                    }
                    /* -> [... retval/err] */

                    duk_pop( ctx ); /* -> [...] */
                }

                /* -> [global ssid passphrase] */

                duk_pop_3( ctx ); /* -> [] */

                return 0;
            }

            duk_pop_2( ctx ); /* -> [global] */
        }

        /* -> [global] */

        /* Disconnect from current AP */
        LOGI( "Leaving AP" );
        wiced_network_down( WICED_STA_INTERFACE );

        duk_push_int( ctx, STA_STATUS_OFF ); /* -> [global status] */
        duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
        /* -> [global] */

        duk_pop( ctx ); /* -> [] */
    }

    LOGD( "Looking for AP '%s'", ssid );
    if ( wiced_wifi_find_ap( ssid, &ap_info, NULL ) != WICED_SUCCESS )
    {
        //TODO How do we handle hidden SSIDs?
        LOGE( "Could not find AP '%s'", ssid );

        duk_push_global_stash( ctx ); /* -> [global] */
        duk_push_int( ctx, STA_STATUS_NO_AP_FOUND ); /* -> [global status] */
        duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
        /* -> [global] */
        duk_pop( ctx ); /* -> [] */

        return DUK_RET_ERROR;
    }

#ifdef DEBUG
    LOGD("Found AP:");
    print_scan_result( &ap_info );
#endif

    if ( ap_info.security != WICED_SECURITY_OPEN )
    {
        if ( strlen( passphrase ) < MIN_PASSPHRASE_LEN )
        {
            LOGE( "Password too short for this AP" );

            duk_push_global_stash( ctx ); /* -> [global] */
            duk_push_int( ctx, STA_STATUS_WRONG_PASSWORD );
            /* -> [global status] */
            duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
            /* -> [global] */
            duk_pop( ctx ); /* -> [] */

            return DUK_RET_ERROR;
        }
    }

    /* Save the SSID, password, security details, callback to global stash */
    duk_push_global_stash( ctx ); /* -> [global] */
    duk_push_string( ctx, ssid ); /* -> [global ssid] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_CONNECT_SSID );
    /* -> [global] */
    duk_push_string( ctx, passphrase ); /* -> [global passphrase] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_CONNECT_PASSPHRASE );
    /* -> [global] */
    duk_push_int( ctx, ap_info.security ); /* -> [global security] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_CONNECT_SECURITY );
    /* -> [global] */
    if ( duk_is_function( ctx, 2 ))
    {
        duk_dup( ctx, 2 ); /* -> [global cb] */
        duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_CONNECT_CALLBACK );
        /* -> [global] */
    }

    duk_pop( ctx ); /* -> [] */

    if ( wiced_duktape_schedule_work( NULL, wifi_module_connect_work,
                                      NULL ) != WICED_SUCCESS )
    {
        /* Remove thread, SSID, passphrase, security details, and 'this' from
         * global stash
         */
        duk_push_global_stash( ctx ); /* -> [global] */
        duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SSID );
        duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_PASSPHRASE );
        duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_SECURITY );
        duk_del_prop_string( ctx, -1, GLOBAL_PROPERTY_CONNECT_CALLBACK );
        duk_pop( ctx ); /* -> [] */

        LOGE( "Failed to create connect thread" );
        return DUK_RET_ERROR;
    }

    return 0;
}

static duk_ret_t wifi_module_disconnect(duk_context *ctx)
{
    /* Arguments:
     * 0 = callback (optional)
     */

    if ( wwd_wifi_is_ready_to_transceive( WWD_STA_INTERFACE ) == WWD_SUCCESS )
    {
        /* Disconnect from current AP */
        LOGI( "Leaving AP" );
        wiced_network_down( WICED_STA_INTERFACE );
    }

    duk_push_global_stash( ctx ); /* -> [global] */
    duk_push_int( ctx, STA_STATUS_OFF ); /* -> [global status] */
    duk_put_prop_string( ctx, -2, GLOBAL_PROPERTY_LAST_STA_STATUS );
    /* -> [global] */
    duk_pop( ctx ); /* -> [] */

    if ( duk_is_function( ctx, 0 ))
    {
        duk_dup( ctx, 0 ); /* -> [cb] */
        duk_push_this( ctx ); /* -> [cb this] */
        if ( duk_pcall_method( ctx, 0 ) == DUK_EXEC_ERROR )
        {
            /* -> [err] */

            LOGE( "Failed to call WiFi disconnect callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [retval/err] */

        duk_pop( ctx ); /* -> [] */
    }

    return 0;
}

static duk_ret_t wifi_module_get_hostname(duk_context *ctx)
{
    wiced_hostname_t hostname;

    /* Arguments:
     * 0 = callback (optional)
     */

    if ( wiced_network_get_hostname( &hostname ) == WICED_SUCCESS )
    {
        duk_push_sprintf( ctx, "%s", hostname.value ); /* -> [hostname] */
    }
    else
    {
        duk_push_string( ctx, "" ); /* -> [hostname] */
    }

    if ( duk_is_function( ctx, 0 ))
    {
        /* Duplicate the callback and return object on stack */
        duk_dup( ctx, 0 ); /* -> [hostname cb] */
        duk_push_this( ctx ); /* -> [hostname cb this] */
        duk_dup( ctx, -3 );  /* -> [hostname cb this hostname] */

        if ( duk_pcall_method( ctx, 1 ) == DUK_EXEC_ERROR )
        {
            /* -> [hostname err] */

            LOGE( "Failed to call WiFi disconnect callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [hostname retval/err] */

        duk_pop( ctx ); /* -> [hostname] */
    }

    /* -> [hostname] */
    return 1;
}

static duk_ret_t wifi_module_set_hostname( duk_context* ctx )
{
    const char* hostname;
    duk_size_t  hostname_len;

    /* Arguments:
     * 0 = hostname (required)
     */

    hostname = duk_require_lstring( ctx, 0, &hostname_len );

    if ( hostname_len > HOSTNAME_SIZE )
    {
        LOGE( "Hostname '%s' too long (> %u characters)", hostname,
              HOSTNAME_SIZE );
        return DUK_RET_ERROR;
    }

    if ( wiced_network_set_hostname( hostname ) != WICED_SUCCESS )
    {
        LOGE( "Failed to set hostname" );
        return DUK_RET_ERROR;
    }

    return 0;
}

static const char* wifi_module_sta_status_str( duk_context* ctx )
{
    sta_status_t last_sta_status = STA_STATUS_OFF;

    duk_push_global_stash( ctx ); /* -> [global] */

    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_STATUS ))
    {
        duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_STATUS );
        /* -> [global status] */
        last_sta_status = duk_to_int( ctx, -1 );
        duk_pop( ctx ); /* -> [global] */
    }
    duk_pop( ctx ); /* -> [] */

    switch ( last_sta_status )
    {
        case STA_STATUS_CONNECTING:
            return "connecting";
        case STA_STATUS_WRONG_PASSWORD:
            return "wrong_password";
        case STA_STATUS_NO_AP_FOUND:
            return "no_ap_found";
        case STA_STATUS_CONNECT_FAIL:
            return "connect_fail";
        case STA_STATUS_CONNECTED:
            return "connected";
        case STA_STATUS_OFF:
        default:
            return "off";
    }

    /* Unreachabe */
    return "off";
}

static duk_ret_t wifi_module_get_details( duk_context* ctx )
{
    int32_t                     rssi;
    platform_dct_wifi_config_t* dct_wifi_config;

    /* Arguments:
     * 0 = callback (optional)
     */

    /* New object for passing back information */
    duk_push_object( ctx ); /* -> [obj] */

    duk_push_string( ctx, wifi_module_sta_status_str( ctx )); /* -> [obj status] */
    duk_put_prop_string( ctx, -2, "status" ); /* -> [obj] */

    duk_push_global_stash( ctx ); /* -> [obj global] */

    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_SSID ))
    {
        duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_SSID );
        /* -> [obj global ssid] */
    }
    else
    {
        duk_push_null( ctx ); /* -> [obj global null] */
    }
    duk_put_prop_string( ctx, -3, "ssid" ); /* -> [obj global] */

    if ( duk_has_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_PASSPHRASE ))
    {
        duk_get_prop_string( ctx, -1, GLOBAL_PROPERTY_LAST_STA_PASSPHRASE );
        /* -> [obj global passphrase] */
    }
    else
    {
        duk_push_null( ctx ); /* -> [obj global null] */
    }
    duk_put_prop_string( ctx, -3, "password" ); /* -> [obj global] */

    duk_pop( ctx ); /* -> [obj] */

    if ( wiced_network_is_ip_up( WICED_STA_INTERFACE ))
    {
        wwd_wifi_get_rssi( &rssi );
        duk_push_int( ctx, rssi ); /* -> [obj rssi] */
        duk_put_prop_string( ctx, -2, "rssi" ); /* -> [obj] */
    }

    /* Read DCT to get saved AP settings */
    wiced_dct_read_lock( (void **)&dct_wifi_config, WICED_FALSE,
                         DCT_WIFI_CONFIG_SECTION, 0,
                         sizeof( platform_dct_wifi_config_t ));
    duk_push_sprintf( ctx, "%s",
                      dct_wifi_config->stored_ap_list[0].details.SSID.value );
    /* -> [obj savedSsid] */
    duk_put_prop_string( ctx, -2, "savedSsid" ); /* -> [obj] */
    wiced_dct_read_unlock( (void *)dct_wifi_config, WICED_FALSE );

    if ( duk_is_function( ctx, 0 ))
    {
        /* Duplicate the callback and return object on stack */
        duk_dup( ctx, 0 ); /* -> [obj cb] */
        duk_push_this( ctx ); /* -> [obj cb this] */
        duk_dup( ctx, -3 ); /* -> [obj cb this obj] */

        if ( duk_pcall_method( ctx, 1 ) == DUK_EXEC_ERROR )
        {
            /* -> [obj err] */

            LOGE( "Failed to call WiFi getDetails callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [obj retval/err] */

        duk_pop( ctx ); /* -> [obj] */
    }

    /* -> [obj] */
    return 1;
}

static duk_ret_t wifi_module_get_ip( duk_context* ctx )
{
    wiced_mac_t mac;

    /* Arguments:
     * 0 = callback (optional)
     */

    /* New object for passing back information */
    duk_push_object( ctx ); /* -> [obj] */

    /* Get IP adress info if we are connected */
    if ( wiced_network_is_ip_up( WICED_STA_INTERFACE ))
    {
        wiced_ip_address_t ipv4_addr;
        wiced_ip_address_t netmask;
        wiced_ip_address_t gateway;

        if ( wiced_ip_get_ipv4_address( WICED_STA_INTERFACE,
                                        &ipv4_addr ) == WICED_SUCCESS )
        {
            duk_push_sprintf( ctx, "%u.%u.%u.%u",
                              (uint8_t)( GET_IPV4_ADDRESS(ipv4_addr) >> 24 ),
                              (uint8_t)( GET_IPV4_ADDRESS(ipv4_addr) >> 16 ),
                              (uint8_t)( GET_IPV4_ADDRESS(ipv4_addr) >> 8  ),
                              (uint8_t)( GET_IPV4_ADDRESS(ipv4_addr) >> 0  ));
            /* -> [obj ip] */
            duk_put_prop_string( ctx, -2, "ip" ); /* -> [obj] */
        }

        if ( wiced_ip_get_netmask( WICED_STA_INTERFACE,
                                   &netmask ) == WICED_SUCCESS )
        {
            duk_push_sprintf( ctx, "%u.%u.%u.%u",
                              (uint8_t)( GET_IPV4_ADDRESS(netmask) >> 24 ),
                              (uint8_t)( GET_IPV4_ADDRESS(netmask) >> 16 ),
                              (uint8_t)( GET_IPV4_ADDRESS(netmask) >> 8  ),
                              (uint8_t)( GET_IPV4_ADDRESS(netmask) >> 0  ));
            /* -> [obj netmask] */
            duk_put_prop_string( ctx, -2, "netmask" ); /* -> [obj] */
        }

        if ( wiced_ip_get_gateway_address( WICED_STA_INTERFACE,
                                           &gateway ) == WICED_SUCCESS )
        {
            duk_push_sprintf( ctx, "%u.%u.%u.%u",
                              (uint8_t)( GET_IPV4_ADDRESS(gateway) >> 24 ),
                              (uint8_t)( GET_IPV4_ADDRESS(gateway) >> 16 ),
                              (uint8_t)( GET_IPV4_ADDRESS(gateway) >> 8  ),
                              (uint8_t)( GET_IPV4_ADDRESS(gateway) >> 0  ));
            /* -> [obj gw] */
            duk_put_prop_string( ctx, -2, "gw" ); /* -> [obj] */
        }
    }

    wwd_wifi_get_mac_address( &mac, WWD_STA_INTERFACE );
    duk_push_sprintf( ctx, MAC_STR_FMT, mac.octet[0], mac.octet[1],
                      mac.octet[2], mac.octet[3], mac.octet[4], mac.octet[5] );
    /* -> [obj mac] */
    duk_put_prop_string( ctx, -2, "mac" ); /* -> [obj] */

    if ( duk_is_function( ctx, 0 ))
    {
        /* Duplicate the callback and return object on stack */
        duk_dup( ctx, 0 ); /* -> [obj cb] */
        duk_push_this( ctx ); /* -> [obj cb this] */
        duk_dup( ctx, -3 );  /* -> [obj cb this obj] */

        if ( duk_pcall_method( ctx, 1 ) == DUK_EXEC_ERROR )
        {
            /* -> [obj err] */

            LOGE( "Failed to call WiFi getIP callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [obj retval/err] */

        duk_pop( ctx ); /* -> [obj] */
    }

    /* -> [obj] */
    return 1;
}

static duk_function_list_entry wifi_module_funcs[] =
{
      /* Name */        /* Function */              /* nargs */
    { "scan",           wifi_module_scan,           1 },
    { "connect",        wifi_module_connect,        3 },
    { "disconnect",     wifi_module_disconnect,     1 },
    { "getHostname",    wifi_module_get_hostname,   1 },
    { "setHostname",    wifi_module_set_hostname,   1 },
    { "getDetails",     wifi_module_get_details,    1 },
    { "getIP",          wifi_module_get_ip,         1 },
    { NULL, NULL, 0 }
};

duk_ret_t wiced_duktape_module_wifi_register( duk_context* ctx )
{
    duk_push_object( ctx ); /* -> [obj] */
    duk_put_function_list( ctx, -1, wifi_module_funcs ); /* -> [obj] */

    return 1;
}

