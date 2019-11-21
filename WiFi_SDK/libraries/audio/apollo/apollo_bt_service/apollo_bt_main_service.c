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

/** @file Apollo audio application.
 *
 */

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_log.h"
#include "wiced_bt_stack.h"
#include "apollo_bt_service.h"
#include "apollo_bt_main_service_private.h"
#include "apollo_config_gatt_server_private.h"
#include "apollo_bt_nv.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BT_SERVICE_TIMER_THREAD_STACK_SIZE      (4096)
#define BT_SERVICE_TIMER_THREAD_PRIORITY        (WICED_DEFAULT_LIBRARY_PRIORITY)

#define BT_DEVICE_NAME_LEN                      (248)
#define BT_DEVICE_ENABLED_TIMEOUT_MSECS         (60000)
#define BT_LINK_LOSS_RETRY_INTERVAL             (5000)
#define BT_LINK_LOSS_NUM_RETRIES                (9)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum
{
    APOLLO_BT_SERVICE_EVENT_NONE             = 0x0,
    APOLLO_BT_SERVICE_EVENT_ENABLED_FAILURE  = 0x1,
    APOLLO_BT_SERVICE_EVENT_ENABLED_SUCCESS  = 0x2,
    APOLLO_BT_SERVICE_EVENT_DISABLED         = 0x4,
    APOLLO_BT_SERVICE_EVENT_ALL              = 0xFFFFFFFF,
} apollo_bt_service_event_t;

typedef enum
{
    APOLLO_BT_SERVICE_TIMER_EVENT_NONE       = 0x0,
    APOLLO_BT_SERVICE_TIMER_EVENT_RUN        = 0x1,
    APOLLO_BT_SERVICE_TIMER_EVENT_QUIT       = 0x2,
    APOLLO_BT_SERVICE_TIMER_EVENT_ALL        = 0xFFFFFFFF,
} apollo_bt_service_timer_event_t;

typedef struct
{
    wiced_event_flags_t            events;
    wiced_event_flags_t            timer_events;
    uint8_t                        bluetooth_device_name[BT_DEVICE_NAME_LEN + 1];
    wiced_bt_management_evt_data_t event_data;
    wiced_bt_smp_sc_local_oob_t    oob_data;
    wiced_bool_t                   connect_state;
    uint8_t                        connect_state_reason;
    wiced_bool_t                   timer_quit;
    wiced_bool_t                   timer_stop;
    wiced_thread_t                 timer_thread;
    wiced_thread_t                *timer_thread_ptr;
    uint8_t                        num_retries;
    uint8_t                        timer_thread_stack[BT_SERVICE_TIMER_THREAD_STACK_SIZE];
} apollo_bt_service_context_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

static apollo_bt_service_context_t  g_bt_service_ctx;
static apollo_bt_service_context_t *g_p_bt_service_ctx = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/

static void bt_reconnect_on_link_loss(void)
{
    apollo_bt_a2dp_sink_connect();
    return;
}


static void bt_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    g_bt_service_ctx.connect_state        = is_connected;
    g_bt_service_ctx.connect_state_reason = reason;

    wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "Connection status callback: is_connected=%d, reason=0x%x\n", is_connected, reason );

    if ( is_connected == WICED_FALSE )
    {
        if ( reason == HCI_ERR_CONNECTION_TOUT )
        {
            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "Device disconnected(link-loss). Starting Reconnection-Timer...\n");
            apollo_bt_service_reconnection_timer_start();
        }
        wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval);
    }
    else
    {
        apollo_bt_service_reconnection_timer_stop();
        wiced_bt_dev_set_discoverability(BTM_NON_DISCOVERABLE, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval);
    }
}


static void bt_linkloss_timer_handler( wiced_thread_arg_t arg )
{
    apollo_bt_service_context_t *p_bt_service_ctx = (apollo_bt_service_context_t *)arg;
    wiced_result_t               result;
    uint32_t                     events;

    while ( p_bt_service_ctx->timer_quit == WICED_FALSE )
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags( &g_bt_service_ctx.timer_events, APOLLO_BT_SERVICE_TIMER_EVENT_ALL, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: wiced_rtos_wait_for_event_flags() failed !!\n", __func__);
            continue;
        }

        if ( events & APOLLO_BT_SERVICE_TIMER_EVENT_QUIT )
        {
            break;
        }

        if ( events & APOLLO_BT_SERVICE_TIMER_EVENT_RUN )
        {
            while ( (p_bt_service_ctx->timer_quit == WICED_FALSE) && (p_bt_service_ctx->timer_stop == WICED_FALSE) && (p_bt_service_ctx->num_retries < BT_LINK_LOSS_NUM_RETRIES) )
            {
                wiced_rtos_delay_milliseconds(BT_LINK_LOSS_RETRY_INTERVAL);
                bt_reconnect_on_link_loss();
                p_bt_service_ctx->num_retries++;
            }
        }
    }
}


static wiced_bt_dev_status_t bluetooth_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_ble_pairing_info_t* p_info;
    wiced_bt_ble_advert_mode_t*      p_mode;
    wiced_bt_dev_status_t            status     = WICED_BT_SUCCESS;
    apollo_bt_paired_device_info_t   out_device;

    switch( event )
    {

        case BTM_ENABLED_EVT:
            g_bt_service_ctx.event_data.enabled.status = p_event_data->enabled.status;

            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
                wiced_result_t                 result;
                wiced_bt_device_address_t      bt_local_addr = BT_DEVICE_ADDRESS;
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
                platform_dct_bt_config_t*      dct_bt_config = NULL;
#endif /* WICED_DCT_INCLUDE_BT_CONFIG */
#ifdef USE_WICED_HCI
                uint8_t                        count = 0;
                wiced_bt_device_link_keys_t    link_keys;
#endif /* USE_WICED_HCI */

#ifdef DISABLE_BLUETOOTH_LPM
                if( wiced_bt_dev_get_low_power_mode() )
                {
                    result = wiced_bt_dev_set_low_power_mode(WICED_FALSE);
                    wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback setting low_power_mode: %d\r\n", (int)WICED_FALSE );
                    if( result != WICED_BT_SUCCESS )
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"bluetooth_management_callback: wiced_bt_dev_set_low_power_mode returns %d\n", (int)result);
                    }
                }
#endif /* DISABLE_BLUETOOTH_LPM */

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
                /* Read config */
                result = wiced_dct_read_lock( (void**) &dct_bt_config, WICED_TRUE, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t) );
                if ( result != WICED_SUCCESS )
                {
                    wiced_bt_set_local_bdaddr ( bt_local_addr );
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"bluetooth_management_callback: wiced_dct_read_lock(DCT_BT_CONFIG_SECTION) failed !!\n");
                }
                else
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: WICED DCT BT ADDR [%x:%x:%x:%x:%x:%x] \r\n",
                                  dct_bt_config->bluetooth_device_address[0], dct_bt_config->bluetooth_device_address[1],
                                  dct_bt_config->bluetooth_device_address[2], dct_bt_config->bluetooth_device_address[3],
                                  dct_bt_config->bluetooth_device_address[4], dct_bt_config->bluetooth_device_address[5]);
                    wiced_bt_set_local_bdaddr ( dct_bt_config->bluetooth_device_address );
                    wiced_dct_read_unlock( (void*) dct_bt_config, WICED_TRUE );
                }
#else /* !WICED_DCT_INCLUDE_BT_CONFIG */
                wiced_bt_set_local_bdaddr ( bt_local_addr );
#endif /* WICED_DCT_INCLUDE_BT_CONFIG */
                memset(bt_local_addr, 0, sizeof(bt_local_addr));
                wiced_bt_dev_read_local_addr( bt_local_addr );
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: LOCAL BT ADDRESS [%x:%x:%x:%x:%x:%x]\n",
                              bt_local_addr[0], bt_local_addr[1], bt_local_addr[2], bt_local_addr[3], bt_local_addr[4], bt_local_addr[5] );

                /**
                 * TODO Remove this, and make changes in the embedded app to solicit this information.
                 */
#ifdef USE_WICED_HCI
                for (count = 0; count < APOLLO_BT_NV_MAX_LAST_PAIRED_DEVICES; count++)
                {
                    if (apollo_bt_nv_get_device_info_by_index(count, &out_device) == WICED_BADARG)
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: no more entries in NVRAM\n");
                        break;
                    }
                    else
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: device addr from NVRAM %x:%x:%x:%x:%x:%x\n",
                                      out_device.device_link.bd_addr[0], out_device.device_link.bd_addr[1], out_device.device_link.bd_addr[2],
                                      out_device.device_link.bd_addr[3], out_device.device_link.bd_addr[4], out_device.device_link.bd_addr[5]);
                        memcpy(&link_keys, &out_device.device_link, sizeof(out_device.device_link));
                        wiced_bt_dev_push_nvram_data(&link_keys);
                    }
                }
#endif /* USE_WICED_HCI */

                result = wiced_bt_dev_register_connection_status_change( bt_connection_status_callback );
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: wiced_bt_dev_register_connection_status_change() returns %d\n", (int)result);

                wiced_rtos_set_event_flags( &g_bt_service_ctx.events, APOLLO_BT_SERVICE_EVENT_ENABLED_SUCCESS );
            }
            else
            {
                wiced_rtos_set_event_flags( &g_bt_service_ctx.events, APOLLO_BT_SERVICE_EVENT_ENABLED_FAILURE );
            }
            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Bluetooth enabled (%s)\n", ( ( p_event_data->enabled.status == WICED_BT_SUCCESS ) ? "success":"failure" ) );
            break;

        case BTM_DISABLED_EVT:
            wiced_rtos_set_event_flags( &g_bt_service_ctx.events, APOLLO_BT_SERVICE_EVENT_DISABLED );
            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Bluetooth disabled\n" );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:

            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;

            break;

        case BTM_PAIRING_COMPLETE_EVT:

            p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "Pairing Complete: %d\n",p_info->reason );
            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Pairing complete %i.\n", p_event_data->pairing_complete.pairing_complete_info.ble.status );

            break;

        case BTM_SECURITY_REQUEST_EVT:

            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );

            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: Security request\n" );

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            p_mode = &p_event_data->ble_advert_state_changed;
            if ( *p_mode == BTM_BLE_ADVERT_OFF )
            {
                apollo_config_advertisement_stopped( );
            }

            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: advertisement State Change=%d\n", *p_mode );

            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:

            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;

            wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "bluetooth_management_callback: IO capabilities BR/EDR request\n" );

            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:

            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: User confirmation request\n");

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:

            apollo_bt_nv_update_device_link_key( &p_event_data->paired_device_link_keys_update );

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Link key update evt\n");

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if (apollo_bt_nv_get_device_info_by_addr(&p_event_data->paired_device_link_keys_request.bd_addr, &out_device) == WICED_SUCCESS)
            {
                memcpy(&p_event_data->paired_device_link_keys_request.key_data, &out_device.device_link.key_data, sizeof( out_device.device_link.key_data ) );
            }
            else
            {
                status = WICED_BT_ERROR;
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Link key request evt\n");
        break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:

            apollo_bt_nv_update_local_id_keys( &p_event_data->local_identity_keys_update );

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bluetooth_management_callback: Local ID key update evt\n");

            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:

#ifndef USE_WICED_HCI
            if ( apollo_bt_nv_get_local_id_keys( &p_event_data->local_identity_keys_request ) != WICED_SUCCESS )
            {
                status = WICED_BT_ERROR;
            }
#endif /* #!USE_WICED_HCI*/

            break;

        case BTM_LPM_STATE_LOW_POWER:

            break;

        default:

            status = WICED_BT_ERROR;

            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "bluetooth_management_callback: Unhandled Bluetooth Management Event: 0x%x\n", event);

            break;
    }

    return ( status );
}


wiced_result_t apollo_bt_service_init( apollo_bt_service_init_params_t *params )
{
    wiced_result_t result                   = WICED_ERROR;
    uint32_t       bt_service_events        = 0;
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Configure the Device Name and Class of Device from the DCT */
    platform_dct_bt_config_t* dct_bt_config = NULL;
#endif

    wiced_action_jump_when_not_true( g_p_bt_service_ctx == NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: already initialized !\n") );

    wiced_action_jump_when_not_true( params != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service:  init params is NULL !\n") );

    g_p_bt_service_ctx = &g_bt_service_ctx;

    result = wiced_rtos_init_event_flags( &g_bt_service_ctx.events );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: init event flags failed !\n") );

    result = wiced_rtos_init_event_flags( &g_bt_service_ctx.timer_events );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: init timer event flags failed !\n") );

    g_p_bt_service_ctx->timer_quit  = WICED_FALSE;
    g_p_bt_service_ctx->timer_stop  = WICED_FALSE;
    g_p_bt_service_ctx->num_retries = 0;
    result = wiced_rtos_create_thread_with_stack( &g_bt_service_ctx.timer_thread, BT_SERVICE_TIMER_THREAD_PRIORITY, "apollo_bt_timer", bt_linkloss_timer_handler,
                                                  g_bt_service_ctx.timer_thread_stack, BT_SERVICE_TIMER_THREAD_STACK_SIZE,  &g_bt_service_ctx );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_create_thread_with_stack() failed !\n") );
    g_bt_service_ctx.timer_thread_ptr = &g_bt_service_ctx.timer_thread;

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Read config */
    result = wiced_dct_read_lock( (void**) &dct_bt_config, WICED_TRUE, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t) );
    if ( result != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: wiced_dct_read_lock(DCT_BT_CONFIG_SECTION) failed !!\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"Apollo BT service: WICED DCT BT NAME [%s]\n", dct_bt_config->bluetooth_device_name);
        strlcpy((char*)g_bt_service_ctx.bluetooth_device_name, (char*)dct_bt_config->bluetooth_device_name, sizeof(g_bt_service_ctx.bluetooth_device_name));
        wiced_bt_cfg_settings.device_name = g_bt_service_ctx.bluetooth_device_name;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"Apollo BT service: WICED DCT BT DEVICE CLASS [%02x %02x %02x]\n", dct_bt_config->bluetooth_device_class[0],
                      dct_bt_config->bluetooth_device_class[1], dct_bt_config->bluetooth_device_class[2]);
        memcpy( wiced_bt_cfg_settings.device_class, dct_bt_config->bluetooth_device_class, sizeof(dct_bt_config->bluetooth_device_class));
        wiced_dct_read_unlock( (void*) dct_bt_config, WICED_TRUE );
    }
#endif

    result = apollo_bt_nv_init( params->app_dct_offset );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg( WLF_AUDIO, WICED_LOG_ERR, "wiced_bt_stack_init: apollo_bt_nv_init() failed\n" ) );

    result = wiced_bt_stack_init( bluetooth_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg( WLF_AUDIO, WICED_LOG_ERR, "Apollo BT service: wiced_bt_stack_init() failed !\n" ) );

    result = wiced_rtos_wait_for_event_flags( &g_bt_service_ctx.events, APOLLO_BT_SERVICE_EVENT_ALL, &bt_service_events, WICED_TRUE, WAIT_FOR_ANY_EVENT, BT_DEVICE_ENABLED_TIMEOUT_MSECS );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: wait for event flags failed with %d !\n", result) );

    if ( bt_service_events & APOLLO_BT_SERVICE_EVENT_ENABLED_SUCCESS )
    {
        result = WICED_SUCCESS;
    }
    else
    {
        result = WICED_ERROR;
    }

 _exit:
    return result;
}


wiced_result_t apollo_bt_service_deinit( void )
{
    wiced_result_t result = WICED_ERROR;

    wiced_action_jump_when_not_true( g_p_bt_service_ctx != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: not yet initialized !\n") );

    if ( g_p_bt_service_ctx->timer_thread_ptr != NULL )
    {
        g_p_bt_service_ctx->timer_quit = WICED_TRUE;
        wiced_rtos_set_event_flags( &g_bt_service_ctx.timer_events, APOLLO_BT_SERVICE_TIMER_EVENT_QUIT );
        wiced_rtos_thread_force_awake( &g_p_bt_service_ctx->timer_thread );
        wiced_rtos_thread_join( &g_p_bt_service_ctx->timer_thread );
        wiced_rtos_delete_thread( &g_p_bt_service_ctx->timer_thread );
        g_p_bt_service_ctx->timer_thread_ptr = NULL;
    }

    result = wiced_bt_stack_deinit();

    apollo_bt_nv_deinit();

    g_p_bt_service_ctx = NULL;

    wiced_rtos_deinit_event_flags( &g_bt_service_ctx.events );
    wiced_rtos_deinit_event_flags( &g_bt_service_ctx.timer_events );
    memset( &g_bt_service_ctx, 0, sizeof(g_bt_service_ctx) );

 _exit:
    return result;
}


wiced_bt_management_evt_data_t *apollo_bt_service_get_management_evt_data(void)
{
    wiced_bt_management_evt_data_t *p_evt_data = NULL;

    if ( g_p_bt_service_ctx != NULL )
    {
        p_evt_data = &g_bt_service_ctx.event_data;
    }

    return p_evt_data;
}


wiced_result_t apollo_bt_service_reconnection_timer_start(void)
{
    wiced_result_t result = WICED_ERROR;

    wiced_action_jump_when_not_true( g_p_bt_service_ctx != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: not yet initialized !\n") );

    g_p_bt_service_ctx->timer_stop  = WICED_FALSE;
    g_p_bt_service_ctx->num_retries = 0;
    result =  wiced_rtos_set_event_flags( &g_bt_service_ctx.timer_events, APOLLO_BT_SERVICE_TIMER_EVENT_RUN );

 _exit:
    return result;
}


wiced_result_t apollo_bt_service_reconnection_timer_stop(void)
{
    wiced_result_t result = WICED_ERROR;

    wiced_action_jump_when_not_true( g_p_bt_service_ctx != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT service: not yet initialized !\n") );

    g_p_bt_service_ctx->timer_stop  = WICED_TRUE;

 _exit:
    return result;
}
