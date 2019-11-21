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

/** @file Apollo OTA2 support functions
 *
 */

#if defined(OTA2_SUPPORT)

#include <ctype.h>
#include "wiced.h"
#include "wiced_log.h"
#include "apollo_config.h"
#include "apollo_context.h"

#include "apollo_ota2_support.h"

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

wiced_result_t apollo_ota2_restore_settings_after_update( apollo_dct_collection_t* dct_tables, ota2_boot_type_t boot_type, wiced_bool_t* save_dct_tables );

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t apollo_is_service_active(apollo_app_t* apollo)
{
    if ( (apollo->player_handle != NULL) || (apollo->streamer_handle != NULL))
    {
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

wiced_result_t apollo_ota2_callback(void* session_id, wiced_ota2_service_status_t status, uint32_t value, void* opaque )
{
    apollo_app_t*    apollo = (apollo_app_t*)opaque;
    UNUSED_PARAMETER(session_id);
    UNUSED_PARAMETER(apollo);
    switch( status )
         {
    case OTA2_SERVICE_STARTED:      /* Background service has started
                                     * return - None
                                     */
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : SERVER STARTED -----------------------------\r\n");
        break;
    case OTA2_SERVICE_AP_CONNECT_ERROR:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : AP CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");

        /* allow the RMC network timer */
        apollo->network_timer_ignore = WICED_FALSE;
        /* Bring RMC network back up */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
        break;

    case OTA2_SERVICE_SERVER_CONNECT_ERROR:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : SERVER_CONNECT_ERROR -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");

        /* allow the RMC network timer */
        apollo->network_timer_ignore = WICED_FALSE;
        /* Bring RMC network back up */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
        break;

    case OTA2_SERVICE_AP_CONNECTED:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : AP_CONNECTED -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_SERVER_CONNECTED:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : SERVER_CONNECTED -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");
        break;

    case OTA2_SERVICE_CHECK_FOR_UPDATE: /* Time to check for updates.
                                         * return - WICED_SUCCESS = Service will check for update availability
                                         *        - WICED_ERROR   = Application will check for update availability   */
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : CHECK_FOR_UPDATE -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS, let Service do the checking.\r\n");

        /* Stopping playback  */
        if ((apollo_is_service_active(apollo) == WICED_TRUE) &&
            (apollo->rmc_network_coming_up == WICED_FALSE) &&
            (apollo->dct_tables.dct_app->ota2_stop_playback_to_update == 0))
        {
            wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "apollo->dct_tables.dct_app->ota2_stop_playback_to_update == 0 (DO NOT stop playback for updates).\r\n");
            /* product may not want to stop playback to get download - skip it */
            return WICED_ERROR; /* Do not stop playback to download an update */
        }
        /* Do not update while bringing up Apollo network */
        if (apollo->rmc_network_coming_up == WICED_TRUE)
        {
            wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "Do not bring down RMC network when RMC is coming up.\r\n");
            return WICED_ERROR;
        }

        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "Bring down RMC network. (stops playback).\r\n");
        /* Ignore the RMC network timer */
        apollo->network_timer_ignore = WICED_TRUE;
        /* set a flag to stop playback and stop RMC network */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RMC_DOWN);
        /* return success so service will load the update */
        break;

    case OTA2_SERVICE_UPDATE_AVAILABLE: /* Service has contacted server, update is available
                                         * value   - pointer to the wiced_ota2_image_header_t structure
                                         *           from the file on the server.
                                         *
                                         * return - WICED_SUCCESS = Application indicating that it wants the
                                         *                           OTA Service to perform the download
                                         *        - WICED_ERROR   = Application indicating that it will perform
                                         *                           the download, the OTA Service will do nothing.
                                         */
         {
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */
        wiced_ota2_image_header_t* ota2_header;

        ota2_header = (wiced_ota2_image_header_t*)value;

        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : UPDATE_AVAILABLE -----------------------------\r\n");

        /*
         * In an actual application, the application would look at the headers information and decide if the
         * file on the update server is a newer version that the currently running application.
         *
         * If the application wants the update to continue, it would return WICED_SUCCESS here
         * If not, return WICED_ERROR
         */
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "Current Version %d.%d\r\n", apollo->dct_tables.dct_app->ota2_major_version,
                                                                      apollo->dct_tables.dct_app->ota2_minor_version);
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "   OTA2 Version %d.%d\r\n", ota2_header->major_version, ota2_header->minor_version);

#if defined(CHECK_OTA2_UPDATE_VERSION)
        if ((apollo->dct_tables.dct_app->ota2_major_version > ota2_header->major_version) ||
            ((apollo->dct_tables.dct_app->ota2_major_version == ota2_header->major_version) &&
             (apollo->dct_tables.dct_app->ota2_minor_version >= ota2_header->minor_version)) )
        {
            wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "OTA2 Update Version Fail - return ERROR, do not update!\r\n");
            return WICED_ERROR;
        }
#endif

        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS, let Service perform the download.\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_NOTICE, "starting download:");

        apollo->ota2_info.ota2_download_percentage            = 0;
        break;
    }

    case OTA2_SERVICE_DOWNLOAD_STATUS:  /* Download status - value has % complete (0-100)
                                         *   NOTE: This will only occur when Service is performing download
                                         *         You will get a call after each packet (not based on percentage).
                                         * return - WICED_SUCCESS = Service will continue download
                                         *        - WICED_ERROR   = Service will STOP download and service will
                                         *                          issue OTA2_SERVICE_TIME_TO_UPDATE_ERROR           */
        wiced_log_msg(WLF_OTA2, WICED_LOG_NOTICE, "OTA2_SERVICE_DOWNLOAD_STATUS %ld %%!\r\n", value);
        if (apollo->ota2_info.ota2_download_percentage != value)
        {
            apollo->ota2_info.ota2_download_percentage = value;
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_OTA2_UPDATE);
        }
        break;

    case OTA2_SERVICE_PERFORM_UPDATE:   /* Download is complete
                                         * return - WICED_SUCCESS = Service will inform Bootloader to extract
                                         *                          and update on next power cycle
                                         *        - WICED_ERROR   = Service will inform Bootloader that download
                                         *                          is complete - Bootloader will NOT extract        */
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : PERFORM_UPDATE -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS, let Service extract update on next reboot.\r\n");
        break;

    case OTA2_SERVICE_UPDATE_ERROR:     /* There was an error in transmission
                                         * This will only occur if Error during Service performing data transfer
                                         * return - WICED_SUCCESS = Service will retry immediately
                                         *        - WICED_ERROR   = Service will retry on next check_interval
                                         *            Application can call
                                         *            wiced_ota2_service_check_for_updates()
                                         *            to run another check earlier than next timed update            */
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : UPDATE_ERROR -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS, let Service use retry check interval.\r\n");

        /* allow the RMC network timer */
        apollo->network_timer_ignore = WICED_FALSE;
        /* Bring RMC network back up */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
        return WICED_ERROR;
        break;

    case OTA2_SERVICE_UPDATE_ENDED:     /* All update actions for this check are complete.
                                         * value - WICED_SUCCESS - Service completed download successfully
                                         *       - WICED_ERROR   - Service failed OR Application stopped service from completing
                                         *                         download - in this case, the application must decide if
                                         *                         the download was successful
                                         * This callback is to allow the application to take any actions when
                                         *      the service is done checking / downloading an update
                                         *      It is called for all outcomes: successful or error.
                                         * return - None  - informational
                                         */
        wiced_log_msg(WLF_OTA2, WICED_LOG_NOTICE, "\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : UPDATE_ENDED value: %ld-----------------------------\r\n", value);
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");

        if (value == WICED_SUCCESS)
       {
           if (apollo->dct_tables.dct_app->ota2_reboot_after_download != 0)
           {
               wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, " Download is complete - reboot! \r\n");
               wiced_framework_reboot();
           }
       }
        /* allow the RMC network timer */
        apollo->network_timer_ignore = WICED_FALSE;
        /* Bring RMC network back up */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
        break;

    case OTA2_SERVICE_STOPPED:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "----------------------------- OTA2 Service Called : SERVICE STOPPED -----------------------------\r\n");
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "        return SUCCESS (not used by service). This is informational \r\n");

        /* allow the RMC network timer */
        apollo->network_timer_ignore = WICED_FALSE;
        /* Bring RMC network back up */
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
        break;

    default:
        wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "apollo_ota2_callback() UNKNOWN STATUS %d!\r\n", status);
        break;
    }

    return WICED_SUCCESS;
}

wiced_result_t apollo_ota2_start_bg_update(apollo_app_t* apollo)
{
    wiced_result_t result = WICED_ERROR;
    apollo_ota2_service_info_t* ota2_info;

    if (apollo == NULL)
    {
        return WICED_BADARG;
    }

    ota2_info = &apollo->ota2_info;

    /* get the image from the server & save in staging area in the backgound */
    wiced_ota2_service_uri_split(ota2_info->update_uri, ota2_info->host_name, sizeof(ota2_info->host_name),
            ota2_info->file_path, sizeof(ota2_info->file_path), &ota2_info->port);

    ota2_info->bg_params.host_name = ota2_info->host_name;
    ota2_info->bg_params.file_path = ota2_info->file_path;
    ota2_info->bg_params.port      = ota2_info->port;
    ota2_info->bg_params.ota2_interface = WICED_STA_INTERFACE;

    wiced_log_printf("ota2_update_uri:%s\r\n", ota2_info->update_uri);
    wiced_log_printf("ota2_host_name :%s \r\n", ota2_info->bg_params.host_name);
    wiced_log_printf("ota2_file_path :%s \r\n", ota2_info->bg_params.file_path);
    wiced_log_printf("ota2_port      :%d \r\n", ota2_info->bg_params.port);

    ota2_info->bg_params.auto_update               = 1;       /* auto update on reboot (callback overrides)  */
    ota2_info->bg_params.retry_check_interval      = SECONDS_PER_MINUTE;     /* minimum retry is one minute  */
    ota2_info->bg_params.max_retries               = 3;       /* maximum retries per update attempt         */
    ota2_info->bg_params.default_ap_info           = NULL;    /* don't restart AP using wiced OTA2 routines  */
    ota2_info->bg_params.ota2_ap_info              = NULL;    /* use the DCT AP list                         */

    if (ota2_info->bg_service == NULL)
    {
        /* start a new instance of the ota2 bg service */
        ota2_info->bg_service = wiced_ota2_service_init(&ota2_info->bg_params, apollo);
        if (ota2_info->bg_service == NULL)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_start_bg_update() wiced_ota2_service_init() failed! \r\n");
        }
    }
    else
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(ota2_info->bg_service, apollo_ota2_callback);
        if (result != WICED_SUCCESS)
        {
                wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "apollo_start_bg_update() ota2_test_get_update register callback failed! %d \r\n", result);
                wiced_ota2_service_deinit(ota2_info->bg_service);
                ota2_info->bg_service = NULL;
        }
        else
        {
            result = wiced_ota2_service_check_for_updates(ota2_info->bg_service);
            wiced_log_msg(WLF_OTA2, WICED_LOG_INFO, "apollo_start_bg_update() already running - apollo_ota2_start_bg_update( %p ) result: %ld\r\n", ota2_info->bg_service, result);
        }
        return result;
    }

    /* start getting the update */
    if (ota2_info->bg_service != NULL)
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(ota2_info->bg_service, apollo_ota2_callback);
        if (result != WICED_SUCCESS)
        {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_start_bg_update() ota2_test_get_update register callback failed! %d \r\n", result);
                wiced_ota2_service_deinit(ota2_info->bg_service);
                ota2_info->bg_service = NULL;
        }
        else
        {
            /* NOTE: This is a non-blocking call (async) */
            result = wiced_ota2_service_start(ota2_info->bg_service);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_start_bg_update wiced_ota2_service_start() failed! %d \r\n", result);
            }
        }
    }
    return result;
}

wiced_result_t apollo_ota2_network_is_down(apollo_app_t* apollo)
{
    apollo_ota2_service_info_t* ota2_info;

    if (apollo == NULL)
    {
        return WICED_BADARG;
    }

    ota2_info = &apollo->ota2_info;

    if (ota2_info->bg_service == NULL)
    {
        return WICED_SUCCESS;
    }
    return wiced_ota2_service_app_network_is_down(ota2_info->bg_service);

}

wiced_result_t apollo_ota2_stop_bg_update(apollo_app_t* apollo)
{
    wiced_result_t                          result;
    apollo_ota2_service_info_t              *ota2_info;

    if (apollo == NULL)
    {
        return WICED_BADARG;
    }

    ota2_info = &apollo->ota2_info;

    result = wiced_ota2_service_deinit(ota2_info->bg_service);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_ota2_service_deinit() returned:%d\r\n", result);
    }
    ota2_info->bg_service = NULL;

    return result;
}

wiced_result_t apollo_ota2_check_boot_type_and_restore_DCT(apollo_dct_collection_t* dct_tables, wiced_bool_t* save_dct_tables)
{
    wiced_result_t      result = WICED_SUCCESS;
    ota2_boot_type_t    boot_type;
    WICED_LOG_LEVEL_T   log_level;

    if ((dct_tables == NULL) || (save_dct_tables == NULL))
    {
        return WICED_BADARG;
    }

    /* set the log level so we output the boot type to the console */
    log_level = wiced_log_get_facility_level(WLF_AUDIO);
    wiced_log_set_facility_level(WLF_AUDIO, WICED_LOG_NOTICE);

    /* determine if this is a first boot, factory reset, or after an update boot */
    boot_type = wiced_ota2_get_boot_type();
    switch( boot_type )
    {
        case OTA2_BOOT_FAILSAFE_FACTORY_RESET:
        case OTA2_BOOT_FAILSAFE_UPDATE:
        default:
            /* We should never get here! */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Unexpected boot_type %d!\r\n", boot_type);
            /* FALL THROUGH */
        case OTA2_BOOT_NEVER_RUN_BEFORE:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "First boot EVER\r\n");
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_NORMAL:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Normal boot.\r\n");
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Factory Reset Occurred!\r\n");
            apollo_ota2_restore_settings_after_update(dct_tables, boot_type, save_dct_tables);
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_SOFTAP_UPDATE:
        case OTA2_BOOT_UPDATE:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Update Occurred!\r\n");
            apollo_ota2_restore_settings_after_update(dct_tables, boot_type, save_dct_tables);
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_LAST_KNOWN_GOOD:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Last Known Good used!\r\n");
            break;
    }

    /* restore the log level */
    wiced_log_set_facility_level(WLF_AUDIO, log_level);

    return result;
}

wiced_result_t apollo_ota2_restore_settings_after_update( apollo_dct_collection_t* dct_tables, ota2_boot_type_t boot_type, wiced_bool_t* save_dct_tables )
{
    platform_dct_network_config_t*   dct_network;
    platform_dct_wifi_config_t*      dct_wifi;
    apollo_dct_t*                    dct_app;
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    platform_dct_bt_config_t*        dct_bt;
#endif

    /* sanity check */
    if (save_dct_tables == NULL)
    {
        return WICED_BADARG;
    }


    /* read in our configurations from the DCT Save Area and copy to RAM dct_tables */

    /* App */
    if (dct_tables->dct_app != NULL)
    {
        dct_app = (apollo_dct_t *)malloc(sizeof(apollo_dct_t));
        if (dct_app != NULL)
        {
            if (wiced_dct_ota2_read_saved_copy( dct_app, DCT_APP_SECTION, 0, sizeof(apollo_dct_t)) == WICED_SUCCESS)
            {
                uint16_t major, minor;

                /* start with basic app DCT data */
                memcpy(dct_tables->dct_app, dct_app, sizeof(apollo_dct_t));

                /* update the Software version number */
                switch (boot_type)
                {
                    case OTA2_BOOT_FAILSAFE_FACTORY_RESET:
                    case OTA2_BOOT_FAILSAFE_UPDATE:
                    case OTA2_BOOT_NEVER_RUN_BEFORE:
                    case OTA2_BOOT_NORMAL:
                    case OTA2_BOOT_LAST_KNOWN_GOOD: /* unsupported */
                    default:
                        break;
                    case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
                    case OTA2_BOOT_FACTORY_RESET:
                        if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, &major, &minor) == WICED_SUCCESS)
                        {
                            dct_tables->dct_app->ota2_major_version = major;
                            dct_tables->dct_app->ota2_minor_version = minor;
                        }
                        break;
                    case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
                    case OTA2_BOOT_SOFTAP_UPDATE:
                    case OTA2_BOOT_UPDATE:
                        if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_STAGED, &major, &minor) == WICED_SUCCESS)
                        {
                            dct_tables->dct_app->ota2_major_version = major;
                            dct_tables->dct_app->ota2_minor_version = minor;
                        }
                        break;
                }
                *save_dct_tables = WICED_TRUE;
            }
            free(dct_app);
            dct_app = NULL;
        }
    }

    /* network */
    if (dct_tables->dct_network != NULL)
    {
        dct_network = (platform_dct_network_config_t *)malloc(sizeof(platform_dct_network_config_t));
        if (dct_network != NULL)
        {
            if (wiced_dct_ota2_read_saved_copy( dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) == WICED_SUCCESS)
            {
                memcpy(dct_tables->dct_network, dct_network, sizeof(platform_dct_network_config_t));
                *save_dct_tables = WICED_TRUE;
            }
            free(dct_network);
            dct_network = NULL;
        }
    }

    /* wifi */
    if (dct_tables->dct_wifi != NULL)
    {
        dct_wifi = (platform_dct_wifi_config_t *)malloc(sizeof(platform_dct_wifi_config_t));
        if (dct_wifi != NULL)
        {
            if (wiced_dct_ota2_read_saved_copy( dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) == WICED_SUCCESS)
            {
                memcpy(dct_tables->dct_wifi, dct_wifi, sizeof(platform_dct_wifi_config_t));
                *save_dct_tables = WICED_TRUE;
            }
            free(dct_wifi);
            dct_wifi = NULL;
        }
    }

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Bluetooth */
    if (dct_tables->dct_bt != NULL)
    {
        dct_bt = (platform_dct_bt_config_t *)malloc(sizeof(platform_dct_bt_config_t));
        if (dct_bt != NULL)
        {
            if (wiced_dct_ota2_read_saved_copy( dct_bt, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t)) == WICED_SUCCESS)
            {
                memcpy(dct_tables->dct_bt, dct_bt, sizeof(platform_dct_bt_config_t));
                *save_dct_tables = WICED_TRUE;
            }
            free(dct_bt);
            dct_bt = NULL;
        }
    }
#endif
    return WICED_SUCCESS;
}

wiced_result_t apollo_ota2_status(apollo_app_t* apollo)
{
    wiced_ota2_image_status_t status;
    apollo_ota2_service_info_t* ota2_info;

    if (apollo == NULL)
    {
        return WICED_BADARG;
    }

    ota2_info = &apollo->ota2_info;

    if (ota2_info->bg_service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "OTA2 Service NOT initialized.\r\n");
    }
    else
    {
        wiced_log_printf("OTA2 Service info:\r\n");
        wiced_log_printf("                update_uri: %s\r\n", (strlen(ota2_info->update_uri) > 0) ? ota2_info->update_uri : "NO URI SET");
        wiced_log_printf("                 cb_opaque: %p\r\n", ota2_info->cb_opaque);
        wiced_log_printf("              ota2_ap_info: %p\r\n", ota2_info->bg_params.ota2_ap_info);
        if (ota2_info->bg_params.ota2_ap_info != NULL)
        {
            wiced_log_printf("              SSID: %.*s\r\n", ota2_info->bg_params.ota2_ap_info->details.SSID.length,
                                                             ota2_info->bg_params.ota2_ap_info->details.SSID.value);
        }
        wiced_log_printf("        ota2_ap_list_count: %ld\r\n", ota2_info->bg_params.ota2_ap_list_count);
        wiced_log_printf("              ota2_ap_list: %p\r\n", ota2_info->bg_params.ota2_ap_list);
        if ( (ota2_info->bg_params.ota2_ap_list_count > 0 ) && (ota2_info->bg_params.ota2_ap_list !=  NULL))
        {
            int i;
            for (i = 0; i < ota2_info->bg_params.ota2_ap_list_count; i++)
            {
                if (ota2_info->bg_params.ota2_ap_list[i].details.SSID.length > 0)
                {
                    wiced_log_printf("                   [%d]  SSID: %.*s\r\n", i, ota2_info->bg_params.ota2_ap_list[i].details.SSID.length,
                                                                                   ota2_info->bg_params.ota2_ap_list[i].details.SSID.value);
                }
            }
        }
        wiced_log_printf("           default_ap_info: %ld\r\n", ota2_info->bg_params.default_ap_info);
        if (ota2_info->bg_params.default_ap_info != NULL)
        {
            wiced_log_printf("              SSID: %.*s\r\n", ota2_info->bg_params.default_ap_info->details.SSID.length,
                                                             ota2_info->bg_params.default_ap_info->details.SSID.value);
        }

        wiced_ota2_service_status(ota2_info->bg_service);
    }

    if (wiced_ota2_image_get_status(WICED_OTA2_IMAGE_TYPE_STAGED, &status) == WICED_SUCCESS)
    {
        /* Staging Area status */
        wiced_log_printf("OTA2 Staged Area Status:\n");
        switch(status)
        {
        case WICED_OTA2_IMAGE_INVALID:
        default:
            wiced_log_printf("    Invalid\n");
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS:
            wiced_log_printf("    In Progress\n");
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_FAILED:
            wiced_log_printf("    Failed\n");
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_UNSUPPORTED:
            wiced_log_printf("    Unsupported\n");
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE:
            wiced_log_printf("    Complete\n");
            break;
        case WICED_OTA2_IMAGE_VALID:
            wiced_log_printf("    Valid\n");
            break;
        case WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT:
            wiced_log_printf("    Extract on Boot\n");
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED:
            wiced_log_printf("    Extracted\n");
            break;
        }
    }

    return WICED_SUCCESS;
}

wiced_bool_t apollo_ota2_support_check_for_one_button_update( void )
{
    uint32_t press_time;

    press_time = wiced_waf_get_button_press_time( PLATFORM_BUTTON_MULTI_FUNC, PLATFORM_GREEN_LED_INDEX, OTA2_HOLD_TIME_FOR_ONE_BUTTON_UPDATE);

    platform_led_set_state(PLATFORM_GREEN_LED_INDEX, WICED_LED_ON );

    if ( press_time >= OTA2_HOLD_TIME_FOR_ONE_BUTTON_UPDATE)
    {
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

#endif  /* defined(OTA2_SUPPORT) */
