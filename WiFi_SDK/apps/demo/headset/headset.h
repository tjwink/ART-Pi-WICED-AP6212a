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
#pragma once

#include "headset_button.h"
#include "headset_config.h"
#include "headset_upnpavrender.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define HEADSET_TRACE_IN             wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "%s In (%d)\n", __func__, __LINE__);
#define HEADSET_TRACE_OUT            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "%s Out (%d)\n", __func__, __LINE__);
#define HEADSET_CHECK_RESULT(expr)   result = (wiced_result_t)(expr);\
                                     if (result != WICED_SUCCESS)\
                                     {wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error@%s(%d)\n", __func__, __LINE__);\
                                      goto _exit;\
                                     }\

/* LEDs */
#define HEADSET_LED_INIT             wiced_gpio_init(WICED_GPIO_16, OUTPUT_PUSH_PULL);\
                                     wiced_gpio_init(WICED_GPIO_17, OUTPUT_PUSH_PULL);
#define HEADSET_GREEN_LED_ON         wiced_gpio_output_low(WICED_GPIO_16);
#define HEADSET_GREEN_LED_OFF        wiced_gpio_output_high(WICED_GPIO_16);
#define HEADSET_RED_LED_ON           wiced_gpio_output_low(WICED_GPIO_17);
#define HEADSET_RED_LED_OFF          wiced_gpio_output_high(WICED_GPIO_17);

#define HEADSET_LED_WLAN_CONNECTING  HEADSET_RED_LED_ON\
                                     HEADSET_GREEN_LED_OFF
#define HEADSET_LED_WLAN_CONNECTED   HEADSET_GREEN_LED_ON\
                                     HEADSET_RED_LED_OFF
#define HEADSET_LED_WLAN_SWITCHING   HEADSET_GREEN_LED_ON\
                                     HEADSET_RED_LED_ON
#define HEADSET_LED_OFF              HEADSET_GREEN_LED_OFF\
                                     HEADSET_RED_LED_OFF

/******************************************************
 *                    Constants
 ******************************************************/

/* Our table is based on the levels of priority. with the current design, Multiple services can have same priority.
 * This macro defines the total number of priority levels we have and not the total number of services.
 * Though, it is quite possible that total number of services is same as priority levels.
 */
#define WICED_MAXIMUM_PRIORITY_LEVELS              5
#define WICED_APP_REBOOT_DELAY_MSECS               (500)

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum {
    HEADSET_ALL_EVENTS                  = -1,

    /* WLAN EVENTs */
    HEADSET_EVENT_WLAN_CONNECTED        = (1 << 0),
    HEADSET_EVENT_WLAN_DISCONNECTED     = (1 << 1),
    HEADSET_EVENT_WLAN_FAILED           = (1 << 2),
    HEADSET_EVENT_WLAN_RECONNECT_REQ    = (1 << 3),

    /* BUTTON EVENTs */
    HEADSET_EVENT_BUTTON_VOL_UP         = (1 << 4),
    HEADSET_EVENT_BUTTON_VOL_DOWN       = (1 << 5),
    HEADSET_EVENT_BUTTON_PLAY_PAUSE     = (1 << 6),
    HEADSET_EVENT_BUTTON_BACKWARD       = (1 << 7),
    HEADSET_EVENT_BUTTON_FORWARD        = (1 << 8),
    HEADSET_EVENT_BUTTON_MULTI_FUNC     = (1 << 9),

    /* TODO: BT EVENTs */
    HEADSET_EVENT_BT_CALL               = (1 << 10),

    /* TODO: UPnP EVENTs */
} HEADSET_EVENTS_T;

/**
 * Service type
 * The service type should always be 1 left shifted by a predefined number, as the
 * service type also helps decide pre-emption policies.
 */
typedef enum {
    SERVICE_NONE        =   (0),
    SERVICE_AIRPLAY     =   (1),         //!< SERVICE_AIRPLAY
    SERVICE_WLAN        =   (2),         //!< SERVICE_WLAN
    SERVICE_BT_A2DP     =   (3),         //!< SERVICE_BT_A2DP
    SERVICE_BT_HFP      =   (4),         //!< SERVICE_BT_HFP
} service_type_t;

typedef enum {
    SERVICE_GROUP_BLUETOOTH = 0,
    SERVICE_GROUP_AIRPLAY,
} service_group_type_t;

/**
 * The states of a service are defined here.
 */
typedef enum {
    /* Service has been disabled or disactivated either by application[user] or got some events
     * from remote device which made the service disabled or It has not been started at all */
    SERVICE_DISABLED = 0,
    /* Service is idle when it is connected but not consuming any system resources, same as ENABLED */
    SERVICE_IDLE,//!< SERVICE_IDLE
    /* same as IDLE. Service is enabled but not using any critical system resource */
    SERVICE_ENABLED = SERVICE_IDLE,
    /* Service is pending getting enabled due to a resource lock elsewhere. */
    SERVICE_PENDING,
    /* Service is stopping. */
    SERVICE_STOPPING,
    /* Service is playing; owner of audio resources */
    SERVICE_PLAYING_AUDIO, //!< SERVICE_PLAYING
    /* Service has been paused, may or may not release audio resources */
    SERVICE_PAUSED_AUDIO,  //!< SERVICE_PAUSED
    /* Service has been stopped(playing), may or may not release audio resources */
    SERVICE_STOPPED_AUDIO, //!< SERVICE_STOPPED

    /* Service has been 'prevented' by a higher-priority service. Future requests from
     * this service to get audio resources must be rejected by the library until application
     * 'allow' it back
     */
    SERVICE_PREVENTED,//!< SERVICE_PREVENTED

    /* Service has been preempted, i.e. the service was active when it got 'prevented' by
     * a higher priority service */
    SERVICE_PREEMPTED,//!< SERVICE_PREEMPTED

} service_state_t;

/**
 * Service priorities are defined here.
 */
typedef enum {
    SERVICE_DAEMON_PRIORITY = 0,                //!< SERVICE_DAEMON_PRIORITY
    SERVICE_BT_A2DP_PRIORITY,                   //!< SERVICE_BT_A2DP_PRIORITY
    SERVICE_AIRPLAY_PRIORITY,                   //!< SERVICE_AIRPLAY_PRIORITY
    SERVICE_WLAN_PRIORITY,                      //!< SERVICE_WLAN_PRIORITY
    SERVICE_BT_HFP_PRIORITY,                    //!< SERVICE_BT_HFP_PRIORITY
} service_priority_t;

/**
 * Specific service events.
 * For new services, append to the enum.
 */
typedef enum {
    /* BT */
    WICED_BT_RECONNECT_ON_LINK_LOSS = (1<<0),
    WICED_BT_CONNECT_A2DP           = (1<<1),
    WICED_BT_DISCONNECT_A2DP        = (1<<2),
    WICED_BT_CONNECT_AVRCP          = (1<<3),
    WICED_BT_DISCONNECT_AVRCP       = (1<<4),
    WICED_BT_CONNECT_HFP            = (1<<5),
    WICED_BT_DISCONNECT_HFP         = (1<<6),
    WICED_BT_AVRC_FF                = (1<<7),
    WICED_BT_AVRC_REW               = (1<<8),
    /* WLAN */
    WICED_WLAN_INIT                 = (1<<9),
    WICED_WLAN_DEINIT               = (1<<10),
    WICED_WLAN_CONNECT              = (1<<11),
    WICED_WLAN_DISCONNECT           = (1<<12),
    WICED_WLAN_GM_START             = (1<<13),
    WICED_WLAN_GM_STOP              = (1<<14),
    WICED_WLAN_P2P_ASYNC_RESULT     = (1<<15),
} app_event_t ;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**
 * API Typedefs.
 */
typedef struct wiced_app_service wiced_app_service_t;

typedef wiced_result_t (*WICED_APP_INIT_SERVICE)(wiced_app_service_t *service);
typedef wiced_result_t (*WICED_APP_DEINIT_SERVICE)(wiced_app_service_t *service);
typedef wiced_result_t (*WICED_APP_CONNECT_SERVICE)(wiced_app_service_t *service);
typedef wiced_result_t (*WICED_APP_DISCONNECT_SERVICE)(wiced_app_service_t *service);

typedef wiced_result_t (*WICED_APP_PREVENT_SERVICE)(wiced_app_service_t *service);
typedef wiced_result_t (*WICED_APP_ALLOW_SERVICE)(wiced_app_service_t *service);
typedef wiced_result_t (*WICED_APP_FORCE_PLAYBACK)(void);
typedef wiced_result_t (*WICED_APP_FORCE_STOP_PLAYBACK)(void);

/******************************************************
 *                    Structures
 ******************************************************/
/**
 * Headset main context
 */
typedef struct
{
    /* Worker Thread */
    wiced_mutex_t                      lock;
    wiced_thread_t                     thread_handle;
    wiced_queue_t                      queue;
    wiced_semaphore_t                  queue_semaphore;
    uint32_t                           event_mask;
    uint32_t                           event_wait;
    wiced_result_t                     event_result;

    /* Service State */
    wiced_app_service_t*               current_service;
    wiced_bool_t                       play_button_state;
    wiced_bool_t                       a2dp_media_state;

    /* WLAN interface from a DCT */
    wiced_interface_t                  wlan_interface;

    /* DCTs */
    headset_dct_collection_t           dct_tables;

    /* UPnP AV renderer */
    headset_upnpavrender_context_t*    upnpavrender_context;

    /* WLAN powersave mode */
    uint8_t                            wlan_powersave_mode;

    /* MCU sleep mode */
    uint8_t                            mcu_sleep_mode;
} headset_context_t;

/**
 * Service Struct: Definition
 */
struct wiced_app_service
{
    /* Priority of the service */
    int                             priority;
    /* Index */
    int                             type;

    /* State of the service */
    volatile service_state_t        state;


    /* Function pointers for controlling the state of the service */

    /* initialize service function pointer. Typically, will invoke library-specific initialization routines */
    WICED_APP_INIT_SERVICE          init_service;
    /* connect service */
    WICED_APP_CONNECT_SERVICE       connect_service;
    /* dusconncet service */
    WICED_APP_DISCONNECT_SERVICE    disconnect_service;
    /* deinitialize service */
    WICED_APP_DEINIT_SERVICE        deinit_service;

    /* Flag the library to prevent this service from being activated until allow_service is called;
     * Underlying libraries should ensure that if 'prevent_service' has been called and the service
     * is currently active, then it must free up the system resources.
     */
    WICED_APP_PREVENT_SERVICE       prevent_service;

    /* Opposite of prevent_service */
    WICED_APP_ALLOW_SERVICE         allow_service;

     /* Feed Button-events and action to this handler */
    WICED_APP_BUTTON_HANDLER        button_handler;

    /* For service specific argument */
    void* arg1;
    void* arg2;
};

/**
 * Queue element for app framework queue.
 */
typedef struct
{
    app_event_t         event;
    wiced_result_t      (*function)(void* arg);
    void*               arg;
    wiced_bool_t        wait_for_event_complete;
    service_type_t      event_source;
} app_queue_element_t;

/**
 * Single cell of the app service table.
 */
typedef struct _app_service_cell
{
    wiced_app_service_t service;
    struct _app_service_cell *next;
} wiced_app_service_cell_t;

/******************************************************
 *                 Global Variables
 ******************************************************/
extern wiced_app_service_cell_t* wiced_service_array[];

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_app_service_t* get_app_current_service                   (void);
wiced_result_t       app_set_service_state                     (service_type_t service_type, service_state_t state);
service_state_t      app_get_service_state                     (service_type_t service_type);
wiced_result_t       app_disable_service                       (service_type_t service_type);
wiced_result_t       app_set_current_service                   (service_type_t service_type);
wiced_result_t       app_reset_current_service                 (service_type_t service_type);

void                 headset_worker_push_work                  (app_event_t event, wiced_result_t (*function)(void* arg), void* func_arg, wiced_bool_t wait);
void                 headset_worker_signal_for_event_semaphore (app_event_t event, wiced_result_t completed);
wiced_result_t       headset_worker_wait_for_event_semaphore   (app_event_t event);
wiced_bool_t         headset_worker_check_pending_event_semaphore ( app_event_t event );

#ifdef __cplusplus
} /* extern "C" */
#endif
