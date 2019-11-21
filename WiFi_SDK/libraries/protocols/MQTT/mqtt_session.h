/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  MQTT session APIs and types.
 *
 *  Internal types not to be included directly by applications.
 */
#pragma once

#include "mqtt_api.h"
#include "mqtt_frame.h"
#include "linked_list.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define     SESSION_ITEMS_SIZE      (WICED_MQTT_QUEUE_SIZE * 2)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *              MQTT session type Definitions
 ******************************************************/
typedef union wiced_mqtt_session_item_agrs_u
{
    mqtt_publish_arg_t     publish;
    mqtt_subscribe_arg_t   subscribe;
    mqtt_unsubscribe_arg_t unsubscribe;
    mqtt_pubrec_arg_t      pubrec;
    mqtt_pubrel_arg_t      pubrel;
}wiced_mqtt_session_item_args_t;

typedef struct mqtt_session_item_s
{
    struct list_head                list;
    mqtt_frame_type_t         type;
    wiced_mqtt_session_item_args_t  args;
}wiced_mqtt_session_item_t;

typedef struct mqtt_session_s
{
    struct list_head used_list;
    struct list_head nonused_list;
    wiced_mqtt_session_item_t items[SESSION_ITEMS_SIZE];
}mqtt_session_t;
/******************************************************
 *             Content Frame Type Definitions
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
wiced_result_t mqtt_session_init                 (                                                                      mqtt_session_t *session );
wiced_result_t mqtt_session_add_item             ( mqtt_frame_type_t type, void *args                           , mqtt_session_t *session );
wiced_result_t mqtt_session_remove_item          ( mqtt_frame_type_t type, uint16_t packet_id                   , mqtt_session_t *session );
wiced_result_t mqtt_session_item_exist           ( mqtt_frame_type_t type, uint16_t packet_id                   , mqtt_session_t *session );
wiced_result_t mqtt_session_iterate_through_items( wiced_result_t (*iter_func)(mqtt_frame_type_t type, void *arg , void *p_user ), void* p_user, mqtt_session_t *session);

#ifdef __cplusplus
} /* extern "C" */
#endif
