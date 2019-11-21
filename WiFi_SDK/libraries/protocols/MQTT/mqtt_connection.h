/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  MQTT internal APIs.
 *
 *  Internal, not to be used directly by applications.
 */
#pragma once

#include "wiced.h"
#include "mqtt_frame.h"
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define XSTR(S) #S
#define STR(S) XSTR(S)

/******************************************************
 *                    Constants
 ******************************************************/
#define MQTT_PROTOCOL_REPLY_SUCCESS     (200)

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *      Backend functions called from mqtt_queue
 ******************************************************/
/*
 * Connection and channel methods functions.
 *
 * Internal not to be used directly by user applications.
 */
wiced_result_t mqtt_backend_put_connect                     (       mqtt_connect_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_connack                     (       mqtt_connack_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_publish                     (       mqtt_publish_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_publish                     (       mqtt_publish_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_puback                      (       mqtt_puback_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_puback                      ( const mqtt_puback_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_subscribe                   (       mqtt_subscribe_arg_t   *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_suback                      (       wiced_mqtt_suback_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_unsubscribe                 (       mqtt_unsubscribe_arg_t *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_unsuback                    (       mqtt_unsuback_arg_t    *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_disconnect                  (                                           mqtt_connection_t *conn );
wiced_result_t mqtt_backend_connection_close                (                                           mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_pubrec                      ( const mqtt_pubrec_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_pubrec                      (       mqtt_pubrec_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_pubrel                      ( const mqtt_pubrel_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_pubrel                      (       mqtt_pubrel_arg_t      *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_pubcomp                     ( const mqtt_pubcomp_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_pubcomp                     (       mqtt_pubcomp_arg_t     *args, mqtt_connection_t *conn );
wiced_result_t mqtt_backend_put_pingreq                     (                                           mqtt_connection_t *conn );
wiced_result_t mqtt_backend_get_pingres                     (                                           mqtt_connection_t *conn );

#ifdef __cplusplus
} /* extern "C" */
#endif
