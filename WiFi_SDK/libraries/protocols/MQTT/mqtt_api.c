/*
 * $ Copyright Broadcom Corporation $
 */


#include "wiced.h"
#include "mqtt_api.h"
#include "mqtt_internal.h"


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
 *               extern Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static mqtt_session_t session;

/******************************************************
 *               Function Definitions
 ******************************************************/


wiced_result_t wiced_mqtt_init( wiced_mqtt_object_t mqtt_obj )
{
    wiced_result_t result = WICED_SUCCESS;

    /* Make sure the actual structure size and size defined in macro are not mismatching */
    if ( sizeof(mqtt_connection_t) > WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT )
    {
        return WICED_ERROR;
    }

    memset( mqtt_obj, 0, WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );

    result = mqtt_core_init( mqtt_obj );
    return result;
}

wiced_result_t wiced_mqtt_deinit( wiced_mqtt_object_t mqtt_obj )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    wiced_result_t result = WICED_SUCCESS;

    result = mqtt_core_deinit( conn );
    return result;
}

wiced_result_t wiced_mqtt_connect( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security, wiced_mqtt_pkt_connect_t *conninfo )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    wiced_result_t result = WICED_SUCCESS;
    mqtt_connect_arg_t args;

    wiced_assert( "[MQTT LIB] : mqtt object cannot be NULL\r\n", mqtt_obj != NULL );
    wiced_assert( "[MQTT LIB] : Address cannot be NULL\r\n", address != NULL );
    wiced_assert( "[MQTT LIB] : Callback function pointer cannot be NULL\r\n", callback != NULL );
    wiced_assert( "[MQTT LIB] : connection packet information cannot be NULL\r\n", conninfo != NULL );

    memset( &args, 0, sizeof(mqtt_connect_arg_t) );

    conn->peer_cn = conninfo->peer_cn;
    result = mqtt_connection_init( address, conninfo->port_number, interface, callback, conn, security );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("[MQTT LIB] : error intializing the  mqtt connection setup\n"));
        return result;
    }

    args.clean_session = conninfo->clean_session;
    args.will_flag = 0;
    if ( conninfo->username != NULL )
    {
        args.username_flag = 1;
        args.username.str = (uint8_t*) conninfo->username;
        args.username.len = (uint16_t) strlen( (char*) conninfo->username );
    }
    else
    {
        args.username_flag = 0;
    }

    if ( conninfo->password != NULL )
    {
        args.password_flag = 1;
        args.password.str = (uint8_t*) conninfo->password;
        args.password.len = (uint16_t) strlen( (char*) conninfo->password );
    }
    else
    {
        args.password_flag = 0;
    }
    args.client_id.str = (uint8_t*) conninfo->client_id;
    args.client_id.len = (uint16_t) strlen( (char*) conninfo->client_id );
    args.keep_alive = conninfo->keep_alive; /*in seconds */
    args.mqtt_version = conninfo->mqtt_version;

    return mqtt_connect( conn, &args, &session );
}

wiced_result_t wiced_mqtt_disconnect( wiced_mqtt_object_t mqtt_obj )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    return mqtt_disconnect( conn );
}

wiced_mqtt_msgid_t wiced_mqtt_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    mqtt_subscribe_arg_t args;

    args.topic_filter.str = (uint8_t*) topic;
    args.topic_filter.len = (uint16_t) strlen( topic );
    args.qos = qos;
    args.packet_id = ( ++conn->packet_id );
    if ( mqtt_subscribe( conn, &args ) != WICED_SUCCESS )
    {
        return 0;
    }
    return args.packet_id;
}

wiced_mqtt_msgid_t wiced_mqtt_unsubscribe( wiced_mqtt_object_t mqtt_obj, char *topic )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    mqtt_unsubscribe_arg_t args;

    args.topic_filter.str = (uint8_t*) topic;
    args.topic_filter.len = (uint16_t) ( strlen( topic ) );
    args.packet_id = ( ++conn->packet_id );
    if ( mqtt_unsubscribe( conn, &args ) != WICED_SUCCESS )
    {
        return 0;
    }
    return args.packet_id;
}

wiced_mqtt_msgid_t wiced_mqtt_publish( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t *data, uint32_t data_len, uint8_t qos )
{
    mqtt_publish_arg_t args = { 0 };
    mqtt_connection_t *conn = (mqtt_connection_t*) mqtt_obj;
    args.topic.str = (uint8_t*) topic;
    args.topic.len = (uint16_t) strlen( (char*) topic );
    args.data = (uint8_t*) data;
    args.data_len = data_len;
    args.total_length = data_len;
    args.dup = 0;
    args.qos = qos;
    args.retain = 0;
    args.packet_id = ( ++conn->packet_id );
    if ( mqtt_publish( conn, &args ) != WICED_SUCCESS )
    {
        return 0;
    }
    return args.packet_id;
}
