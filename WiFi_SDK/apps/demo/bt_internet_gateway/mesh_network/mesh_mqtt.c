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
#include "cloud_config.h"

#if (CLOUD_CONFIG != WICED_REST)
#include "mqtt_api.h"
#include "mesh.h"
#include "JSON.h"
#include "resources.h"
#include "wiced_hci_bt_mesh.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define MQTT_PUBLISH_RETRY_COUNT            (3)

#define MQTT_SUBSCRIBE_RETRY_COUNT          (3)

#define WICED_MQTT_TIMEOUT                  (5000)

#define WICED_MQTT_DELAY_IN_MILLISECONDS    (1000)

#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)

#define BIG_HTTP_SERVER_STACK_SIZE          ( 5000 )

#define MQTT_HOSTNAME_LOOKUP_RETRY          (3)

#define MQTT_EVENT_SUBSCRIBE                (2)

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
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event );
static wiced_result_t mqtt_wait_for( wiced_mqtt_event_type_t event, uint32_t timeout );
static wiced_result_t mqtt_conn_open( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security );
static wiced_result_t mqtt_app_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos );
wiced_result_t mqtt_app_publish( wiced_mqtt_object_t mqtt_obj, uint8_t qos, char *topic, uint8_t *data, uint32_t data_len );
static wiced_result_t parse_json_shadow_status(wiced_json_object_t * json_object );

void mesh_init_mqtt_transport( wiced_thread_arg_t arg );
extern int32_t process_send_mesh_pkt_rcvd( struct wiced_big_mqtt *big_mqtt , char *msg );

#if (CLOUD_CONFIG ==  WICED_AWS_BROKER)
wiced_result_t mesh_read_aws_credentials( void );
/******************************************************
 *               Variables Definitions
 ******************************************************/
static wiced_mqtt_security_t security;
#endif

static wiced_ip_address_t broker_address;
static wiced_mqtt_callback_t callbacks = mqtt_connection_event_cb;
static wiced_mqtt_event_type_t expected_event;
static wiced_semaphore_t semaphore;
static wiced_semaphore_t wake_semaphore;
char recieved_data[110]="ABCD";
uint32_t DISCONNECTED = WICED_FALSE;

int state = 0xff;
wiced_mqtt_topic_msg_t msg;
wiced_mqtt_object_t mqtt_object;
struct wiced_big_mqtt big_mqtt;

/******************************************************
 *               Function Definitions
 ******************************************************/

void mesh_init_mqtt_transport( wiced_thread_arg_t arg )
{

    int host_lookup_retry=0;
    int connection_retries = 0;
    int retries = 0;

    UNUSED_VARIABLE(arg);

    wiced_rtos_delay_milliseconds( 5000 );

    wiced_result_t ret = WICED_SUCCESS;
    // register a callback with JSON PARSER
    wiced_JSON_parser_register_callback( parse_json_shadow_status );

    /* Memory allocated for mqtt object*/
    mqtt_object = (wiced_mqtt_object_t) malloc( WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if ( mqtt_object == NULL )
    {
        WPRINT_APP_ERROR( ("Don't have memory to allocate for MQTT object...\n") );
        return;
    }

    WPRINT_APP_INFO( ( "Resolving IP address of MQTT broker...\n" ) );

    do
    {
        ret = wiced_hostname_lookup( MQTT_BROKER_ADDRESS, &broker_address, 10000, WICED_STA_INTERFACE );
        host_lookup_retry++;

    } while(( ret != WICED_SUCCESS ) && ( host_lookup_retry < MQTT_HOSTNAME_LOOKUP_RETRY ) );

    WPRINT_APP_INFO( ("Resolved Broker IP: %u.%u.%u.%u\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)) );

    if ( ret == WICED_ERROR || broker_address.ip.v4 == 0 )
    {
        WPRINT_APP_INFO( ("Error in resolving DNS\n") );
        return;
    }

    wiced_mqtt_init( mqtt_object );
    wiced_rtos_init_semaphore( &semaphore );
    wiced_rtos_init_semaphore( &wake_semaphore );    /* Initialize wake semaphore */

    do
    {
        retries = 0;
        connection_retries = 0;

        do
        {
            WPRINT_APP_INFO( ("[MQTT] Opening connection...\n") );

#if (CLOUD_CONFIG == WICED_AWS_BROKER)
            mesh_read_aws_credentials();
            WPRINT_APP_INFO( ("[MQTT] done reading creds...\n") );
            wiced_rtos_delay_milliseconds( 100 );
            ret = mqtt_conn_open( mqtt_object, &broker_address, WICED_STA_INTERFACE, callbacks, &security );
            WPRINT_APP_INFO( ("[MQTT] connection open called..\n") );

#elif (CLOUD_CONFIG == WICED_BLUEMIX_BROKER)
            ret = mqtt_conn_open( mqtt_object, &broker_address, WICED_STA_INTERFACE, callbacks, NULL );
#endif
            wiced_rtos_delay_milliseconds( 100 );
            connection_retries++ ;

        } while ( ( ret != WICED_SUCCESS ) && ( connection_retries < WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES ) );

        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO((" Failed\n"));
            return;
        }

        WPRINT_APP_INFO(("Connection open Success\n"));
        DISCONNECTED = WICED_FALSE;
        big_mqtt.mqtt_object = mqtt_object;
        big_mqtt.topic = NULL;
        big_mqtt.topic_len = 0;

        /*  Subscribing  */

        do
        {
            WPRINT_APP_INFO( ("[MQTT] Subscribing...WICED_TOPIC_SUBSCRIBE\n") );
            ret = mqtt_app_subscribe( mqtt_object, WICED_TOPIC_SUBSCRIBE, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE );
            retries ++;
        } while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_SUBSCRIBE_RETRY_COUNT ) );

        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO((" Failed\n"));
            return;
        }
        WPRINT_APP_INFO(("Success...\n"));
        wiced_rtos_delay_milliseconds( WICED_MQTT_DELAY_IN_MILLISECONDS );

        while ( WICED_TRUE )
        {
            wiced_rtos_get_semaphore( &wake_semaphore, WICED_NEVER_TIMEOUT );

            if ( DISCONNECTED == WICED_TRUE )
                break;
//            WPRINT_APP_INFO( ( "[MQTT] Received %.*s  for TOPIC : %.*s\n\n", (int) msg.data_len, msg.data, (int) msg.topic_len, msg.topic ) );

            if ( state == 0 )
            {
                mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE, WICED_TOPIC_PUBLISH, (uint8_t*) msg.data, (int) msg.data_len );
                char *mqtt_data = malloc( msg.data_len + 1 );
                memcpy( mqtt_data, msg.data, msg.data_len );
                mqtt_data[ msg.data_len + 1 ] = '\0';
                big_mqtt.topic = WICED_TOPIC_PUBLISH;
                big_mqtt.topic_len = strlen( WICED_TOPIC_PUBLISH );
                free( mqtt_data );
                mqtt_data = NULL;
            }
            if ( state == MQTT_EVENT_SUBSCRIBE )
            {
                WPRINT_APP_INFO( ("\n RECIEVED MESH DATA \n") );
                ret = wiced_JSON_parser( (const char*) msg.data, msg.data_len );
                if ( ret == WICED_SUCCESS )
                {
                    WPRINT_APP_INFO( ( "\n Reported data from SHADOW :  [%s]\n", recieved_data ) );
                }

                process_send_mesh_pkt_rcvd( &big_mqtt, recieved_data);

            }
            if ( state == 3 )
            {
                WPRINT_APP_INFO( ("MQTT : Received in response of provision topic\n") );
            }
        }
        mqtt_connection_deinit( mqtt_object );
    } while ( WICED_TRUE );

    wiced_rtos_deinit_semaphore( &wake_semaphore );
    wiced_rtos_deinit_semaphore( &semaphore );
    WPRINT_APP_INFO( ("[MQTT] Deinit connection...") );
    wiced_mqtt_deinit( mqtt_object );
    free( mqtt_object );
    mqtt_object = NULL;

}

static wiced_result_t parse_json_shadow_status(wiced_json_object_t * json_object )
 {

#if (CLOUD_CONFIG == WICED_BLUEMIX_BROKER)
     if(strncmp(json_object->object_string, "meshdata", strlen("meshdata")) == 0)
     {
         WPRINT_APP_INFO((" json_object->object_string works fine \n"));
         WPRINT_APP_INFO((" json_object->value_length = %d \n", json_object->value_length));
         if(json_object->value_length > 0 && json_object->value_length < sizeof(recieved_data)-1)
         {
             memcpy(recieved_data, json_object->value, json_object->value_length);
             recieved_data[json_object->value_length] = '\0';
         }
         WPRINT_APP_INFO(("recieved_data = %s" , recieved_data));
     }
#elif (CLOUD_CONFIG == WICED_AWS_BROKER)

     if(strncmp(json_object->object_string, "status", strlen("status")) == 0)
          {
              WPRINT_APP_INFO((" json_object->object_string works fine \n"));
              WPRINT_APP_INFO((" json_object->value_length = %d \n", json_object->value_length));
              if(json_object->value_length > 0 && json_object->value_length < sizeof(recieved_data)-1)
              {
                  memcpy(recieved_data, json_object->value, json_object->value_length);
                  recieved_data[json_object->value_length] = '\0';
              }
              WPRINT_APP_INFO(("recieved_data = %s" , recieved_data));
          }
#endif

     return WICED_SUCCESS;
 }

#if (CLOUD_CONFIG == WICED_AWS_BROKER)
wiced_result_t mesh_read_aws_credentials( void )
{
    uint32_t              size_out = 0;
    /* Get AWS root certificate, client certificate and private key respectively */
    resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.ca_cert );
    security.ca_cert_len = size_out;

    resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_mesh_DIR_client_cer, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.cert );
    if ( size_out < 64 )
    {
        WPRINT_APP_INFO( ( "\nNot a valid Certificate! Please replace the dummy certificate file 'resources/app/aws/iot/mesh/client.cer' with the one got from AWS\n\n" ) );
        return WICED_FALSE;
    }
    security.cert_len = size_out;

    resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_mesh_DIR_privkey_cer, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.key );
    if ( size_out < 64 )
    {
        WPRINT_APP_INFO( ( "\nNot a valid Private Key! Please replace the dummy private key file 'resources/app/aws/iot/mesh/privkey.cer' with the one got from AWS\n\n" ) );
        return WICED_FALSE;
    }
    security.key_len = size_out;

    return WICED_TRUE;
}
#endif

static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{

    WPRINT_APP_INFO(( "mqtt_connection_event_cb, event : %d \n\n" , event->type));
    switch ( event->type )
    {

        case WICED_MQTT_EVENT_TYPE_DISCONNECTED:
            DISCONNECTED = WICED_TRUE;
            wiced_rtos_set_semaphore( &wake_semaphore );
            break;

        case WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS:
        case WICED_MQTT_EVENT_TYPE_PUBLISHED:
        case WICED_MQTT_EVENT_TYPE_SUBCRIBED:
        case WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED:
        {
            expected_event = event->type;
            wiced_rtos_set_semaphore( &semaphore );
        }
            break;
        case WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED:
        {
            msg = event->data.pub_recvd;
            WPRINT_APP_INFO(( "[MQTT] Received Data\n" ));
//            WPRINT_APP_INFO(( "mqtt_connection_event_cb : [MQTT] Received %.*s  for TOPIC : %.*s\n\n", (int) msg.data_len, msg.data, (int) msg.topic_len, msg.topic ));
//            WPRINT_APP_INFO(( "[MQTT] Received TOPIC : %s\n\n", msg.topic ));
//            WPRINT_APP_INFO(( "[MQTT] Received Data  : %s\n\n", msg.data ));

            if ( strncmp( (char*) msg.topic, (char*) WICED_TOPIC_SUBSCRIBE, msg.topic_len ) == 0 )
            {
                state = MQTT_EVENT_SUBSCRIBE;
                wiced_rtos_set_semaphore( &wake_semaphore );
            }

        }
            break;
        default:
            break;
    }
    return WICED_SUCCESS;
}

/*
 * Call back function to handle channel events.
 *
 * For each event:
 *  - The call back will set the expected_event global to the received event.
 *  - The call back will set the event semaphore to run any blocking thread functions waiting on this event
 *  - Some events will also log other global variables required for extra processing.
 *
 * A thread function will probably be waiting for the received event. Once the event is received and the
 * semaphore is set, the thread function will check for the received event and make sure it matches what
 * it is expecting.
 *
 * Note:  This mechanism is not thread safe as we are using a non protected global variable for read/write.
 * However as this snip is a single controlled thread, there is no risc of racing conditions. It is
 * however not recommended for multi-threaded applications.
 */

/*
 * A blocking call to an expected event.
 */
static wiced_result_t mqtt_wait_for( wiced_mqtt_event_type_t event, uint32_t timeout )
{
    if ( wiced_rtos_get_semaphore( &semaphore, timeout ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    else
    {
        if ( event != expected_event )
        {
            return WICED_ERROR;
        }
    }
    return WICED_SUCCESS;
}

/*
 * Open a connection and wait for WICED_MQTT_TIMEOUT period to receive a connection open OK event
 */
static wiced_result_t mqtt_conn_open( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security )
{
    wiced_mqtt_pkt_connect_t conninfo;
    wiced_result_t ret = WICED_SUCCESS;
    memset( &conninfo, 0, sizeof( conninfo ) );
    conninfo.port_number = 0;                   /* set to 0 indicates library to use default settings */
    conninfo.mqtt_version = WICED_MQTT_PROTOCOL_VER4;
    conninfo.clean_session = 1;
    conninfo.client_id = (uint8_t*) CLIENT_ID;
    conninfo.keep_alive = 10;
    conninfo.password = (uint8_t*) PASSWORD;
    conninfo.username = (uint8_t*) USERNAME;
    ret = wiced_mqtt_connect( mqtt_obj, address, interface, callback, security, &conninfo );
    if ( ret != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS, WICED_MQTT_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}


/*
 * Subscribe to WICED_TOPIC and wait for 5 seconds to receive an ACM.
 */
static wiced_result_t mqtt_app_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_subscribe( mqtt_obj, topic, qos );
    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_SUBCRIBED, WICED_MQTT_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}


/*
 * Publish (send) WICED_MESSAGE_STR to WICED_TOPIC and wait for 5 seconds to receive a PUBCOMP (as it is QoS=2).
 */

 wiced_result_t mqtt_app_publish( wiced_mqtt_object_t mqtt_obj, uint8_t qos, char *topic, uint8_t *data, uint32_t data_len )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_publish( mqtt_obj, topic, data, data_len, qos );

    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }

    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_PUBLISHED, WICED_MQTT_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}
#endif
