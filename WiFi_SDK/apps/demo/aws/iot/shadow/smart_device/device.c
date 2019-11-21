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
 * This application should run on the device which needs to be controlled using IOT protocols and Cloud.
 * The device used here is LED1 on the WICED board.
 *
 * It has 2 states (ON/OFF) which could be controlled using one of the below mentioned topics which the device subscribes:
 * a) Name of this device (thing name). Whoever publishes message to this thing name (device) can control the device.
 *    This can be treated as SMART CONTROL of the device. This publish message can be triggered from one of the below:
 *    i) AWS Ice Breaker.
 *    ii) Other AWS Cloud service like Lambda.
 *    iii) Directly from remote sensor. We are using this for this demo.
 * b) AWS shadow for this thing name (device). This is used to create a virtual instance of this device in cloud.
 *    Users can get to know the status of the thing (device) by checking shadow. Also the device can be controlled using Shadow.
 *    This topic is derived from thing name which needs to be configured as already mentioned in item a) above.
 *    This can be treated as REMOTE CONTROL of the device.
 *
 * Also the device can be configured to be smart controlled or not using 2 ways:
 * i) Press SW3 button to allow/disallow smart control.
 * ii) Update "auto" element in "desired" object to "YES" or "NO" in AWS Shadow for the given thing name
 *
 * The device publishes to shadow when there is any change in its status.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Make sure AWS Root Certifcate 'resources/apps/aws/iot/rootca.cer' is proper while building the app.
 * 2. Configuring the device using Web-UI:
 *    a) Connect from a PC to the device SSID "WICED_AWS" which runs as soft AP. SSID and credentials are as per DCT configuration.
 *    b) After successful connection, this device will act as WebServer with IP address 192.168.0.1.
 *    c) From host system type URL in browser as 192.168.0.1.
 *    d) Configure settings for thing name and upload certificate and private key. And join to a router which is connected to internet.
 *    e) The device will now reboot.
 * 3. The device will connect to selected Wi-Fi configurations. And then connect to broker.
 * 4. Try to control the device with one of the following:
 *    a) Start sensor app and try to get ON/OFF published to the device thing name.
 *    b) Update desired shadow status (ON/OFF) for the device thing name in the AWS IOT service.
 *    Item a) can be disabled/enabled by toggling the SW3 button on the device or by changing the "auto" state in the shadow to YES/NO.
 *
 */

#include "wiced.h"
#include "resources.h"
#include "JSON.h"
#include "device_config.h"
#include "wiced_aws.h"
#include "aws_common.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define APP_CERTIFICATES_MAX_SIZE        (0x7fffffff)
#define APP_DELAY_IN_MILLISECONDS        (1000)

#define SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED    "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"OFF\" , \"auto\": \"NO\"} ,\
                                                                        \"reported\": { \"status\": \"OFF\" ,\"auto\": \"NO\"} \
                                                                    }\
                                                                }"

#define SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED     "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"ON\" , \"auto\": \"YES\" } ,\
                                                                        \"reported\": { \"status\": \"ON\" ,\"auto\": \"YES\"} \
                                                                    }\
                                                                }"
#define SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON    "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"OFF\" , \"auto\": \"YES\" } ,\
                                                                        \"reported\": { \"status\": \"OFF\" ,\"auto\": \"YES\"} \
                                                                    }\
                                                                }"

#define SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF     "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"ON\" , \"auto\": \"NO\" },\
                                                                        \"reported\": { \"status\": \"ON\" ,\"auto\": \"NO\"}\
                                                                    }\
                                                                }"

#define CLIENT_ID                           "wiced_subcriber_aws"

#define APP_SUBSCRIBE_RETRY_COUNT          (3)

#define APP_CONNECTION_NUMBER_OF_RETRIES   (3)

/******************************************************
 *               Variable Definitions
 ******************************************************/

typedef struct
{
    wiced_semaphore_t         wake_semaphore;
    char                      thing_name[32];
    char                      shadow_state_topic[64];
    char                      shadow_delta_topic[64];
    wiced_aws_handle_t        handle;
} my_smart_device_info_t;

static my_smart_device_info_t app_info;

static char*          led_status = "OFF";
static char           req_led_status[8] = "OFF";

static uint8_t        smart_control = 0;

static wiced_aws_thing_security_info_t my_smart_device_security_creds =
{
    /* Read security credentials either from DCT or 'resources' */
    .private_key        = NULL,
    .key_length         = 0,
    .certificate        = NULL,
    .certificate_length = 0,
};

static wiced_aws_endpoint_info_t my_smart_device_aws_iot_endpoint =
{
    .transport           = WICED_AWS_TRANSPORT_MQTT_NATIVE,

    .uri                 = "a38td4ke8seeky.iot.us-east-1.amazonaws.com",
    .ip_addr             = {0},
    .port                = WICED_AWS_IOT_DEFAULT_MQTT_PORT,
    .root_ca_certificate = NULL,
    .root_ca_length      = 0,
    .peer_common_name    = NULL,
};

static wiced_aws_thing_info_t my_smart_device_aws_config = {
    .name            = NULL,
    .credentials     = &my_smart_device_security_creds,
};

/******************************************************
 *               Static Function Definitions
 ******************************************************/

static wiced_result_t parse_json_shadow_status(wiced_json_object_t * json_object )
{
    if(strncmp(json_object->object_string, "status", strlen("status")) == 0)
    {
        if(json_object->value_length > 0 && json_object->value_length < sizeof(req_led_status)-1)
        {
            memcpy(req_led_status, json_object->value, json_object->value_length);
            req_led_status[json_object->value_length] = '\0';
        }
    }
    else if(strncmp(json_object->object_string, "auto", strlen("auto")) == 0)
    {
        if ( strncmp( json_object->value, "YES", 3 ) == 0 )
        {
            smart_control = 1;
        }
        else
        {
            smart_control = 0;
        }
        WPRINT_APP_INFO (( "smart_control %d\n", smart_control ));
    }

    return WICED_SUCCESS;
}

/*
 * Call back function to handle AWS events.
 */
static void my_smart_device_aws_callback( wiced_aws_handle_t aws, wiced_aws_event_type_t event, wiced_aws_callback_data_t* data )
{
    if( !aws || !data || ( aws != app_info.handle ) )
        return;

    switch ( event )
    {
        case WICED_AWS_EVENT_CONNECTED:
        case WICED_AWS_EVENT_DISCONNECTED:
        case WICED_AWS_EVENT_PUBLISHED:
        case WICED_AWS_EVENT_SUBSCRIBED:
        case WICED_AWS_EVENT_UNSUBSCRIBED:
            break;

        case WICED_AWS_EVENT_PAYLOAD_RECEIVED:
        {
            wiced_result_t ret = WICED_ERROR;
            wiced_aws_callback_message_t message;
            if( !data )
                return;

            message = data->message;

            WPRINT_APP_DEBUG ( ("[Smart-Device] Received Payload [ Topic: %.*s ] :\n====\n%.*s\n====\n",
                (int) message.topic_length, message.topic, (int) message.data_length, message.data ) );

            if ( strncmp( (char*) message.topic, (const char*) app_info.thing_name, message.topic_length ) == 0 )
            {
                WPRINT_APP_INFO (( "From Sensor :: Requested LED State [%.*s]. Current LED State [%s]\n", (int) message.data_length, message.data, led_status ));

                if ( strncasecmp( (char*) message.data, led_status, message.data_length ) )
                {
                    if ( smart_control == 1 )
                    {
                        if ( strncmp( (char*) message.data, "ON", 2 ) == 0 )
                            led_status = "ON";
                        else
                            led_status = "OFF";

                        wiced_rtos_set_semaphore( &app_info.wake_semaphore );
                    }
                }
                else
                {
                    break;
                }
            }
            else if ( strncmp( (char*) message.topic, app_info.shadow_delta_topic, message.topic_length ) == 0 )
            {
                ret = wiced_JSON_parser( (const char*)message.data , message.data_length );
                if ( ret == WICED_SUCCESS )
                {
                    WPRINT_APP_INFO(( "From Shadow :: Requested LED State [%s]. Current LED State [%s]\n", req_led_status, led_status ));
                    if ( strncmp( req_led_status, "ON", 2 ) == 0 )
                    {
                        led_status = "ON";
                    }
                    else
                    {
                        led_status = "OFF";
                    }
                    wiced_rtos_set_semaphore( &app_info.wake_semaphore );
                }
            }
            else
            {
                WPRINT_APP_INFO(( "Topic Not found\n" ));
            }
            break;
        }
        default:
            break;
    }
}

static void publish_callback( void* arg )
{
    if ( smart_control == 0 )
    {
        smart_control = 1;
    }
    else
    {
        smart_control = 0;
    }

    WPRINT_APP_INFO (( "smart_control %d\n", smart_control ));
}

static wiced_result_t get_aws_credentials_from_dct( void )
{
    wiced_result_t ret  = WICED_ERROR;
    uint32_t size_out   = 0;
    platform_dct_security_t* dct_security = NULL;
    wiced_aws_thing_security_info_t* security = &my_smart_device_security_creds;
    uint8_t** root_ca_certificate = &my_smart_device_aws_iot_endpoint.root_ca_certificate;

    if( security->certificate && security->private_key && (*root_ca_certificate) )
    {
        WPRINT_APP_INFO(("[Shadow] Security Credentials already set(not NULL). Abort Reading from DCT...\n"));
        return WICED_SUCCESS;
    }

    /* Get AWS Root CA certificate filename: 'rootca.cer' located @ resources/apps/aws/iot folder */
    ret = resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, 0, APP_CERTIFICATES_MAX_SIZE, &size_out, (const void **) root_ca_certificate);
    if( ret != WICED_SUCCESS )
    {
        return ret;
    }

    if( size_out < 64 )
    {
        WPRINT_APP_INFO( ( "\n[Shadow] Invalid Root CA Certificate! Replace the dummy certificate with AWS one[<YOUR_WICED_SDK>/resources/app/aws/iot/'rootca.cer']\n\n" ) );
        resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, (const void *)*root_ca_certificate );
        return WICED_ERROR;
    }

    my_smart_device_aws_iot_endpoint.root_ca_length = size_out;

    /* Reading Security credentials of the device from DCT section */
    WPRINT_APP_INFO(( "[Shadow] Reading Device's certificate and private key from DCT...\n" ));

    /* Lock the DCT to allow us to access the certificate and key */
    ret = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Shadow] Unable to lock DCT to read certificate\n"));
        return ret;
    }

    security->certificate           = (uint8_t *)dct_security->certificate;
    security->certificate_length    = strlen( dct_security->certificate );
    security->private_key           = (uint8_t *)dct_security->private_key;
    security->key_length            = strlen( dct_security->private_key );

    /* Finished accessing the certificates */
    wiced_dct_read_unlock( dct_security, WICED_FALSE );

    return WICED_SUCCESS;
}

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    wiced_aws_handle_t aws_handle;
    wiced_result_t ret = WICED_SUCCESS;
    uint32_t       connection_retries = 0;
    uint32_t       retries = 0;

    wiced_init( );

    /* Checks if User has triggered the DCT reset, if Yes - start the configuration routine */
    ret = aws_configure_device();
    if ( ret != WICED_ALREADY_INITIALIZED )
    {
        WPRINT_APP_INFO(("[Shadow] Restarting the device...\n"));
        wiced_framework_reboot();
        return;
    }

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    ret = get_aws_credentials_from_dct();
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Shadow] Failed to get Security credentials for AWS IoT\n"));
        return;
    }

    wiced_rtos_init_semaphore( &app_info.wake_semaphore );

    ret = wiced_aws_init( &my_smart_device_aws_config, my_smart_device_aws_callback );
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Shadow] Failed to Initialize Wiced AWS library\n"));
        return;
    }

    aws_handle = (wiced_aws_handle_t)wiced_aws_create_endpoint(&my_smart_device_aws_iot_endpoint);

    if( !aws_handle )
    {
        WPRINT_APP_INFO( ( "[Shadow] Failed to create AWS connection handle\n" ) );
        return;
    }

    app_info.handle = aws_handle;

    wiced_JSON_parser_register_callback(parse_json_shadow_status);

    wiced_gpio_input_irq_enable( WICED_BUTTON1, IRQ_TRIGGER_RISING_EDGE, publish_callback, NULL );
    do
    {
        ret = wiced_aws_connect( aws_handle );
        connection_retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( connection_retries < APP_CONNECTION_NUMBER_OF_RETRIES ) );

    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Smart-Device] Failed to connect to AWS endpoint\n"));
        return;
    }

    wiced_aws_publish( aws_handle, app_info.shadow_state_topic,
                            (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED,
                            sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED ),
                            WICED_AWS_QOS_ATLEAST_ONCE );

    wiced_rtos_delay_milliseconds( APP_DELAY_IN_MILLISECONDS * 2 );

    wiced_aws_subscribe( aws_handle, app_info.shadow_delta_topic, WICED_AWS_QOS_ATLEAST_ONCE );

    do
    {
        ret = wiced_aws_subscribe( aws_handle, app_info.thing_name, WICED_AWS_QOS_ATMOST_ONCE );
        retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( retries < APP_SUBSCRIBE_RETRY_COUNT ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Smart-Device] Failed to Subscribe on Topic: %s\n", app_info.thing_name ) );
        return;
    }

    while ( 1 )
    {
        /* Wait forever on wake semaphore until the LED status is changed */
        wiced_rtos_get_semaphore( &app_info.wake_semaphore, WICED_NEVER_TIMEOUT );

        /* Toggle the LED */
        WPRINT_APP_INFO(("[Smart-Device] Publishing to Thing state topic\n" ) );
        if ( ( strncasecmp( led_status, "OFF", 3 ) == 0 ) && smart_control == 1 )
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            strcpy(req_led_status, led_status);
            wiced_aws_publish( aws_handle, app_info.shadow_state_topic,
                                (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON,
                                sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON ),
                                WICED_AWS_QOS_ATLEAST_ONCE );
        }
        else if ( ( strncasecmp( led_status, "ON", 2 ) == 0 ) && smart_control == 0 )
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            strcpy(req_led_status, led_status);
            wiced_aws_publish( aws_handle, app_info.shadow_state_topic,
                                (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF,
                                sizeof( SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF ),
                                WICED_AWS_QOS_ATLEAST_ONCE );
        }
        else if ( ( strncasecmp( led_status, "ON", 2 ) == 0 ) && smart_control == 1 )
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            strcpy(req_led_status, led_status);
            wiced_aws_publish( aws_handle, app_info.shadow_state_topic,
                                (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED,
                                sizeof( SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED ),
                                WICED_AWS_QOS_ATLEAST_ONCE );
        }
        else
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            strcpy(req_led_status, led_status);
            wiced_aws_publish( aws_handle, app_info.shadow_state_topic,
                                (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED,
                                sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED ),
                                WICED_AWS_QOS_ATLEAST_ONCE );
        }
    }

    wiced_aws_disconnect( aws_handle );

    wiced_aws_deinit();

    wiced_rtos_deinit_semaphore( &app_info.wake_semaphore );

    return;
}
