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
 * This application should be run on the sensors. The sensor used here is Thermistor (temperature sensor).
 * The app gets outside temperature from openweathermap.org and room temperature from the sensor.
 * Finds the difference and publishes ON/OFF to the device (thing name) which it wants to control smartly.
 *
 * This application reads room temperature every 5 seconds and gets outside temperature from www.openweathermap.org.
 * And publishes "ON" or "OFF" based on the difference in temperature with QOS-1.
 * If temperature difference is >= 5 degrees it publishes "ON" to the topic configured via Web-Ui else publishes "OFF".
 *
 * To demonstrate the app, work through the following steps.
 * 1. Make sure AWS Root Certifcate 'resources/apps/aws/iot/rootca.cer' is proper while building the app.
 * 2. Configuring the sensor using Web-UI:
 *    a) Connect from a PC to the sensor SSID "WICED_AWS" which runs as soft AP. SSID and credentials are as per DCT configuration.
 *    b) After successful connection, this sensor will act as WebServer with IP address 192.168.0.1.
 *    c) From host system type URL in browser as 192.168.0.1.
 *    d) Configure settings for thing name and upload certificate and private key. And join to a router which is connected to internet.
 *    e) The WICED board (sensor) will now reboot.
 * 3. The device will connect to selected Wi-Fi configurations. And then connect to broker.
 * 4. To send "ON" to the device try to make the temperature difference >= 5 degrees else make it < 5.
 *
 * NOTE: In this APP, for getting weather data from openweathermap.org, Cypress specific user ID is used.
 *       Follow the mentioned link http://openweathermap.org/appid to generate own APPID
 *       Please use your own user ID instead for better results. Currently used ID "APPID=611684a92a4de8b5f4281ecb71f12002".
 *
 */

#include "wiced.h"
#include "resources.h"
#include <math.h>
#include "thermistor.h" // Using Murata NCP18XH103J03RB thermistor
#include "JSON.h"
#include "device_config.h"
#include "wiced_aws.h"
#include "aws_common.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define APP_DELAY_IN_MILLISECONDS           (1000)

#define CLIENT_ID                           "wiced_publisher_aws"
#define BUFFER_LENGTH                       (2048)
#define MQTT_PUBLISH_RETRY_COUNT            (3)
#define WEATHER_HOST_NAME                   "api.openweathermap.org"
#define WEATHER_CITY_NAME                   "bangalore"
/* Follow the http://openweathermap.org/appid to get your own APP ID */
#define WEATHER_USER_ID                     "APPID=611684a92a4de8b5f4281ecb71f12002"
#define WEATHER_HTTP_GET_REQUEST            "GET /data/2.5/weather?q="WEATHER_CITY_NAME"&"WEATHER_USER_ID" HTTP/1.1\r\n" \
                                            "Host: "WEATHER_HOST_NAME"\r\n" \
                                            "Connection: close\r\n" \
                                            "\r\n"
#define MSG_ON                              "ON"
#define MSG_OFF                             "OFF"
#define CONTENT_LENGTH_FIELD                "Content-Length:"

#define SENSOR_CERTIFICATES_MAX_SIZE        (0x7fffffff)

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    int16_t               last_sample;
} temp_data_t;

typedef struct
{
    char                      thing_name[32];
    wiced_aws_handle_t        handle;
} my_sensor_app_info_t;

/******************************************************
 *               Variable Definitions
 ******************************************************/

static my_sensor_app_info_t      app_info;
static temp_data_t               temperature_data;
static float                     outside_temperature;
static wiced_ip_address_t        http_address;
static uint8_t                   buffer[BUFFER_LENGTH];

static wiced_aws_thing_security_info_t my_sensor_security_creds =
{
    /* Read security credentials either from DCT or 'resources' */
    .private_key        = NULL,
    .key_length         = 0,
    .certificate        = NULL,
    .certificate_length = 0,
};

static wiced_aws_endpoint_info_t my_sensor_aws_iot_endpoint =
{
    .transport           = WICED_AWS_TRANSPORT_MQTT_NATIVE,

    .uri                 = "a38td4ke8seeky.iot.us-east-1.amazonaws.com",
    .ip_addr             = {0},
    .port                = WICED_AWS_IOT_DEFAULT_MQTT_PORT,
    .root_ca_certificate = NULL,
    .root_ca_length      = 0,
    .peer_common_name    = NULL,
};

static wiced_aws_thing_info_t my_sensor_aws_config = {
    .name            = NULL,
    .credentials     = &my_sensor_security_creds,
};

/******************************************************
 *               Static Function Definitions
 ******************************************************/

/* FIXME: Using wiced_http_get() causes to include two conflicting libraries: protocols/http & protocols/http_client.
 * Change this send-data function to use protocols/http_client later.
 */
static wiced_result_t send_raw_http_data( wiced_ip_address_t* address, const char* query, void* buffer, uint32_t buffer_length )
{
    wiced_tcp_socket_t socket;
    wiced_packet_t*    reply_packet;
    wiced_result_t     result = WICED_ERROR;
    wiced_result_t     rx_result;
    char *             buffer_ptr = (char*) buffer;

    if ( ( result = wiced_tcp_create_socket( &socket, WICED_STA_INTERFACE ) ) != WICED_SUCCESS )
    {
        return result;
    }
    result = wiced_tcp_connect( &socket, address, 80, 10000 );
    if ( result != WICED_SUCCESS )
    {
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    if ( ( result = wiced_tcp_send_buffer( &socket, query, (uint16_t) strlen( query ) ) != WICED_SUCCESS ) )
    {
        wiced_tcp_disconnect( &socket );
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    WPRINT_APP_INFO( ("[Sensor] waiting for HTTP reply\n") );

    do
    {
        rx_result = wiced_tcp_receive( &socket, &reply_packet, 5000 );
        if ( rx_result == WICED_SUCCESS )
        {
            uint8_t* data;
            uint16_t offset;
            uint16_t data_length;
            uint16_t available;
            uint32_t data_to_copy;

            /* Record the fact we received a reply of some kind */
            result = WICED_SUCCESS;

            /* Copy data into provided buffer */
            offset = 0;
            do
            {
                wiced_packet_get_data( reply_packet, offset, &data, &data_length, &available );
                data_to_copy = MIN(data_length, buffer_length);
                memcpy( buffer_ptr, data, data_to_copy );
                buffer_ptr    += data_to_copy;
                buffer_length -= data_to_copy;
                offset  = (uint16_t)(offset + data_length);
            } while (data_length < available);
            wiced_packet_delete( reply_packet );
        }
    } while ( rx_result == WICED_SUCCESS );

    wiced_tcp_disconnect( &socket );
    wiced_tcp_delete_socket( &socket );

    return ( result );
}

static wiced_result_t parse_json_weather_info(wiced_json_object_t * json_object )
{
    if(strncmp(json_object->object_string, "temp", json_object->object_string_length) == 0)
    {
        outside_temperature = atof(json_object->value);
        WPRINT_APP_DEBUG(("[Sensor] json_object->value [%.*s] outside_temperature [%f]\n", json_object->value_length, json_object->value, outside_temperature));
    }

    return WICED_SUCCESS;
}

static wiced_result_t get_weather_info( float *value)
{
    int http_status_code = 0;
    wiced_result_t result;
    char *data;
    int content_length = 0;

    result = send_raw_http_data( &http_address, WEATHER_HTTP_GET_REQUEST, buffer, sizeof( buffer ) );
    if ( result == WICED_SUCCESS )
    {
        http_status_code = atoi((char*)(buffer+(strlen( "HTTP/1.1" ) + 1)));
        if ( http_status_code < 200 || http_status_code > 299 )
        {
            WPRINT_APP_INFO( ( "[Sensor] HTTP error code [%d] while getting weather data!\n", http_status_code ) );
            return WICED_UNSUPPORTED;
        }

        data = strncasestr( (char*) buffer, sizeof(buffer), CONTENT_LENGTH_FIELD, strlen(CONTENT_LENGTH_FIELD) );
        if(data != NULL)
        {
            data += strlen(CONTENT_LENGTH_FIELD);
            content_length = atoi((char*)(data));

            data = strstr( (char*) data, "\r\n\r\n" );
            if ( data != NULL )
            {
                data += strlen("\r\n\r\n");
                result = wiced_JSON_parser( (const char*)data,  content_length);
                if(result == WICED_SUCCESS)
                {
                    *value = outside_temperature;
                }
            }
        }
    }
    else
        WPRINT_APP_INFO( ( "[Sensor] HTTP GET failed: %u\n", result ) );

    return result;
}

/*
 * Call back function to handle AWS events
 */

static void my_sensor_aws_callback( wiced_aws_handle_t aws, wiced_aws_event_type_t event, wiced_aws_callback_data_t* data )
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
            wiced_aws_callback_message_t* message = NULL;
            if( !data )
                return;

            message = &data->message;

            WPRINT_APP_INFO( ("[Sensor] Received Payload [ Topic: %.*s ] :\n====\n%.*s\n====\n",
                (int) message->topic_length, message->topic, (int) message->data_length, message->data ) );
            break;
        }
        default:
            break;
    }
}

static wiced_result_t get_aws_credentials_from_dct( void )
{
    wiced_result_t ret  = WICED_ERROR;
    uint32_t size_out   = 0;
    platform_dct_security_t* dct_security = NULL;
    wiced_aws_thing_security_info_t* security = &my_sensor_security_creds;
    uint8_t** root_ca_certificate = &my_sensor_aws_iot_endpoint.root_ca_certificate;

    if( security->certificate && security->private_key && (*root_ca_certificate) )
    {
        WPRINT_APP_INFO(("[Sensor] Security Credentials already set(not NULL). Abort Reading from DCT...\n"));
        return WICED_SUCCESS;
    }

    /* Get AWS Root CA certificate filename: 'rootca.cer' located @ resources/apps/aws/iot folder */
    ret = resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, 0, SENSOR_CERTIFICATES_MAX_SIZE, &size_out, (const void **) root_ca_certificate);
    if( ret != WICED_SUCCESS )
    {
        return ret;
    }

    if( size_out < 64 )
    {
        WPRINT_APP_INFO( ( "\n[Sensor] Invalid Root CA Certificate! Replace the dummy certificate with AWS one[<YOUR_WICED_SDK>/resources/app/aws/iot/'rootca.cer']\n\n" ) );
        resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, (const void *)*root_ca_certificate );
        return WICED_ERROR;
    }

    my_sensor_aws_iot_endpoint.root_ca_length = size_out;

    /* Reading Security credentials of the device from DCT section */

    WPRINT_APP_INFO(( "[Sensor] Reading Device's certificate and private key from DCT...\n" ));

    /* Lock the DCT to allow us to access the certificate and key */
    ret = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Sensor] Unable to lock DCT to read certificate\n"));
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
    float            value = 0, delta = 0;
    char            *msg = MSG_OFF;
    wiced_result_t   ret = WICED_SUCCESS;
    int              retries;
    wiced_aws_handle_t aws_sensor_handle;
    wiced_init( );

    /* Checks if User has triggered the DCT reset, if Yes - start the configuration routine */
    ret = aws_configure_device();
    if ( ret != WICED_ALREADY_INITIALIZED )
    {
        WPRINT_APP_INFO(("[Sensor] Restarting the device...\n"));
        wiced_framework_reboot();
        return;
    }

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    ret = get_aws_credentials_from_dct();
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Sensor] Failed to get Security credentials for AWS IoT\n"));
        return;
    }

    ret = wiced_hostname_lookup( WEATHER_HOST_NAME, &http_address, 10000, WICED_STA_INTERFACE );
    if ( ret == WICED_ERROR )
    {
        WPRINT_APP_INFO(("[Sensor] DNS Error resolving %s\n", WEATHER_HOST_NAME ) );
        return;
    }
    WPRINT_APP_INFO( ( "[Sensor] Weather Server IP: %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 0) ) );

    ret = wiced_aws_init( &my_sensor_aws_config, my_sensor_aws_callback );
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Sensor] Failed to Initialize Wiced AWS library\n"));
        return;
    }

    aws_sensor_handle = (wiced_aws_handle_t)wiced_aws_create_endpoint(&my_sensor_aws_iot_endpoint);
    if( !aws_sensor_handle )
    {
        WPRINT_APP_INFO( ( "[Sensor] Failed to create AWS connection handle\n" ) );
        return;
    }

    app_info.handle = aws_sensor_handle;

    wiced_JSON_parser_register_callback(parse_json_weather_info);

    /* Initialize thermistor */
    wiced_adc_init( WICED_THERMISTOR_JOINS_ADC, 5 );
    memset( &temperature_data, 0, sizeof( temperature_data ) );

    do
    {
        ret = wiced_aws_connect( aws_sensor_handle );
        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("[Sensor] Failed to connect AWS endpoint\n"));
            break;
        }

        while ( 1 )
        {
            /* Getting the temperature value for bangalore location from www.openweathermap.org */
            if ( get_weather_info( &value ) == WICED_SUCCESS )
            {
                /* Reading temperature sensor value */
                thermistor_take_sample( WICED_THERMISTOR_JOINS_ADC, &temperature_data.last_sample );
                delta = temperature_data.last_sample - value;
                WPRINT_APP_INFO(( "[Sensor] Delta = %.1f - %.1f = %.1f degrees\n", temperature_data.last_sample/10.0, value/10.0, (delta/10) ));

                if ( abs( delta ) >= 50 )
                {
                    msg = MSG_ON;
                }
                else
                {
                    msg = MSG_OFF;
                }
                /* Controlling the LED by publishing to mqtt topic "WICED_BULB" */
                retries = 0;
                do
                {
                    ret = wiced_aws_publish( aws_sensor_handle, app_info.thing_name, (uint8_t*) msg, strlen( msg ), WICED_AWS_QOS_ATLEAST_ONCE );
                    retries++;
                } while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_PUBLISH_RETRY_COUNT ) );
                if ( ret != WICED_SUCCESS )
                {
                    break;
                }
            }

            wiced_rtos_delay_milliseconds( 5000 );
        }

        wiced_aws_disconnect( aws_sensor_handle );

        wiced_rtos_delay_milliseconds( APP_DELAY_IN_MILLISECONDS * 2 );
    } while ( 1 );

    wiced_aws_disconnect( aws_sensor_handle );

    WPRINT_APP_INFO(("[Sensor] Deinitializing AWS...\n"));

    wiced_aws_deinit();

    return;
}
