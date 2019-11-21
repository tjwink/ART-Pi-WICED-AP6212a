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
 * Temperature Control & Report Application
 *
 * This application measures the temperature of the WICED evaluation
 * board and displays temperature & setpoint status on a local webpage.
 *
 * The application demonstrates the following features ...
 *  - Wi-Fi client mode to send temperature measurements to the Internet
 *  - DNS redirect
 *  - Webserver
 *  - Gedday mDNS / DNS-SD Network Discovery
 *  - GPIO interface to capture button inputs
 *  - Timer peripheral interface to control LED brightness (via PWM)
 *  - Factory Programming - please read WICED-AN8xx-R Factory Programming Application Note
 *
 * [TBD : The application will eventually be configured to control the temperature of
 *        a thermistor on a personality board plugged into the WICED evaluation board.
 *        At present, the temperature and setpoint are independent. There is no feedback
 *        mechanism in place to heat or cool the thermistor (to make the board temperature
 *        equal to the setpoint).]
 *
 * Device Configuration
 *    The application is configured to use the Wi-Fi configuration
 *    from the local wifi_config_dct.h file. Application configuration is taken
 *    from the temp_control_dct.c file and stored in the Device Configuration table (DCT).
 * *
 * Application Operation
 * This section provides a description of the application flow and usage.
 * The app runs in a thread, the entry point is application_start()
 *
 *    Startup
 *      - Initialise the device
 *      - Check the device has a valid configuration in the DCT
 *      - Setup peripherals including buttons, LEDs, ADC and timers
 *      - Start the network interface to connect the device to the network
 *      - Set the local time from a time server on the internet
 *      - Setup a timer to take temperature measurements
 *      - Start a webserver to display temperature & setpoint values
 *      - Start Gedday to advertise the webserver on the network
 *
 *    Usage
 *        Two buttons on the eval board are used to increase or decrease
 *        the temperature setpoint. The D1 & D2 LEDs on the eval board change
 *        in brightness according to the temperature setpoint.
 *
 *        The current time, date, board temperature and setpoint are published to
 *        a webpage by a local webserver. The setpoint can also be changed using
 *        buttons on the webpage.
 *
 *        A mDNS (or Bonjour) browser may be used to find the webpage, or alternately, the
 *        IP address of the device (which is printed to the UART) may be entered
 *        into a web browser.
 *
 */


#include "temp_control.h"
#include "temp_control_dct.h"
#include <math.h>
#include "button_manager.h"
#include "wiced.h"
#include "thermistor.h" // Using Murata NCP18XH103J03RB thermistor
#include "http_server.h"
#include "sntp.h"
#include "gedday.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_NTP_ATTEMPTS        (3)
#define BUTTON_WORKER_STACK_SIZE            ( 1024 )
#define BUTTON_WORKER_QUEUE_SIZE            ( 4 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    float         temperature;
    wiced_mutex_t mutex;
} setpoint_t;

typedef struct
{
    temperature_reading_t temperature_readings[MAX_TEMPERATURE_READINGS];
    uint16_t              temperature_reading_index;
    uint16_t              last_sent_temperature_index;
    int16_t               last_sample;
    wiced_mutex_t         mutex;
} temp_data_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t take_temperature_reading       ( void* arg );
static float          get_setpoint                   ( void );
static void           increase_setpoint              ( void );
static void           decrease_setpoint              ( void );
static void           adjust_setpoint_led_brightness ( void );

static void           setpoint_control_button_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state );
static int32_t        process_temperature_update     ( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data );
static int32_t        process_temperature_up         ( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data );
static int32_t        process_temperature_down       ( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static temp_data_t         temperature_data;
static setpoint_t          setpoint;
static wiced_http_server_t http_server;
static wiced_timed_event_t temperature_timed_event;
static START_OF_HTTP_PAGE_DATABASE(web_pages)
    ROOT_HTTP_PAGE_REDIRECT("/apps/temp_control/main.html"),
    { "/apps/temp_control/main.html",    "text/html",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_apps_DIR_temp_control_DIR_main_html, },
    { "/temp_report.html",               "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_update, 0 }, },
    { "/temp_up",                        "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_up, 0 }, },
    { "/temp_down",                      "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_down, 0 }, },
    { "/images/favicon.ico",             "image/vnd.microsoft.icon", WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_favicon_ico, },
    { "/scripts/general_ajax_script.js", "application/javascript",   WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_scripts_DIR_general_ajax_script_js, },
    { "/images/cypresslogo.png",         "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_png, },
    { "/images/cypresslogo_line.png",    "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png, },
    { "/styles/buttons.css",             "text/css",                 WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_styles_DIR_buttons_css, },
END_OF_HTTP_PAGE_DATABASE();

static const wiced_button_manager_configuration_t button_manager_configuration =
{
    .short_hold_duration     = 500  * MILLISECONDS,
    .debounce_duration       = 100  * MILLISECONDS,

    .event_handler           = setpoint_control_button_handler,
};

/* Static button configuration */
static const wiced_button_configuration_t button_configurations[] =
{
#if ( WICED_PLATFORM_BUTTON_COUNT > 0 )
    [ TEMPERATURE_UP_BUTTON ]     = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT , SETPOINT_UP_KEY_CODE },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 1 )
    [ TEMPERATURE_DOWN_BUTTON ]   = { PLATFORM_BUTTON_2, BUTTON_CLICK_EVENT , SETPOINT_DOWN_KEY_CODE },
#endif
};

/* Button objects for the button manager */
button_manager_button_t buttons[] =
{
#if ( WICED_PLATFORM_BUTTON_COUNT > 0 )
    [ TEMPERATURE_UP_BUTTON ]             = { &button_configurations[ TEMPERATURE_UP_BUTTON ]        },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 1 )
    [ TEMPERATURE_DOWN_BUTTON ]           = { &button_configurations[ TEMPERATURE_DOWN_BUTTON ]     },
#endif
};

static button_manager_t button_manager;
wiced_worker_thread_t   button_worker_thread;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    uint16_t max_sockets = 10;

    wiced_init( );

    /* Initialise temperature data. Temperature data are shared among multiple threads; therefore a mutex is required */
    memset( &temperature_data, 0, sizeof( temperature_data ) );
    wiced_rtos_init_mutex( &temperature_data.mutex );

    /* Initialise temperature set point. Set point is shared among multiple threads; therefore a mutex is required */
    wiced_rtos_init_mutex( &setpoint.mutex );
    setpoint.temperature = DEFAULT_SETPOINT;
    adjust_setpoint_led_brightness( );

    /* Initialise Thermistor */
    wiced_adc_init( THERMISTOR_ADC, 5 );

    wiced_rtos_create_worker_thread( &button_worker_thread, WICED_DEFAULT_WORKER_PRIORITY, BUTTON_WORKER_STACK_SIZE, BUTTON_WORKER_QUEUE_SIZE );

    button_manager_init( &button_manager, &button_manager_configuration, &button_worker_thread, buttons, ARRAY_SIZE( buttons ) );

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

     /* Start automatic time synchronisation and synchronise once every day. */
    sntp_start_auto_time_sync( 1 * DAYS );

    /* Setup a timed event that will take a measurement */
    wiced_rtos_register_timed_event( &temperature_timed_event, WICED_HARDWARE_IO_WORKER_THREAD, take_temperature_reading, 1 * SECONDS, 0 );

    /* Start web server to display current temperature & setpoint */
    wiced_http_server_start( &http_server, 80, max_sockets, web_pages, WICED_STA_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );

    /* Advertise webpage services using Gedday */
    gedday_init( WICED_STA_INTERFACE, "wiced-temp-controller" );
    gedday_add_service( "temp_control web server", "_http._tcp.local", 80, 300, "" );
}

/*
 * Takes a temperature reading
 */
static wiced_result_t take_temperature_reading( void* arg )
{
    UNUSED_PARAMETER(arg);

    wiced_rtos_lock_mutex( &temperature_data.mutex );

    /* Take thermistor reading */
    if ( thermistor_take_sample( THERMISTOR_ADC, &temperature_data.last_sample ) != WICED_SUCCESS )
    {
        wiced_rtos_unlock_mutex( &temperature_data.mutex );
        return WICED_ERROR;
    }

    /* Get the current ISO8601 time */
    wiced_time_get_iso8601_time( &temperature_data.temperature_readings[temperature_data.temperature_reading_index].timestamp );

    /* Create sample string */
    xively_u16toa( temperature_data.last_sample / 10, temperature_data.temperature_readings[temperature_data.temperature_reading_index].sample, 2 );
    temperature_data.temperature_readings[temperature_data.temperature_reading_index].sample[2] = '.';
    xively_u16toa( temperature_data.last_sample % 10, &temperature_data.temperature_readings[temperature_data.temperature_reading_index].sample[3], 1 );

    if ( ( ++temperature_data.temperature_reading_index ) == MAX_TEMPERATURE_READINGS )
    {
        temperature_data.temperature_reading_index = 0;
    }

    wiced_rtos_unlock_mutex( &temperature_data.mutex );

    return WICED_SUCCESS;
}

/*
 * Gets current temperature set point
 */
static float get_setpoint( void )
{
    float current_setpoint;

    wiced_rtos_lock_mutex( &setpoint.mutex );
    current_setpoint = setpoint.temperature;
    wiced_rtos_unlock_mutex( &setpoint.mutex );

    return current_setpoint;
}

/*
 * Adjusts brightness of the set point LEDs
 */
static void adjust_setpoint_led_brightness( void )
{
    float up_led_brightness_level;
    float down_led_brightness_level;
    float up_led_duty_cycle;
    float down_led_duty_cycle;

    if ( setpoint.temperature > DEFAULT_SETPOINT )
    {
        up_led_brightness_level   = setpoint.temperature - DEFAULT_SETPOINT;
        down_led_brightness_level = 0.0f;
    }
    else
    {
        up_led_brightness_level   = 0.0f;
        down_led_brightness_level = DEFAULT_SETPOINT - setpoint.temperature;
    }

    /* Update LED brightness */
    up_led_duty_cycle  = LED_BRIGHTNESS_EQUATION( up_led_brightness_level );
    down_led_duty_cycle = LED_BRIGHTNESS_EQUATION( down_led_brightness_level );

    wiced_pwm_init ( SETPOINT_UP_LED, SETPOINT_LED_PWM_FREQ_HZ, up_led_duty_cycle );
    wiced_pwm_start( SETPOINT_UP_LED );
    wiced_pwm_init ( SETPOINT_DOWN_LED, SETPOINT_LED_PWM_FREQ_HZ, down_led_duty_cycle );
    wiced_pwm_start( SETPOINT_DOWN_LED );
}

/*
 * Increases temperature set point
 */
static void increase_setpoint( void )
{
    wiced_rtos_lock_mutex( &setpoint.mutex );
    setpoint.temperature += ( setpoint.temperature < MAX_SETPOINT ) ? SETPOINT_INCREMENT : 0.0f;
    adjust_setpoint_led_brightness( );
    wiced_rtos_unlock_mutex( &setpoint.mutex );
}

/*
 * Decreases temperature set point
 */
static void decrease_setpoint( void )
{
    wiced_rtos_lock_mutex( &setpoint.mutex );
    setpoint.temperature -= ( setpoint.temperature > MIN_SETPOINT ) ? SETPOINT_INCREMENT : 0.0f;
    adjust_setpoint_led_brightness( );
    wiced_rtos_unlock_mutex( &setpoint.mutex );
}

/*
 * Handles Button events
 */
static void setpoint_control_button_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state )
{
    if( event == BUTTON_CLICK_EVENT )
    {
        switch ( button->configuration->application_event )
        {
            case SETPOINT_UP_KEY_CODE:
                increase_setpoint( );
                break;

            case SETPOINT_DOWN_KEY_CODE:
                decrease_setpoint( );
                break;

            default:
                break;
        }
    }
    return;
}

/*
 * Updates temperature information in the web page
 */
static int32_t process_temperature_update( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data )
{
    wiced_iso8601_time_t* curr_time;
    float temp_celcius;
    float temp_fahrenheit;
    float setpoint_celcius = get_setpoint();
    float setpoint_fahrenheit;
    char  temp_char_buffer[6];
    int   temp_char_buffer_length;

    UNUSED_PARAMETER( url_path );
    UNUSED_PARAMETER( http_data );

    wiced_rtos_lock_mutex( &temperature_data.mutex );

    if ( temperature_data.temperature_reading_index == 0 )
    {
        curr_time = &temperature_data.temperature_readings[MAX_TEMPERATURE_READINGS - 1].timestamp;
    }
    else
    {
        curr_time = &temperature_data.temperature_readings[temperature_data.temperature_reading_index - 1].timestamp;
    }

    /* Update temperature report with the most recent temperature reading */
    temp_celcius        = (float) temperature_data.last_sample / 10.0f;
    temp_fahrenheit     = temp_celcius / 5.0f * 9.0f + 32.0f;
    setpoint_fahrenheit = setpoint_celcius / 5.0f * 9.0f + 32.0f;

    wiced_rtos_unlock_mutex( &temperature_data.mutex );

    /* Write the time */
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_time_start );
    wiced_http_response_stream_write( stream, curr_time->hour, sizeof(curr_time->hour)   +
                                                     sizeof(curr_time->colon1) +
                                                     sizeof(curr_time->minute) +
                                                     sizeof(curr_time->colon2) +
                                                     sizeof(curr_time->second) );
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_time_end );

    /* Write the date */
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_date_start );
    wiced_http_response_stream_write(stream, curr_time->year, sizeof(curr_time->year)  +
                                                    sizeof(curr_time->dash1) +
                                                    sizeof(curr_time->month) +
                                                    sizeof(curr_time->dash2) +
                                                    sizeof(curr_time->day) );

    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_date_end );

    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_temp_start );
    temp_char_buffer_length = sprintf(temp_char_buffer, "%.1f", temp_celcius);
    wiced_http_response_stream_write(stream, temp_char_buffer, temp_char_buffer_length );
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_temp_mid );
    temp_char_buffer_length = sprintf(temp_char_buffer, "%.1f", temp_fahrenheit);
    wiced_http_response_stream_write(stream, temp_char_buffer, temp_char_buffer_length );
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_temp_end );

    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_set_start );
    temp_char_buffer_length = sprintf(temp_char_buffer, "%.1f", setpoint_celcius);
    wiced_http_response_stream_write(stream, temp_char_buffer, temp_char_buffer_length );
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_set_mid );
    temp_char_buffer_length = sprintf(temp_char_buffer, "%.1f", setpoint_fahrenheit);
    wiced_http_response_stream_write(stream, temp_char_buffer, temp_char_buffer_length );
    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_set_end );

    wiced_http_response_stream_write_resource( stream, &resources_apps_DIR_temp_control_DIR_data_html_page_end );

    return 0;
}

/*
 * Handles increase temperature set point button from web page
 */
static int32_t process_temperature_up( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data )
{
    UNUSED_PARAMETER( url_path );
    UNUSED_PARAMETER( url_parameters );
    UNUSED_PARAMETER( arg );
    UNUSED_PARAMETER( http_data );
    increase_setpoint( );
    return 0;
}

/*
 * Handles decrease temperature set point button from web page
 */
static int32_t process_temperature_down( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data )
{
    UNUSED_PARAMETER( url_path );
    UNUSED_PARAMETER( url_parameters );
    UNUSED_PARAMETER( arg );
    UNUSED_PARAMETER( http_data );
    decrease_setpoint( );
    return 0;
}
