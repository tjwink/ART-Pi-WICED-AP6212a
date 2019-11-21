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
#include "wiced.h"

/* Default to CapSense platform API. */
#if !defined( USE_CAPSENSE_COMPONENT_API )
#define USE_CAPSENSE_COMPONENT_API 0
#endif

#if ( USE_CAPSENSE_COMPONENT_API != 0 )
#define CAPSENSE_LOOP       capsense_component_api_loop
#else
#define CAPSENSE_LOOP       capsense_platform_api_loop
#endif /* #if ( USE_CAPSENSE_COMPONENT_API ) */

#define LED_PWM_FREG_HZ 2000

#define RED_LED_PWM     (WICED_PWM_1)
#define GREEN_LED_PWM   (WICED_PWM_2)
#define BLUE_LED_PWM    (WICED_PWM_3)

#define LIGHT_RED_LED   ( 0 )
#define LIGHT_GREEN_LED ( 1 )
#define LIGHT_BLUE_LED  ( 2 )

typedef enum
{
  INPUT_EVENT_NONE    = 0,
  INPUT_EVENT_BUTTON0 = 1,
  INPUT_EVENT_BUTTON1 = 2,
  INPUT_EVENT_SLIDER  = 3
} input_event_t;

static void capsense_action ( input_event_t input_event, uint8_t sliderpos )
{
    static uint32_t  duty_cycle  = 0;
    static uint32_t  cur_led     = 0;       /* 0 => Red, 1 => Green. 2=> Blue */
    uint32_t         prev_led    = 0;       /* 0 => Red, 1 => Green. 2=> Blue */
    uint8_t          led_pwm     = 0;       /* 0 => Red, 1 => Green. 2=> Blue */

    if ( input_event == INPUT_EVENT_NONE )
    {
        return;
    }

    prev_led = cur_led;

    if ( ( input_event == INPUT_EVENT_BUTTON0 ) || ( input_event == INPUT_EVENT_SLIDER ) )
    {
        if ( input_event == INPUT_EVENT_BUTTON0 )
        {
             switch ( prev_led )
             {
                 case LIGHT_RED_LED:
                      led_pwm = RED_LED_PWM;
                      break;
                 case LIGHT_GREEN_LED:
                      led_pwm = GREEN_LED_PWM;
                      break;
                 case LIGHT_BLUE_LED:
                      led_pwm = BLUE_LED_PWM;
                      break;
                 default:
                      led_pwm = RED_LED_PWM;
                      break;
             }
             wiced_pwm_stop( led_pwm );

             cur_led++;
             if ( cur_led > LIGHT_BLUE_LED )
             {
                 cur_led = LIGHT_RED_LED;
             }
         }
         else if ( input_event == INPUT_EVENT_SLIDER )
         {
             duty_cycle = sliderpos;
         }

         switch ( cur_led )
         {
             case LIGHT_RED_LED:
                  led_pwm = RED_LED_PWM;
                  break;
             case LIGHT_GREEN_LED:
                  led_pwm = GREEN_LED_PWM;
                  break;
             case LIGHT_BLUE_LED:
                  led_pwm = BLUE_LED_PWM;
                  break;
             default:
                  led_pwm = RED_LED_PWM;
                  break;
         }
         wiced_pwm_init( led_pwm, LED_PWM_FREG_HZ, duty_cycle );
         wiced_pwm_start( led_pwm );
    }
    else if ( input_event == INPUT_EVENT_BUTTON1 )
    {
         switch ( cur_led )
         {
             case LIGHT_RED_LED:
                  led_pwm = RED_LED_PWM;
                  break;
             case LIGHT_GREEN_LED:
                  led_pwm = GREEN_LED_PWM;
                  break;
             case LIGHT_BLUE_LED:
                  led_pwm = BLUE_LED_PWM;
                  break;
             default:
                  led_pwm = RED_LED_PWM;
                  break;
         }
         wiced_pwm_stop( led_pwm );
    }
}

void capsense_component_api_loop(void)
{
    uint8_t      currSliderPos = 0;
    uint8_t      input_event;     /* Input event flag */

    /*Start CapSense*/
    CapSense_Start();

    for(;;)
    {
        /* Do this only when the CapSense isn't busy with a previous operation*/
        if (CapSense_NOT_BUSY == CapSense_IsBusy())
        {
            input_event = INPUT_EVENT_NONE;

            /* Process data from all the sensors and find out the touch
               information */
            CapSense_ProcessAllWidgets();

            /* Check if button 0 is touched */
            if (CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
            {
                /* Print Button 0 touch detection */
                WPRINT_APP_INFO( ( "Button 0 Pressed \n\r" ) );
                input_event = INPUT_EVENT_BUTTON0;
            }
            /* Check if button 1 is touched */
            else if (CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
            {
                /* Print button 1 touch detection */
                WPRINT_APP_INFO( ( "Button 1 Pressed \n\r" ) );
                input_event = INPUT_EVENT_BUTTON1;
            }
            /* Check if the slider is touched */
            else if (CapSense_IsWidgetActive(CapSense_LINEARSLIDER0_WDGT_ID))
            {
                /* Read Current slider position*/
                currSliderPos = CapSense_GetCentroidPos
                                (CapSense_LINEARSLIDER0_WDGT_ID);

                input_event = INPUT_EVENT_SLIDER;
                WPRINT_APP_INFO( ( "Slider Position: %d \n\r", currSliderPos ) );
            }

            capsense_action( input_event, currSliderPos );

            /* Start CapSense scan */
            CapSense_ScanAllWidgets();
        }
    }
}

void platform_capsense_event_callback( const platform_capsense_widget_t* widget, platform_widget_event_id_t event_id, const void* void_data)
{
    uint8_t     input_event = INPUT_EVENT_NONE;       /* Input event flag */
    uint32_t    duty_cycle  = 0;

    switch ( event_id )
    {
        case PLATFORM_WIDGET_TRACK_BUTTON_EVENT_ID:
        {
            const platform_track_button_event_data_t* data = (const platform_track_button_event_data_t*)void_data;
            WPRINT_APP_INFO( ( "TRACK_BUTTON id=%u active=%d\r\n", (unsigned int)widget->id, data->base.is_active ) );
            if ( (unsigned int) widget->id == CapSense_BUTTON0_WDGT_ID )
            {
                if ( data->base.is_active )
                {
                    input_event = INPUT_EVENT_BUTTON0;
                }
            }
            else if ( (unsigned int) widget->id == CapSense_BUTTON1_WDGT_ID )
            {
                if ( data->base.is_active )
                {
                    input_event = INPUT_EVENT_BUTTON1;
                }
            }
            break;
        }

        case PLATFORM_WIDGET_TRACK_LINEAR_SLIDER_EVENT_ID:
        {
            const platform_track_linear_slider_event_data_t* data = (const platform_track_linear_slider_event_data_t*)void_data;
            WPRINT_APP_INFO( ( "TRACK_LINEAR_SLIDER id=%u active=%d pos=%u\r\n", (unsigned int)widget->id, data->base.is_active, (unsigned int)data->position ) );
            input_event = INPUT_EVENT_SLIDER;
            duty_cycle = (unsigned int)data->position;
            break;
        }

        default:
            WPRINT_APP_INFO( ( "Unknown capsense id=%u event=%x\r\n", (unsigned int)widget->id, (unsigned int)event_id) );
            break;
    }

    capsense_action( input_event, duty_cycle );
}

void capsense_platform_api_loop( void )
{
    uint32_t            idx;
    wiced_semaphore_t   semaphore;
    extern const platform_capsense_widget_t platform_capsense_widget[CapSense_TOTAL_WIDGETS];
    extern const platform_capsense_interface_t platform_capsense_interface;

    wiced_rtos_init_semaphore( &semaphore );

    platform_capsense_init( platform_capsense_interface.driver, platform_capsense_interface.data );
    platform_capsense_register_event_callback( platform_capsense_event_callback );

    for (idx = 0; idx < ARRAY_SIZE( platform_capsense_widget ); idx++ )
    {
        platform_capsense_enable( &platform_capsense_widget[ idx ], WICED_TRUE );
    }

    /* Wait indefinitely. */
    wiced_rtos_get_semaphore( &semaphore, WICED_WAIT_FOREVER );
}

void application_start()
{
    wiced_core_init();

    /* Remove buffering from all std streams */
    setvbuf( stdin,  NULL, _IONBF, 0 );
    setvbuf( stdout, NULL, _IONBF, 0 );
    setvbuf( stderr, NULL, _IONBF, 0 );

    CAPSENSE_LOOP();
}

/* [] END OF FILE */
