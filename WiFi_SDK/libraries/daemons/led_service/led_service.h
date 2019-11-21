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

/*
 * LED service daemon
 *
 *  1) Define 2 LEDs, their GPIOs in the header:
 *
 * in <platform>/platform.h:
 *    (see platforms/BCM943907WAE_1/platform.h)
 *
 * Remove:
 * #define GPIO_LED_NOT_SUPPORTED
 *
 * Add:
 * #define PLATFORM_LED_COUNT               ( 2 )
 * #define WICED_LED1           ( WICED_GPIO_xx )
 * #define WICED_LED2           ( WICED_GPIO_xx )
 *
 * 2) Add table and code in <platform>/platform.c:
 *    (see platforms/BCM943907WAE_1/platform.c)
 *
 * Add this table:
 * const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
 * {
 *     [WICED_LED_INDEX_1] = WICED_LED1,
 *     [WICED_LED_INDEX_2] = WICED_LED2,
 * };
 *
 * Add this code (pay attention to platform_led_set_state() to be your active state is correct):
 *
 *  // LEDs on this platform are active HIGH
 * platform_result_t platform_led_set_state(int led_index, int off_on )
 * {
 *     if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
 *     {
 *         switch (off_on)
 *         {
 *             case WICED_LED_OFF:
 *                 platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
 *                 break;
 *             case WICED_LED_ON:
 *                 platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
 *                 break;
 *         }
 *         return PLATFORM_SUCCESS;
 *     }
 *     return PLATFORM_BADARG;
 * }
 *
 * void platform_led_init( void )
 * {
 *     // Initialise LEDs and turn off by default
 *     platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
 *     platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
 *     platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
 *     platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);
 *  }
 *
 *  Modify platform_init_external_devices() to call platform_led_init()
 *
 *
 * 3) Use pre-defined patterns, or define tables of LED flashing patterns
 *
 * See the pre-defined patterns for examples in libraries/daemons/led_service/led_service.c.
 *     and apps/snip/led_example/led_example.c
 *
 * Example: Use a pre-defined pattern to blinking LED 1 slowly
 *
 * {
 *  wiced_led_service_init();
 *
 *  wiced_led_service_start_pattern( WICED_LED_PATTERN_LED_1_SLOW_FLASH );
 *
 *  wiced_rtos_delay_milliseconds(10 * 1000);  // let it run 10 seconds
 *
 *  wiced_led_service_stop();
 * }
 *
 */

#include "platform.h"
#include "wiced_platform.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/


/*****************************************************************************/
/** @addtogroup LED       LED
 *  @ingroup platform
 *
 * API for programmable on/off sequences for visual indications using LEDs
 *
 *
 *  @{
 */
/*****************************************************************************/


/**
 *  Predefined LED Flash patterns
 */
typedef enum
{
    WICED_LED_PATTERN_LED_1_SLOW_FLASH = 0,     /**< Slow flash - PLATFORM_LED_1                            */
    WICED_LED_PATTERN_LED_1_FAST_FLASH,         /**< Fast flash - PLATFORM_LED_1                            */
    WICED_LED_PATTERN_LED_1_ONE_FLASH,          /**< One fast flash then 1 second off - PLATFORM_LED_1      */
    WICED_LED_PATTERN_LED_1_TWO_FLASH,          /**< Two fast flashes then 1 second off - PLATFORM_LED_1    */
    WICED_LED_PATTERN_LED_1_THREE_FLASH,        /**< Three fast flashes then 1 second off - PLATFORM_LED_1  */
    WICED_LED_PATTERN_LED_1_SOS,                /**< Morse code SOS - PLATFORM_LED_1                        */
    WICED_LED_PATTERN_LED_2_SLOW_FLASH,         /**< Slow flash - PLATFORM_LED_2                            */
    WICED_LED_PATTERN_LED_2_FAST_FLASH,         /**< Fast flash - PLATFORM_LED_2                            */
    WICED_LED_PATTERN_LED_2_ONE_FLASH,          /**< One fast flash then 1 second off - PLATFORM_LED_2      */
    WICED_LED_PATTERN_LED_2_TWO_FLASH,          /**< Two fast flashes then 1 second off - PLATFORM_LED_2    */
    WICED_LED_PATTERN_LED_2_THREE_FLASH,        /**< Three fast flashes then 1 second off - PLATFORM_LED_2  */
    WICED_LED_PATTERN_LED_2_SOS,                /**< Morse code SOS - PLATFORM_LED_2                        */

    WICED_LED_PATTERN_LED_1_LED_2_SLOW_FLASH,   /**< Slow Flashes both Platform_LED_1 and PLATFORM_LED_2    */
    WICED_LED_PATTERN_LED_1_LED_2_FAST_FLASH,   /**< Fast flashes both Platform_LED_1 and PLATFORM_LED_2    */
    WICED_LED_PATTERN_ALTERNATE_SLOW_FLASH,     /**< Slow Flashes Platform_LED_1 then PLATFORM_LED_2        */
    WICED_LED_PATTERN_ALTERNATE_FAST_FLASH,     /**< Fast flashes Platform_LED_1 then PLATFORM_LED_2        */

    WICED_LED_PATTERN_MAX                       /**< Denotes the end of the list */
} wiced_led_pattern_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/**
 *  Structure to define one state of a flashing LED sequence
 */
typedef struct
{
        wiced_led_state_t               off_on[WICED_LED_INDEX_MAX];   /**< One entry per LED, WICED_LED_OFF or WICED_LED_ON */
        uint32_t                        state_length_ms;               /**< How long to display this LED state */
} wiced_led_service_state_t;

typedef struct
{
        uint8_t                         num_states;                 /**< number of states in the table        */
        wiced_led_service_state_t*      led_states;                 /**< array of states, one per num_states  */
} wiced_led_service_parameters_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
 * Initialize LED GPIOs for this platform
 *
 * Initializes the platform's LEDs for use and turns them off
 * To turn on/off individual LEDs use wiced_led_set_state()
 *
 * To use patterns, you must use wiced_led_service_init().
 *
 */
void wiced_led_init( void );

/**
 * Set the on/off state for the LED
 *
 * NOTE: call wiced_led_init() or wiced_led_service_init() before calling this function.
 *
 * @param      led     : LED enumeration for the platform
 * @param      off_on  : off / on state for the LED
 *
 * @return      WICED_SUCCESS
 *              WICED_ERROR
 */
wiced_result_t wiced_led_set_state(wiced_led_index_t led, wiced_led_state_t off_on );

/**
 * Initialize LED Service
 *
 * Initializes the platform's LEDs for use and turns them off
 *
 * Starts a worker thread for use in flashing LED patterns
 * Start a pattern using wiced_led_start() or wiced_led_start_pattern()
 * To turn on individual LEDs use wiced_led_set_state()
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *
 */
wiced_result_t wiced_led_service_init( void );

/**
 * De-Initialize LED service and free system resources
 *
 * Calls wiced_led_service_stop().
 * After calling wiced_led_service_deinit(), you can
 * still turn on individual LEDs using wiced_led_set_state()
 *
 */
void wiced_led_service_deinit( void );

/**
 * Start the LED service
 *
 * NOTE: This is non-blocking, and will continue until
 *       user calls wiced_led_service_stop() or
 *       another call to wiced_led_service_start() or wiced_led_service_start_pattern()
 *       which will cancel the previous pattern
 *
 * @param params      : parameter structure
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_led_service_start(const wiced_led_service_parameters_t* params);

/**
 * Start the LED service using a predefined pattern
 *
 * NOTE: This is non-blocking, and will continue until
 *       user calls wiced_led_service_stop() or
 *       another call to wiced_led_service_start() or wiced_led_service_start_pattern()
 *       which will cancel the previous pattern
 *
 * @param pattern      : which pattern to display
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_led_service_start_pattern(wiced_led_pattern_t led_pattern);

/**
 * Stop the LED Service playback
 *
 * Sets all LEDs off
 * You must call wiced_led_service_deinit() if you want to free system resources
 *
 * @return  WICED_SUCCESS
 *
 */
wiced_result_t wiced_led_service_stop( void );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif

