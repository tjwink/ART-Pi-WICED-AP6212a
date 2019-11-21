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

#include "xively.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond */
/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define THERMISTOR_ADC                (WICED_THERMISTOR_JOINS_ADC)  /* Handle for Thermistor                                                                      */
#define SETPOINT_UP_KEY_GPIO          (WICED_BUTTON1)               /* Handle for Set Point UP key (button) GPIO                                                  */
#define SETPOINT_DOWN_KEY_GPIO        (WICED_BUTTON2)               /* Handle for Set Point DOWN key (button) GPIO                                                */
#define SETPOINT_UP_KEY_CODE          (1)                           /* Set Point UP key code                                                                      */
#define SETPOINT_DOWN_KEY_CODE        (2)                           /* Set Point DOWN key code                                                                    */
#define SETPOINT_UP_LED               (WICED_LED1_JOINS_PWM)        /* Handle for Set Point LED 1                                                                 */
#define SETPOINT_DOWN_LED             (WICED_LED2_JOINS_PWM)        /* Handle for Set Point LED 2                                                                 */
#define SETPOINT_LED_PWM_FREQ_HZ      (2000)                        /* Set Point LED pulse-width-modulation (PWM) frequency in Hz                                 */
#define DEFAULT_SETPOINT              (25.0f)                       /* Default Set Point (in degree C) when the application starts                                */
#define MAX_SETPOINT                  (35.0f)                       /* Maximum Set Point in degree C                                                              */
#define MIN_SETPOINT                  (15.0f)                       /* Minimum Set Point in degree C                                                              */
#define SETPOINT_INCREMENT            (1.0f)                        /* Set point increment in degree C                                                            */
#define MAX_TEMPERATURE_READINGS      (64)                          /* Maximum records of temperature readings to be fed to Xively. This needs to be an order of 2  */

#define LED_BRIGHTNESS_CONST_A        (0.08f)                       /* Set Point LED brightness equation: a * b ^ (brightness_level + c) ...                      */
#define LED_BRIGHTNESS_CONST_B        (1.75f)                       /* Constants have been set for the equations to produce distinctive brightness levels         */
#define LED_BRIGHTNESS_CONST_C        (2.00f)
#define LED_BRIGHTNESS_EQUATION(level) (LED_BRIGHTNESS_CONST_A * pow(LED_BRIGHTNESS_CONST_B, level + LED_BRIGHTNESS_CONST_C))


/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CONFIGURE_NETWORK_USING_WPS,
    CONFIGURE_NETWORK_USING_AP_MODE,
    CONFIGURE_NETWORK_USING_P2P,
} network_configuration_method_t;

typedef enum
{
    TEMPERATURE_UP_BUTTON,
    TEMPERATURE_DOWN_BUTTON,
} application_button_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#pragma pack(1)
typedef struct
{
    wiced_iso8601_time_t timestamp;
    char                 sample[4];
} temperature_reading_t;

#pragma pack()

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** @endcond */
#ifdef __cplusplus
} /*extern "C" */
#endif

