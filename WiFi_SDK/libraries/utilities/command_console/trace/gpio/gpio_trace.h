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

#ifndef GPIO_TRACE_H_
#define GPIO_TRACE_H_

#include "../trace.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef TRACE_ENABLE_GPIO
#define TRACE_T_GPIO                                                \
    {                                                               \
            (char*) "gpio",                                         \
            gpio_trace_start_trace,                                 \
            gpio_trace_stop_trace,                                  \
            NULL,                                                   \
            NULL,                                                   \
            gpio_trace_task_hook,                                   \
            gpio_trace_tick_hook,                                   \
            (trace_process_t []) {                                  \
                    { NULL, NULL, NULL }                            \
            }                                                       \
    },
#else
#define TRACE_T_GPIO
#endif /* TRACE_ENABLE_GPIO */

/******************************************************
 *        Hook functions
 ******************************************************/
void gpio_trace_task_hook( TRACE_TASK_HOOK_SIGNATURE );
void gpio_trace_tick_hook( TRACE_TICK_HOOK_SIGNATURE );


/******************************************************
 *        API functions to start/pause/end tracing
 ******************************************************/
void gpio_trace_start_trace( trace_flush_function_t flush_f );
void gpio_trace_stop_trace( void );
void gpio_trace_cleanup_trace( void );

/******************************************************
 *        Configuration
 ******************************************************/
/**
 * The TCB number is encoded in the output.
 */
#define TRACE_OUTPUT_TYPE_ENCODED   (1)

/**
 * Each task occupies a separate GPIO bit. This can makes results easier to see
 * immediately, but drastically reduces the number of tasks that can be output.
 */
#define TRACE_OUTPUT_TYPE_ONEBIT    (2)

#define TRACE_OUTPUT_TYPE           TRACE_OUTPUT_TYPE_ENCODED

/******************************************************
 *        Platform specific
 ******************************************************/
struct breakout_pins_setting_t;
/** Get the pin number of a specific breakout pin. */
inline uint16_t breakout_pins_get_pin_number( struct breakout_pins_setting_t * );
/** Initialize a specific breakout pin. Returns 1 if the specified pin was able to be used for output, otherwise 0. */
inline int breakout_pins_gpio_init( struct breakout_pins_setting_t * );
/** Sets the specified breakout pin. */
inline void breakout_pins_gpio_setbits( struct breakout_pins_setting_t * );
/** Resets the specified breakout pin. */
inline void breakout_pins_gpio_resetbits( struct breakout_pins_setting_t * );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* GPIO_TRACE_H_ */
