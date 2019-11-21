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
 * If you need to add LED support to your platform, please see led_service.h
 */
#include <string.h>
#include "wiced.h"
#include "led_service.h"
#include "wiced_rtos.h"
#include "platform.h"
#include "platform_mcu_peripheral.h"
#include "wiced_platform.h"

#ifndef GPIO_LED_NOT_SUPPORTED

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LED_WORKER_STACK_SIZE               ( 1024 )
#define LED_WORKER_QUEUE_SIZE               (    4 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    wiced_worker_thread_t           worker_thread;
    wiced_bool_t                    worker_thread_started;

    wiced_timer_t                   led_timer;          /* timer to send asynch message to thread */
    wiced_bool_t                    led_timer_inited;

    wiced_led_service_parameters_t  params;
    int                             curr_state;         /* current state in the LED state table                         */
} led_service_context_t;
/******************************************************
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t wiced_led_worker_thread_callback( void *arg);
static void led_timer_event_handler( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static led_service_context_t* led_ctx;

/* Predefined patterns */
static wiced_led_service_state_t  led_pattern_slow_led1[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF }, 500 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 200 },
};

/* fast green blink */
static wiced_led_service_state_t  led_pattern_fast_led1[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },

};

static wiced_led_service_state_t  led_pattern_one_led1[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_two_led1[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_three_led1[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },
    { {WICED_LED_ON,  WICED_LED_OFF }, 250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_led1_sos[] =
{
    { {WICED_LED_ON,  WICED_LED_OFF},  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_ON,  WICED_LED_OFF},  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  500 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_ON,  WICED_LED_OFF},  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_ON,  WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_slow_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 500 },
    { {WICED_LED_OFF, WICED_LED_OFF}, 200 },
};

static wiced_led_service_state_t  led_pattern_fast_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
};

static wiced_led_service_state_t  led_pattern_one_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_two_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_three_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_led2_sos[] =
{
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_ON },  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  500 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  250 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};

static wiced_led_service_state_t  led_pattern_slow_both[] =
{
    { {WICED_LED_ON,  WICED_LED_ON  }, 500 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 200 },

};

/* fast green blink */
static wiced_led_service_state_t  led_pattern_fast_both[] =
{
    { {WICED_LED_ON,  WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },

};

static wiced_led_service_state_t  led_pattern_slow_led1_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 500 },
    { {WICED_LED_ON,  WICED_LED_OFF}, 500 },
};

static wiced_led_service_state_t  led_pattern_fast_led1_led2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 250 },
    { {WICED_LED_ON,  WICED_LED_OFF}, 250 },
};

static const wiced_led_service_parameters_t predefined_led_flash_patterns[] =
{
        [ WICED_LED_PATTERN_LED_1_SLOW_FLASH ] =
        {
            .led_states = led_pattern_slow_led1,
            .num_states = sizeof(led_pattern_slow_led1) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_FAST_FLASH ] =
        {
            .led_states = led_pattern_fast_led1,
            .num_states = sizeof(led_pattern_fast_led1) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_ONE_FLASH ] =
        {
            .led_states = led_pattern_one_led1,
            .num_states = sizeof(led_pattern_one_led1) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_TWO_FLASH ] =
        {
            .led_states = led_pattern_two_led1,
            .num_states = sizeof(led_pattern_two_led1) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_THREE_FLASH ] =
        {
            .led_states = led_pattern_three_led1,
            .num_states = sizeof(led_pattern_three_led1) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_SOS ] =
        {
            .led_states = led_pattern_led1_sos,
            .num_states = sizeof(led_pattern_led1_sos) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_SLOW_FLASH ] =
        {
            .led_states = led_pattern_slow_led2,
            .num_states = sizeof(led_pattern_slow_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_FAST_FLASH ] =
        {
            .led_states = led_pattern_fast_led2,
            .num_states = sizeof(led_pattern_fast_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_ONE_FLASH ] =
        {
            .led_states = led_pattern_one_led2,
            .num_states = sizeof(led_pattern_one_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_TWO_FLASH ] =
        {
            .led_states = led_pattern_two_led2,
            .num_states = sizeof(led_pattern_two_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_THREE_FLASH ] =
        {
            .led_states = led_pattern_three_led2,
            .num_states = sizeof(led_pattern_three_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_2_SOS ] =
        {
            .led_states = led_pattern_led2_sos,
            .num_states = sizeof(led_pattern_led2_sos) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_LED_2_SLOW_FLASH ] =
        {
            .led_states = led_pattern_slow_both,
            .num_states = sizeof(led_pattern_slow_both) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_LED_1_LED_2_FAST_FLASH ] =
        {
            .led_states = led_pattern_fast_both,
            .num_states = sizeof(led_pattern_fast_both) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_ALTERNATE_SLOW_FLASH ] =
        {
            .led_states = led_pattern_slow_led1_led2,
            .num_states = sizeof(led_pattern_slow_led1_led2) / sizeof(wiced_led_service_state_t)
        },
        [ WICED_LED_PATTERN_ALTERNATE_FAST_FLASH ] =
        {
            .led_states = led_pattern_fast_led1_led2,
            .num_states = sizeof(led_pattern_fast_led1_led2) / sizeof(wiced_led_service_state_t)
        },
};
/******************************************************
 *               Internal Function Definitions
 ******************************************************/

/******************************************************
 *               Timer Event Handler
 ******************************************************/
static void wiced_led_stop_timer(led_service_context_t*  led_ctx)
{
    if (led_ctx->led_timer_inited == WICED_TRUE)
    {
        wiced_rtos_deinit_timer( &led_ctx->led_timer );
        led_ctx->led_timer_inited = WICED_FALSE;
    }
}

static void wiced_led_start_timer(led_service_context_t*  led_ctx)
{
    wiced_led_stop_timer(led_ctx);
    if (wiced_rtos_init_timer( &led_ctx->led_timer, led_ctx->params.led_states[led_ctx->curr_state].state_length_ms,
                           led_timer_event_handler, (void*) led_ctx ) == WICED_SUCCESS)
    {
        led_ctx->led_timer_inited = WICED_TRUE;
        wiced_rtos_start_timer( &led_ctx->led_timer );
    }
}

static wiced_result_t wiced_led_worker_thread_callback( void *arg)
{
    led_service_context_t*  led_ctx = (led_service_context_t*)arg;
    int                     led;

    if (led_ctx == NULL)
    {
        return WICED_BADARG;
    }

    /* move on to next state */
    led_ctx->curr_state++;
    if (led_ctx->curr_state >= led_ctx->params.num_states)
    {
        /* reset to state 0 */
        led_ctx->curr_state = 0;
    }

    /* turn on/off LEDs */
    for (led = 0; led < WICED_LED_INDEX_MAX; led++)
    {
        wiced_led_set_state(led, led_ctx->params.led_states[led_ctx->curr_state].off_on[led] );
    }

    /* restart timer (we may have a new value for the timer) */
    wiced_led_start_timer(led_ctx);

    return WICED_SUCCESS;
}

static void led_timer_event_handler( void* arg )
{
    led_service_context_t*  led_ctx = (led_service_context_t*)arg;

    if ((led_ctx != NULL) && (led_ctx->worker_thread_started == WICED_TRUE))
    {
        /* send async event to worker thread */
        wiced_rtos_send_asynchronous_event( &led_ctx->worker_thread, wiced_led_worker_thread_callback, led_ctx );
    }
}

/******************************************************
 *               External Function Definitions
 ******************************************************/

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
 *          WICED_ERROR     - unable to start worker thread or timer
 *
 */
wiced_result_t wiced_led_service_init( void )
{
    int             led;
    wiced_result_t  result;

    if (led_ctx != NULL)
    {
        return WICED_ERROR;
    }

    led_ctx = (led_service_context_t*)calloc_named("led_service", 1, sizeof(led_service_context_t));
    if (led_ctx == NULL)
    {
        return WICED_ERROR;
    }
    memset(led_ctx, 0x00, sizeof(led_service_context_t));

    /* start the worker thread */
    result = wiced_rtos_create_worker_thread( &led_ctx->worker_thread, WICED_DEFAULT_WORKER_PRIORITY, LED_WORKER_STACK_SIZE, LED_WORKER_QUEUE_SIZE );
    if (result != WICED_SUCCESS)
    {
        goto _led_service_init_fail;
    }
    led_ctx->worker_thread_started = WICED_TRUE;

    /* Turn off LEDs */
    for (led = 0; led < WICED_LED_INDEX_MAX; led++)
    {
        wiced_led_set_state(led, WICED_LED_OFF );
    }
    return WICED_SUCCESS;

_led_service_init_fail:

    if (led_ctx != NULL)
    {
        free(led_ctx);
    }
    led_ctx = NULL;

    return WICED_ERROR;
}

/**
 * De-Initialize LED service
 *
 * Deletes the worker thread and turns off the LEDs
 * After calling wiced_led_service_deinit(), you can
 * still turn on individual LEDs using wiced_led_set_state()
 *
 */
void wiced_led_service_deinit( void )
{
    if (led_ctx == NULL)
    {
        return;
    }

    wiced_led_service_stop();

    if (led_ctx->worker_thread_started == WICED_TRUE)
    {
        wiced_rtos_delete_worker_thread( &led_ctx->worker_thread );
        led_ctx->worker_thread_started = WICED_FALSE;
    }
    free(led_ctx);
    led_ctx = NULL;
}

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
wiced_result_t wiced_led_service_start(const wiced_led_service_parameters_t* params)
{
    int             led;

    if (params == NULL)
    {
        return WICED_BADARG;
    }

    /* Do we have an LED context ? */
    if (led_ctx == NULL)
    {
        return WICED_ERROR;
    }

    /* stop any currently running timer */
    wiced_led_stop_timer(led_ctx);

    /* copy over parameters */
    memcpy(&led_ctx->params, params, sizeof(wiced_led_service_parameters_t));

    /* turn on/off LEDs for first state */
    led_ctx->curr_state = 0;
    for (led = 0; led < WICED_LED_INDEX_MAX; led++)
    {
        wiced_led_set_state(led, led_ctx->params.led_states[led_ctx->curr_state].off_on[led] );
    }

    /* initialize and start the timer */
    wiced_led_start_timer(led_ctx);

    return WICED_SUCCESS;

}

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
wiced_result_t wiced_led_service_start_pattern(wiced_led_pattern_t led_pattern)
{
    if (led_pattern < WICED_LED_PATTERN_MAX)
    {
        return wiced_led_service_start( &predefined_led_flash_patterns[led_pattern]);
    }

    return WICED_BADARG;
}

/**
 * Stop the LED Service playback
 *
 * NOTES:   Sets all LEDs off
 *          You must call wiced_led_service_deinit() if you want to free system resources
 *
 * @return  WICED_SUCCESS
 */
wiced_result_t wiced_led_service_stop( void )
{
    int led;

    if (led_ctx != NULL)
    {
        wiced_led_stop_timer(led_ctx);
    }

    /* Turn off LEDs */
    for (led = 0; led < WICED_LED_INDEX_MAX; led++)
    {
        wiced_led_set_state(led, WICED_LED_OFF );
    }

    return WICED_SUCCESS;
}

#else
/* dummy functions for when there are no LEDs defined */
wiced_result_t wiced_led_service_init( void )
{
    return WICED_ERROR;
}
void wiced_led_service_deinit( void )
{
}
wiced_result_t wiced_led_service_start(const wiced_led_service_parameters_t* params)
{
    return WICED_ERROR;
}
wiced_result_t wiced_led_service_start_pattern(wiced_led_pattern_t led_pattern)
{
    return WICED_ERROR;
}
wiced_result_t wiced_led_service_stop( void )
{
    return WICED_ERROR;
}
#endif  /* ifndef GPIO_LED_NOT_SUPPORTED */
