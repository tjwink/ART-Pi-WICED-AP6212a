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
 * LED Example Application
 *
 * Features demonstrated
 *  - using daemons/led_service to turn LEDs on and off, display patterns
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 */

#include <stdlib.h>
#include "wiced.h"
#include "command_console.h"
#include "led_service.h"

#ifdef GPIO_LED_NOT_SUPPORTED
#error "Platform does not support LEDs"
#endif

#ifndef PLATFORM_LED_COUNT
#error "Please define # of leds on this platform in platforms/<platform>/platform.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define MAX_COMMAND_LENGTH                   (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH      (10)

#define LED_CONSOLE_COMMANDS \
    { (char*) "init",           led_console_command,    0, NULL, NULL, (char *)"", (char *)"Initialize LED Service" }, \
    { (char*) "deinit",         led_console_command,    0, NULL, NULL, (char *)"", (char *)"De-initialize LED Service" }, \
    { (char*) "on",             led_console_command,    1, NULL, NULL, (char *)"", (char *)"Turn LED on  ex. on 0" }, \
    { (char*) "off",            led_console_command,    1, NULL, NULL, (char *)"", (char *)"Turn LED off  ex. off 0" }, \
    { (char*) "start",          led_console_command,    1, NULL, NULL, (char *)"", (char *)"Start Application Pattern x ex. start 0" }, \
    { (char*) "pattern",        led_console_command,    1, NULL, NULL, (char *)"", (char *)"Start Predefined Pattern x ex. pattern 0" }, \
    { (char*) "stop",           led_console_command,    0, NULL, NULL, (char *)"", (char *)"Stop Pattern x ex. stop 0" }, \


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    LED_CONSOLE_CMD_INIT = 0,
    LED_CONSOLE_CMD_DEINIT,

    LED_CONSOLE_CMD_LED_ON,
    LED_CONSOLE_CMD_LED_OFF,
    LED_CONSOLE_CMD_START,
    LED_CONSOLE_CMD_PATTERN_START,
    LED_CONSOLE_CMD_STOP,

    LED_CONSOLE_CMD_MAX,
} LED_CONSOLE_CMDS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct cmd_lookup_s {
        char *cmd;
        uint32_t event;
} cmd_lookup_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
int led_console_command(int argc, char *argv[]);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static char led_command_buffer[MAX_COMMAND_LENGTH];
static char led_command_history_buffer[MAX_COMMAND_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t led_command_table[] = {
    LED_CONSOLE_COMMANDS
    CMD_TABLE_END
};

static cmd_lookup_t command_lookup[LED_CONSOLE_CMD_MAX] =
{
        { "init",       0 },
        { "deinit",     0 },
        { "on",         0 },
        { "off",        0 },
        { "start",      0 },
        { "pattern",    0 },
        { "stop",       0 },
};


typedef enum
{
    LED_TEST_PATTERN_SLOW_GREEN_RED = 0,
    LED_TEST_PATTERN_FAST_GREEN_RED,
    LED_TEST_PATTERN_RED_GREEN_1,
    LED_TEST_PATTERN_RED_GREEN_2,
    LED_TEST_PATTERN_RED_GREEN_3,
    LED_TEST_PATTERN_SOS,

    LED_TEST_MAX_PATTERNS
} led_test_patterns_t;



/* slow green blink */
static wiced_led_service_state_t  led_pattern_slow_green_red[] =
{
    { {WICED_LED_ON,  WICED_LED_ON  }, 500 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 200 },

};

/* fast green blink */
static wiced_led_service_state_t  led_pattern_fast_green_red[] =
{
    { {WICED_LED_ON,  WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF }, 100 },

};

static wiced_led_service_state_t  led_pattern_red_green_1[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 500 },
    { {WICED_LED_ON,  WICED_LED_OFF}, 200 },
};

static wiced_led_service_state_t  led_pattern_red_green_2[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 200 },
    { {WICED_LED_ON,  WICED_LED_OFF}, 500 },
};

static wiced_led_service_state_t  led_pattern_red_green_3[] =
{
    { {WICED_LED_OFF, WICED_LED_ON }, 200 },
    { {WICED_LED_ON,  WICED_LED_ON }, 200 },
    { {WICED_LED_OFF, WICED_LED_ON }, 200 },
    { {WICED_LED_ON,  WICED_LED_OFF}, 200 },
};

static wiced_led_service_state_t  led_pattern_sos[] =
{
    { {WICED_LED_OFF, WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  200 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_ON },  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  500 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  500 },

    { {WICED_LED_OFF, WICED_LED_OFF},  250 },

    { {WICED_LED_OFF, WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  200 },
    { {WICED_LED_OFF, WICED_LED_OFF},  100 },
    { {WICED_LED_OFF, WICED_LED_ON },  200 },

    { {WICED_LED_OFF, WICED_LED_OFF}, 1000 },
};


static wiced_led_service_parameters_t led_test_patterns[LED_TEST_MAX_PATTERNS] =
{
    [ LED_TEST_PATTERN_SLOW_GREEN_RED ] =
    {
        .led_states = led_pattern_slow_green_red,
        .num_states = sizeof(led_pattern_slow_green_red) / sizeof(wiced_led_service_state_t)
    },

    [ LED_TEST_PATTERN_FAST_GREEN_RED ] =
    {
        .led_states = led_pattern_fast_green_red,
        .num_states = sizeof(led_pattern_fast_green_red) / sizeof(wiced_led_service_state_t)
    },

    [ LED_TEST_PATTERN_RED_GREEN_1 ] =
    {
        .led_states = led_pattern_red_green_1,
        .num_states = sizeof(led_pattern_red_green_1) / sizeof(wiced_led_service_state_t)
    },

    [ LED_TEST_PATTERN_RED_GREEN_2 ] =
    {
        .led_states = led_pattern_red_green_2,
        .num_states = sizeof(led_pattern_red_green_2) / sizeof(wiced_led_service_state_t)
    },

    [ LED_TEST_PATTERN_RED_GREEN_3 ] =
    {
        .led_states = led_pattern_red_green_3,
        .num_states = sizeof(led_pattern_red_green_3) / sizeof(wiced_led_service_state_t)
    },

    [ LED_TEST_PATTERN_SOS ] =
    {
        .led_states = led_pattern_sos,
        .num_states = sizeof(led_pattern_sos) / sizeof(wiced_led_service_state_t)
    },
};




/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t  result;

    /* Initialize the device */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return;
    }

    /* LED 1 on */
    wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);

    /*
     * Create the command console.
     */

    result = command_console_init(STDIO_UART, sizeof(led_command_buffer), led_command_buffer, CONSOLE_COMMAND_HISTORY_LENGTH, led_command_history_buffer, " ");
    if (result != WICED_SUCCESS)
    {
        printf("Error starting the command console\r\n");
        return;
    }
    console_add_cmd_table(led_command_table);

}

int led_console_command(int argc, char *argv[])
{
    int i;

    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < LED_CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i].cmd, argv[0]) == 0)
            break;
    }

    if (i >= LED_CONSOLE_CMD_MAX)
    {
        printf("Unrecognized command: %s\n", argv[0]);
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case LED_CONSOLE_CMD_INIT:
            wiced_led_service_init();
            break;

        case LED_CONSOLE_CMD_DEINIT:
            wiced_led_service_deinit();
            break;

        case LED_CONSOLE_CMD_LED_ON:
            if (argc > 1)
            {
                int led;
                led = atoi(argv[1]);
                if (led < PLATFORM_LED_COUNT)
                {
                    wiced_led_set_state(led, WICED_LED_ON);
                }
            }
            break;
        case LED_CONSOLE_CMD_LED_OFF:
            if (argc > 1)
            {
                int led;
                led = atoi(argv[1]);
                if (led < PLATFORM_LED_COUNT)
                {
                    wiced_led_set_state(led, WICED_LED_OFF);
                }
            }
            break;
        case LED_CONSOLE_CMD_START: /* start one of our own patterns */
            if (argc > 1)
            {
                int pattern;
                pattern = atoi(argv[1]);
                if (pattern < LED_TEST_MAX_PATTERNS)
                {
                    /* NOTE: this will fail if wiced_led_service_init() not called */
                    if (wiced_led_service_start( &led_test_patterns[pattern] ) != WICED_SUCCESS)
                    {
                        printf("wiced_led_service_start() Failed - use 'init' command first\r\n");
                    }
                }
            }
            break;
        case LED_CONSOLE_CMD_PATTERN_START: /* start a pre-defined pattern */
            if (argc > 1)
            {
                int pattern;
                pattern = atoi(argv[1]);
                if (pattern < WICED_LED_PATTERN_MAX)
                {
                    /* NOTE: this will fail if wiced_led_service_init() not called */
                    if (wiced_led_service_start_pattern( pattern ) != WICED_SUCCESS)
                    {
                        printf("wiced_led_service_start_pattern() Failed - use 'init' command first\r\n");
                    }
                }
            }
            break;
        case LED_CONSOLE_CMD_STOP:
            wiced_led_service_stop();
            break;
    }

    return 0;
}
