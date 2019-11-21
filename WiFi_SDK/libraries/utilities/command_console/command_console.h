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

#include "wiced_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define CMD_TABLE_END      { NULL, NULL, 0, NULL, NULL, NULL, NULL }
#define CMD_TABLE_DIV(str) { (char*) "",   NULL, 0, NULL, NULL, str,  NULL }

#ifdef WICED_WIFI_SOFT_AP_WEP_SUPPORT_ENABLED
    #define WEP_KEY_TYPE        WEP_KEY_ASCII_FORMAT /* WEP key type is set to ASCII format */
#endif

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_0_ARGUMENT_COMMAND_ENTRY(nm, func) \
        { \
          .name         = #nm,     \
          .command      = func, \
          .arg_count    = 0,        \
          .delimit      = NULL,     \
          .help_example = NULL,     \
          .format       = NULL,     \
          .brief        = NULL      \
        },

#define COMMAND_TABLE_ENDING() \
        { \
          .name         = NULL,     \
          .command      = NULL,     \
          .arg_count    = 0,        \
          .delimit      = NULL,     \
          .help_example = NULL,     \
          .format       = NULL,     \
          .brief        = NULL      \
        }

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    ERR_CMD_OK           =  0,
    ERR_UNKNOWN          = -1,
    ERR_UNKNOWN_CMD      = -2,
    ERR_INSUFFICENT_ARGS = -3,
    ERR_TOO_MANY_ARGS    = -4,
    ERR_ADDRESS          = -5,
    ERR_NO_CMD           = -6,
    ERR_TOO_LARGE_ARG    = -7,
    ERR_OUT_OF_HEAP      = -8,
    ERR_BAD_ARG          = -9,
/* !!!when adding values here, also update command_console.c:console_default_error_strings */
    ERR_LAST_ERROR       = -10
} cmd_err_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int       (*command_function_t)     ( int argc, char *argv[] );
typedef cmd_err_t (*help_example_function_t)( char* command_name, uint32_t eg_select );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    char* name;                             /* The command name matched at the command line. */
    command_function_t command;             /* Function that runs the command. */
    int arg_count;                          /* Minimum number of arguments. */
    const char* delimit;                          /* Custom string of characters that may delimit the arguments for this command - NULL value will use the default for the console. */

    /*
     * These three elements are only used by the help, not the console dispatching code.
     * The default help function will not produce a help entry if both format and brief elements
     * are set to NULL (good for adding synonym or short form commands).
     */
    help_example_function_t help_example;   /* Command specific help function. Generally set to NULL. */
    char *format;                           /* String describing argument format used by the generic help generator function. */
    char *brief;                            /* Brief description of the command used by the generic help generator function. */
} command_t;

/*
*
*/
typedef struct
{
    /* base parameters */
    wiced_uart_t uart;
    uint32_t line_len;
    char* buffer;
    uint32_t history_len;
    char* history_buffer_ptr;
    const char* delimiter_string;
    /*
     * exted with new parameters here
    */
    uint8_t thread_priority;
    uint8_t params_num;
} command_console_cfg_t;

typedef struct
{
    const char *name;
    void       *value;
} command_console_name_value_pair_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


wiced_result_t command_console_init ( wiced_uart_t uart, uint32_t line_len, char* buffer, uint32_t history_len, char* history_buffer_ptr, const char* delimiter_string);

wiced_result_t command_console_init_cfg ( command_console_cfg_t *cfg);

wiced_result_t command_console_deinit( void );
int            console_add_cmd_table ( const command_t *commands );
int            console_del_cmd_table ( const command_t *commands );
cmd_err_t      console_parse_cmd     ( const char* line );
int            console_prompt_confirm( void );

int hex_str_to_int( const char* hex_str );
int str_to_int( const char* str );

#ifdef __cplusplus
}
#endif
