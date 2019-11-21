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

#include <stdio.h>
#include <string.h>
//#include "wwd_debug.h"
#include "trace.h"
#include "trace_hook.h"
#include "command_console.h"
#include "buffered/buffered_trace.h"
#include "gpio/gpio_trace.h"

extern const command_t *console_command_table;

/** A table containing all possible trace types. */
static const trace_t trace_types[] = {
    TRACE_T_BUFFER
    TRACE_T_GPIO
    { NULL, NULL, NULL, NULL, NULL, NULL, NULL, (trace_process_t []) { { NULL, NULL, NULL } } }
};

/** The currently active trace type. */
static const trace_t *active_trace = NULL;
/** The currently active trace process type. */
static const trace_process_t *active_trace_process = NULL;


int console_trace( int argc, char *argv[] )
{
    if ( active_trace != NULL )
    {
        WINFO_APP( ( "Already tracing...\r\n" ) );
        return ERR_CMD_OK;
    }

    const command_t *cmd_ptr = NULL;
    int cmd_argc;
    char **cmd_argv;

    /* Make sure a command and trace type was specified */
    if ( argc < 3 )
    {
        return ERR_INSUFFICENT_ARGS;
    }

    cmd_argc = argc - 1;
    cmd_argv = &argv[1];

    /* Find the trace type */
    for ( active_trace = trace_types; active_trace->name != NULL; active_trace++ )
    {
        if ( strcmp( cmd_argv[0], active_trace->name ) == 0 )
        {
            cmd_argc--;
            cmd_argv = &cmd_argv[1];
            break;
        }
    }

    /**
     * If a valid trace type was specified, then cmd_ptr now points to the
     * specified command.
     */

    if ( active_trace->name == NULL )
    {
        WINFO_APP( ( "Invalid trace type. Valid trace types are:\r\n" ) );
        const trace_t *p;
        int printed_anything = 0;
        for ( p = trace_types; p->name != NULL; p++ )
        {
            printed_anything = 1;
            WINFO_APP( ( "\t%s\r\n", p->name ) );
        }
        if ( !printed_anything )
        {
            WINFO_APP( ( "\t(none)\r\n" ) );
        }
        active_trace = NULL;
        active_trace_process = NULL;
        return ERR_UNKNOWN;
    }

    /* Find the trace process type */
    for ( active_trace_process = active_trace->process_types; active_trace_process->name != NULL; active_trace_process++ )
    {
        if ( strcmp( cmd_argv[0], active_trace_process->name ) == 0 )
        {
            cmd_argc--;
            cmd_argv = &cmd_argv[1];
            break;
        }
    }

    if ( ( active_trace_process->name == NULL ) && ( active_trace->process_types->name != NULL ) )
    {
        WINFO_APP( ( "Invalid trace process type. Valid trace process types are:\r\n" ) );
        const trace_process_t *p;
        int printed_anything = 0;
        for ( p = active_trace->process_types; p->name != NULL; p++ )
        {
            printed_anything = 1;
            WINFO_APP( ( "\t%s\r\n", p->name ) );
        }
        if ( !printed_anything )
        {
            WINFO_APP( ( "\t(none)\r\n" ) );
        }
        active_trace = NULL;
        active_trace_process = NULL;
        return ERR_UNKNOWN;
    }

    /* Find the command */
    for ( cmd_ptr = console_command_table; cmd_ptr->name != NULL; cmd_ptr++ )
    {
        if ( strcmp( cmd_argv[0], cmd_ptr->name ) == 0 )
        {
            break;
        }
    }

    /**
     * If a valid command was specified, then cmd_ptr now points to the
     * specified command.
     */

    if ( cmd_ptr->command == NULL )
    {
        active_trace = NULL;
        active_trace_process = NULL;
        return ERR_UNKNOWN_CMD;
    }

    /* Start tracing */
    WINFO_APP( ( "Starting trace...\r\n" ) );
    if ( active_trace->start_command )
    {
        active_trace->start_command( active_trace->process_types->flush_command );
    }
    set_trace_hooks( active_trace->task_hook, active_trace->tick_hook );


    /* Execute the command and fetch the return value */
    int return_value = cmd_ptr->command( cmd_argc, cmd_argv );
    UNUSED_PARAMETER( return_value );

    /* Finish tracing */
    unset_trace_hooks( );
    if ( active_trace->stop_command )
    {
        active_trace->stop_command( );
    }
    WINFO_APP( ( "Ended trace...\r\n" ) );
    if ( active_trace->preprocess_command )
    {
        active_trace->preprocess_command( );
    }
    if ( active_trace_process->process_command )
    {
        active_trace_process->process_command( );
    }
    if ( active_trace->cleanup_command )
    {
        active_trace->cleanup_command( );
    }

    active_trace = NULL;
    active_trace_process = NULL;

    return ERR_CMD_OK;
} /* trace */


int console_start_trace( int argc, char *argv[] )
{
    if ( active_trace != NULL )
    {
        WINFO_APP( ( "Already tracing...\r\n" ) );
        return ERR_CMD_OK;
    }

    int cmd_argc;
    char **cmd_argv;

    /* Make sure a command and trace type was specified */
    if ( argc < 2 )
    {
        return ERR_INSUFFICENT_ARGS;
    }

    cmd_argc = argc - 1;
    cmd_argv = &argv[1];

    /* Find the trace type */
    for ( active_trace = trace_types; active_trace->name != NULL; active_trace++ )
    {
        if ( strcmp( cmd_argv[0], active_trace->name ) == 0 )
        {
            cmd_argc--;
            cmd_argv = &cmd_argv[1];
            break;
        }
    }

    /**
     * If a valid trace type was specified, then cmd_ptr now points to the
     * specified command.
     */

    if ( active_trace->name == NULL )
    {
        WINFO_APP( ( "Invalid trace type. Valid trace types are:\r\n" ) );
        const trace_t *p;
        int printed_anything = 0;
        for ( p = trace_types; p->name != NULL; p++ )
        {
            printed_anything = 1;
            WINFO_APP( ( "\t%s\r\n", p->name ) );
        }
        if ( !printed_anything )
        {
            WINFO_APP( ( "\t(none)\r\n" ) );
        }
        active_trace = NULL;
        active_trace_process = NULL;
        return ERR_UNKNOWN;
    }

    /* Start tracing */
    WINFO_APP( ( "Starting trace...\r\n" ) );
    if ( active_trace->start_command )
    {
        active_trace->start_command( NULL );
    }
    set_trace_hooks( active_trace->task_hook, active_trace->tick_hook );

    return ERR_CMD_OK;
} /* console_start_trace */


int console_end_trace( int argc, char *argv[] )
{
    int cmd_argc = argc - 1;
    char **cmd_argv = &argv[1];

    if ( active_trace == NULL )
    {
        WINFO_APP( ( "Trace not active!\r\n" ) );
        return ERR_CMD_OK;
    }

    unset_trace_hooks( );
    if ( active_trace->stop_command )
    {
        active_trace->stop_command( );
    }
    WINFO_APP( ( "Ended trace...\r\n" ) );

    /* Find the trace process type */
    for ( active_trace_process = active_trace->process_types; active_trace_process->name != NULL; active_trace_process++ )
    {
        if ( strcmp( cmd_argv[0], active_trace_process->name ) == 0 )
        {
            cmd_argc--;
            cmd_argv = &cmd_argv[1];
            break;
        }
    }

    if ( ( active_trace_process->name == NULL ) && ( active_trace->process_types->name != NULL ) )
    {
        WINFO_APP( ( "Invalid trace process type. Valid trace process types are:\r\n" ) );
        const trace_process_t *p;
        int printed_anything = 0;
        for ( p = active_trace->process_types; p->name != NULL; p++ )
        {
            printed_anything = 1;
            WINFO_APP( ( "\t%s\r\n", p->name ) );
        }
        if ( !printed_anything )
        {
            WINFO_APP( ( "\t(none)\r\n" ) );
        }
        active_trace = NULL;
        active_trace_process = NULL;
        return ERR_UNKNOWN;
    }

    if ( active_trace_process && active_trace_process->process_command )
    {
        active_trace_process->process_command( );
    }
    if ( active_trace->cleanup_command )
    {
        active_trace->cleanup_command( );
    }

    active_trace = NULL;
    active_trace_process = NULL;

    return ERR_CMD_OK;
} /* console_end_trace */
