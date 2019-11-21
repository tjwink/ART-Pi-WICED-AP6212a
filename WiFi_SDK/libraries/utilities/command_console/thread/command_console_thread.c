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
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "command_console.h"
#include "wiced_rtos.h"
#include "wiced_defaults.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define CONSOLE_THREAD_STACKSIZE    ( 4096 )
#define MAX_SPAWNED_THREADS         ( 5 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef enum
{
    THREAD_SPAWN_ARG_PRIORITY = 0,
    THREAD_SPAWN_ARG_INTERVAL,
} thread_spawn_args_t;

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    wiced_thread_t     thread;
    uint32_t           thread_number;
    wiced_bool_t       thread_running;
    char*              line;
    int                interval;    /* Thread sleep time(ms) before repeating the execution of command */
    int                priority;
    int                return_value;
} thread_details_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void thread_wrapper( uint32_t arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_bool_t threads_inited = WICED_FALSE;
static thread_details_t* spawned_threads[MAX_SPAWNED_THREADS] = {0};
static const char* thread_spawn_args[] = { "--priority", "--interval" };

/******************************************************
 *               Function Definitions
 ******************************************************/
int thread_spawn( int argc, char *argv[] )
{
    thread_details_t* detail = NULL;
    int priority_val         = WICED_DEFAULT_LIBRARY_PRIORITY;
    int interval_val         = 0;
    uint32_t command_size    = 0;
    int i, cmd_offset;
    wiced_result_t result;

    if ( argc < 2 )
    {
        return ERR_INSUFFICENT_ARGS;
    }

    /*
     * Check if any optional parameters are provided
     */
    cmd_offset = 1; /* Offset at which console command starts if no optional parameters are provided */
    for ( i = 1; i < ( argc - 1 ); i++ )
    {
        /* Check if --priority argument is provided */
        if ( strcmp( argv[i], thread_spawn_args[THREAD_SPAWN_ARG_PRIORITY] ) == 0 )
        {
            priority_val = atoi( argv[i + 1] );
            if ( priority_val == 0 )
            {
                printf( "Error in parsing --priority argument \r\n" );
                return ERR_INSUFFICENT_ARGS;
            }
            cmd_offset += 2;
            i++; /* Skip checking next argument */
        }

        /* Check if --interval argument is provided */
        if ( strcmp( argv[i], thread_spawn_args[THREAD_SPAWN_ARG_INTERVAL] ) == 0 )
        {
            interval_val = atoi( argv[i + 1] );
            if ( interval_val == 0 )
            {
                printf( "Error in parsing --interval argument \r\n" );
                return ERR_INSUFFICENT_ARGS;
            }
            cmd_offset += 2;
            i++; /* Skip checking next argument */
        }
    }

    if ( argc < ( cmd_offset + 1 ))
    {
        return ERR_INSUFFICENT_ARGS;
    }
    argc -= cmd_offset;
    argv = &argv[cmd_offset];

    if ( threads_inited == WICED_FALSE )
    {
        memset( spawned_threads, 0, sizeof(spawned_threads) );
        threads_inited = WICED_TRUE;
    }

    /* Find a spare thread detail */
    for ( i = 0; i < MAX_SPAWNED_THREADS; ++i )
    {
        if ( spawned_threads[i] == NULL )
        {
            spawned_threads[i] = calloc( 1, sizeof(thread_details_t) );
            detail = spawned_threads[i];
            detail->thread_number = i;
            break;
        }
        else if ( spawned_threads[i]->thread_running == WICED_FALSE )
        {
            wiced_rtos_delete_thread( &spawned_threads[i]->thread );
            detail = spawned_threads[i];
            detail->thread_number = i;
            break;
        }
    }

    if ( detail == NULL )
    {
        return ERR_UNKNOWN;
    }

    /* Count the total argument length */
    for( i = 0; i < argc; i++ )
    {
        command_size += strlen( argv[i] );
    }
    command_size += argc; /* Add space for delimiting spaces and terminating null */

    detail->line = (char*) calloc( 1, command_size );
    memset( detail->line, 0, command_size );
    for( i = 0; i < ( argc - 1 ); i++ )
    {
        strcat( detail->line, argv[i] );
        strcat( detail->line, " " );
    }
    /* The last argument should not be followed by a delimiter. */
    strcat( detail->line, argv[i] );
    detail->line[command_size - 1] = '\x00';
    detail->interval = interval_val;
    detail->priority = priority_val;

    /* Spawn the new thread */
    detail->thread_running = WICED_TRUE;
    result = wiced_rtos_create_thread( &detail->thread, priority_val, argv[0], thread_wrapper, CONSOLE_THREAD_STACKSIZE, detail );
    if ( result != WICED_SUCCESS )
    {
        detail->thread_running = WICED_FALSE;
        printf( "Thread creation failed with error code:%d\r\n", result );
        return ERR_UNKNOWN;
    }
    return ERR_CMD_OK;
}

static void thread_wrapper( uint32_t arg )
{
    thread_details_t *detail = (thread_details_t*) arg;

    printf( "Started thread %ld to execute \"%s\" command with interval:%dms priority:%d\r\n",
            detail->thread_number, detail->line, detail->interval, detail->priority );
    do
    {
        /* Execute the command and fetch the return value */
        detail->return_value = (int) console_parse_cmd( detail->line );
        printf( "Thread %ld, command \"%s\" exited with return value %d\r\n", detail->thread_number, detail->line, detail->return_value );

        /* Sleep until it is time to execute the command again */
        wiced_rtos_delay_milliseconds( detail->interval );
    } while ( detail->interval != 0 ); /* Zero interval indicates no repetition, so end loop */

    free( detail->line );
    detail->thread_running = WICED_FALSE;
    WICED_END_OF_CURRENT_THREAD( );
}

int thread_list( int argc, char* argv[] )
{
    int a;
    printf( "Running threads:\r\n" );
    for ( a = 0; a < MAX_SPAWNED_THREADS; ++a )
    {
        if (( spawned_threads[a] != NULL ) && ( spawned_threads[a]->thread_running == WICED_TRUE ))
        {
            printf("  %d: %s\r\n", a, spawned_threads[a]->line);
        }
    }
    return ERR_CMD_OK;
}

int thread_kill( int argc, char* argv[] )
{
    int a = atoi( argv[1] );
    if ( ( a >= 0 ) && ( a < MAX_SPAWNED_THREADS ) &&
         ( spawned_threads[a] != NULL ) && ( spawned_threads[a]->thread_running == WICED_TRUE ) )
    {
        wiced_rtos_delete_thread( &spawned_threads[a]->thread );
        spawned_threads[a]->thread_running = WICED_FALSE;
        printf( "Thread %d has been terminated\r\n", a );
    }
    else
    {
        printf( "Thread not found\r\n" );
    }
    return ERR_CMD_OK;
}
