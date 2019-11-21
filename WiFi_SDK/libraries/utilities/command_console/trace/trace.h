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
#ifndef TRACE_H_
#define TRACE_H_

#include "trace_hook.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * ORDER OF EXECUTION
 * 1. trace_start_function_t
 * 2. set_trace_hooks
 * 3. COMMAND
 * 4. unset_trace_hooks
 * 5. trace_stop_function_t
 * 6. trace_process_function_t
 * 7. trace_cleanup_function_t
 */

/** A function to process trace data. */
typedef void (*trace_process_function_t) ( void );

/** A function to prematurely process trace data. */
typedef void (*trace_flush_function_t) ( void );

/** Details about a method for processing trace data. */
typedef struct trace_process_t
{
    char *name;
    trace_process_function_t process_command;
    trace_flush_function_t flush_command;
} trace_process_t;

/** A function to start tracing. */
typedef void (*trace_start_function_t) ( trace_flush_function_t );

/** A method to pause/stop tracing. */
typedef void (*trace_stop_function_t) ( void );

/** A method to be executed immediately before processing trace data. */
typedef void (*trace_preprocess_function_t) ( void );

/** A method to clean up after processing a trace. */
typedef void (*trace_cleanup_function_t) ( void );

/** Details about a method for tracing thread execution. */
typedef struct trace_t
{
    char *name;
    trace_start_function_t start_command;
    trace_stop_function_t stop_command;
    trace_preprocess_function_t preprocess_command;
    trace_cleanup_function_t cleanup_command;

    trace_task_hook_f task_hook;
    trace_tick_hook_f tick_hook;

    trace_process_t *process_types;
} trace_t;

/**
 * Trace the thread execution of a command.
 *
 * Prefix a command with `trace' to execute the command and then print out
 * thread tracing information in a graphical format.
 *
 * This function is a stub which is network and RTOS independent.
 */
int console_trace( int argc, char *argv[] );

/**
 * Starts or resumes the tracing of the RTOS scheduler.
 */
int console_start_trace( int argc, char *argv[] );

/**
 * Temporarily pauses scheduler tracing without removing any of the existing
 * trace data.
 */
int console_end_trace( int argc, char *argv[] );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef TRACE_H_ */
