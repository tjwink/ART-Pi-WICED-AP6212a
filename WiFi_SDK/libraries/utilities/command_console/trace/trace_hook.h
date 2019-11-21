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

/**
 * This is the main include file for trace functionality. This file contains
 * functions to start and stop tracing, but no way of access trace data.
 */

#ifndef TRACE_HOOK_H_
#define TRACE_HOOK_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*trace_task_hook_f)( TRACE_TASK_HOOK_SIGNATURE );
typedef void (*trace_tick_hook_f)( TRACE_TICK_HOOK_SIGNATURE );

/**
 * @name These functions can be used to set and unset the hook functions for
 * scheduler tracing.
 */
/** @{ */
void set_trace_hooks( trace_task_hook_f task, trace_tick_hook_f tick );
void unset_trace_hooks( void );
/** @} */


/** Hook called every time the scheduler does something interesting. */
void trace_task_hook( TRACE_TASK_HOOK_SIGNATURE );

/** Hook called every time the tick count is incremented. */
void trace_tick_hook( TRACE_TICK_HOOK_SIGNATURE );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* TRACE_HOOK_H_ */
