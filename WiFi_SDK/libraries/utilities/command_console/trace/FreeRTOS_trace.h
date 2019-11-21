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

#ifndef FREERTOS_TRACE_H_
#define FREERTOS_TRACE_H_

/**
 * This is the main include file to enable FreeRTOS trace functionality.
 */

#include "portmacro.h"
#include "trace_action.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @remarks CLOCKTIME_T must be defined for the processor. This is currently
 * done using makefiles.
 */

/******************************************************************************/
/** @name Hook function signatures and variables                              */
/******************************************************************************/
/** @{ */

/** The signature of the task hook function. */
#define TRACE_TASK_HOOK_SIGNATURE           \
    void *p,                                \
    signed char TaskName[],                 \
    unsigned long RunTimeCounter,           \
    unsigned portBASE_TYPE TCBNumber,       \
    unsigned portBASE_TYPE TaskNumber,      \
    unsigned portBASE_TYPE Priority,        \
    unsigned portBASE_TYPE NewPriority,     \
    trace_action_t action

/**
 * The variables of TRACE_TASK_HOOK_ARGS.  Used to pass the variables on to the
 * currently active handler hook. */
#define TRACE_TASK_HOOK_VARIABLES           \
    p,                                      \
    TaskName,                               \
    RunTimeCounter,                         \
    TCBNumber,                              \
    TaskNumber,                             \
    Priority,                               \
    NewPriority,                            \
    action

/** The signature of the tick hook function. */
#define TRACE_TICK_HOOK_SIGNATURE           \
    portTickType TickCount

/**
 * The variables of TRACE_TICK_HOOK_ARGS. Used to pass the variables on to the
 * currently active handler hook.
 */
#define TRACE_TICK_HOOK_VARIABLES           \
    TickCount

/** @} */
/******************************************************************************/

/******************************************************************************/
/** @name RTOS-specific types                                                 */
/******************************************************************************/
/** @{ */
#define TCB_NUMBER_T        unsigned portBASE_TYPE
#define MAX_TASKNAME_LEN    configMAX_TASK_NAME_LEN
#define TICK_T              portTickType
#define TICKS_PER_SEC       configTICK_RATE_HZ
/** @} */
/******************************************************************************/

/**
 * configUSE_TRACE_FACILITY must be set to enable some FreeRTOS trace-specific
 * functionality.
 */
#ifndef configUSE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY (1)
#endif /* configUSE_TRACE_FACILITY */

#include "trace_hook.h"

/** @name Macros to be called by the scheduler to provide trace functionality */
/** @{ */
#define traceTASK_CREATE( pxNewTCB ) \
    traceTASK( pxNewTCB, 0, Trace_Create )
#define traceTASK_DELETE( pxTCB ) \
    traceTASK( pxTCB, 0, Trace_Delete )
#define traceTASK_SUSPEND( pxTCB ) \
    traceTASK( pxTCB, 0, Trace_Suspend )
#define traceTASK_RESUME( pxTCB ) \
    traceTASK( pxTCB, 0, Trace_Resume )
#define traceTASK_RESUME_FROM_ISR( pxTCB ) \
    traceTASK( pxTCB, 0, Trace_ResumeFromISR )
#define traceTASK_DELAY() \
    traceTASK( pxCurrentTCB, 0, Trace_Delay )
#define traceTASK_DELAY_UNTIL() \
    traceTASK( pxCurrentTCB, 0, Trace_Delay )
#define traceTASK_CREATE_FAILED() \
    traceTASK( pxNewTCB, 0,  Trace_Die )
#define traceTASK_PRIORITY_SET( pxTask, uxNewPriority ) \
    traceTASK( pxTask, uxNewPriority, Trace_PrioritySet )
#define traceTASK_SWITCHED_OUT() \
    traceTASK( pxCurrentTCB, 0, Trace_SwitchOut )
#define traceTASK_SWITCHED_IN() \
    traceTASK( pxCurrentTCB, 0, Trace_SwitchIn )
/** @} */

#define traceTASK( pxTCB, uxNewPriority, action )       \
    do                                                  \
    {                                                   \
        trace_task_hook( (void*) pxTCB,                 \
                         pxTCB->pcTaskName,             \
                         0,                             \
                         pxTCB->uxTCBNumber,            \
                         pxTCB->uxTaskNumber,           \
                         pxTCB->uxPriority,             \
                         uxNewPriority,                 \
                         action );                      \
    }                                                   \
    while( 0 )

#define traceTASK_INCREMENT_TICK( xTickCount )          \
    do                                                  \
    {                                                   \
        trace_tick_hook( xTickCount );                  \
    }                                                   \
    while( 0 )

#include "trace_hook.h"

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* FREERTOS_TRACE_H_ */
