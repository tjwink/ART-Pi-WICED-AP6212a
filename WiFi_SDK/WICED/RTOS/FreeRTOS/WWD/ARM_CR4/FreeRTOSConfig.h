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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "platform_config.h"
#include "wwd_FreeRTOS_systick.h"
#include "wwd_assert.h"

#ifdef ENABLE_TASK_TRACE
#include "FreeRTOS_trace.h"
#endif /* ifdef ENABLE_TASK_TRACE */


#if defined ( __IAR_SYSTEMS_ICC__ )
/* This file is included from the IAR portasm.s, so must avoid C
declarations in that case */
#include "platform_sleep.h"
#endif /* if defined ( __IAR_SYSTEMS_ICC__ ) */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configNO_MALLOC                             ( 0 )
#define configUSE_NEWLIB_MALLOC_LOCK                ( 1 )
#define configUSE_TIMERS                            ( 1 )
#define configTIMER_TASK_PRIORITY                   ( 7 )
#define configTIMER_QUEUE_LENGTH                    ( 5 )
#define configTIMER_TASK_STACK_DEPTH                ( ( unsigned short ) (1024 / sizeof( portSTACK_TYPE )) )
#define configUSE_PREEMPTION                        ( 1 )
#define configUSE_IDLE_HOOK                         ( 0 )
#define configUSE_TICK_HOOK                         ( 0 )
#define configCPU_CLOCK_HZ                          ( ( unsigned long ) CPU_CLOCK_HZ )
#define configTICK_RATE_HZ                          ( ( TickType_t ) SYSTICK_FREQUENCY )
#define configMAX_PRIORITIES                        ( 10 )
#define configMINIMAL_STACK_SIZE                    ( ( unsigned short ) (350 / sizeof( portSTACK_TYPE )) ) /* size of idle thread stack */
#define configMAX_TASK_NAME_LEN                     ( 16 )
#ifndef configUSE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY                    ( 0 )
#endif /* configUSE_TRACE_FACILITY */
#define configUSE_16_BIT_TICKS                      ( 0 )
#define configIDLE_SHOULD_YIELD                     ( 1 )
#ifndef configUSE_MUTEXES
#define configUSE_MUTEXES                           ( 1 )
#endif /* ifndef configUSE_MUTEXES */
#define configUSE_COUNTING_SEMAPHORES               ( 1 )
#define configSUPPORT_STATIC_ALLOCATION             ( 1 )

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                       ( 0 )
#define configMAX_CO_ROUTINE_PRIORITIES             ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
 to exclude the API function. */

#define INCLUDE_vTaskPrioritySet                    ( 1 )
#define INCLUDE_uxTaskPriorityGet                   ( 1 )
#define INCLUDE_vTaskDelete                         ( 1 )
#define INCLUDE_vTaskCleanUpResources               ( 0 )
#define INCLUDE_vTaskSuspend                        ( 1 )
#define INCLUDE_vTaskDelayUntil                     ( 1 )
#define INCLUDE_vTaskDelay                          ( 1 )
#define INCLUDE_xTaskGetCurrentThread               ( 1 )
#define INCLUDE_eTaskGetState                       ( 1 )
#define INCLUDE_xTaskAbortDelay                     ( 1 )
#define INCLUDE_vTaskGetStackInfo                   ( 1 )
#define INCLUDE_xTaskIsTaskFinished                 ( 1 )


/* Check for stack overflows - requires defining vApplicationStackOverflowHook */
#define configCHECK_FOR_STACK_OVERFLOW              ( 2 )

/* Run a handler if a malloc fails - vApplicationMallocFailedHook */
#define configUSE_MALLOC_FAILED_HOOK                ( 1 )


/* platform-specific tick functions */
#define configSETUP_TICK_INTERRUPT                  platform_tick_start
/* FreeRTOS generally needs this because it does not expect the platform-specific
 irq handler to clear the interrupt, but ours, being RTOS-independent, does.
 So we have to explicitly #define this to nothing.  */
#define configCLEAR_TICK_INTERRUPT()
/* fiqirq_status */
#define configEOI_ADDRESS                           PLATFORM_APPSCR4_REGBASE(0x10)


#if defined( DEBUG ) && ( ! defined( UNIT_TESTER ) )
#define configASSERT( expr )   wiced_assert( "FreeRTOS assert", expr )
#endif /* ifdef DEBUG */


#ifdef WICED_DISABLE_MCU_POWERSAVE

#define configUSE_IDLE_SLEEP_HOOK ( 1 )

#else /* ifdef WICED_DISABLE_MCU_POWERSAVE */

#define configUSE_TICKLESS_IDLE ( 1 )

#endif /* ifdef WICED_DISABLE_MCU_POWERSAVE */


#ifdef NETWORK_NOTIFY_RELEASED_PACKETS

#define MEMP_FREE_NOTIFY
extern void memp_free_notify( unsigned int type );

#endif /* ifdef NETWORK_NOTIFY_RELEASED_PACKETS */


#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* FREERTOS_CONFIG_H */

