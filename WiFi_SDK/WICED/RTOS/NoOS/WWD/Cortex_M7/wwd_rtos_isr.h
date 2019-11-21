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

#include "wwd_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/* Use this macro to define an RTOS-aware interrupt handler where RTOS
 * primitives can be safely accessed
 *
 * @usage:
 * WWD_RTOS_DEFINE_ISR( my_irq_handler )
 * {
 *     // Do something here
 * }
 */
#if defined( __GNUC__ )

#define WWD_RTOS_DEFINE_ISR( function ) \
        void function( void ); \
        __attribute__(( interrupt, used, section(IRQ_SECTION) )) void function( void )

#elif defined ( __IAR_SYSTEMS_ICC__ )

#define WWD_RTOS_DEFINE_ISR( function ) \
        __root void function( void ); \
        __root void function( void )
#else

#define WWD_RTOS_DEFINE_ISR( function ) \
        void function( void )

#endif


/* Macro for mapping a function defined using WWD_RTOS_DEFINE_ISR
 * to an interrupt handler declared in
 * <Wiced-SDK>/WICED/platform/<Arch>/<Family>/platform_irq_handlers.h
 *
 * @usage:
 * WWD_RTOS_MAP_ISR( my_irq, USART1_irq )
 */
#if defined( __GNUC__ )

#define WWD_RTOS_MAP_ISR( function, isr ) \
        extern void isr( void ); \
        __attribute__(( alias( #function ))) void isr ( void );

#elif defined ( __IAR_SYSTEMS_ICC__ )

#define WWD_RTOS_MAP_ISR( function, isr ) \
        extern void isr( void ); \
        _Pragma( TO_STRING( weak isr=function ) )
#else

#define WWD_RTOS_MAP_ISR( function, isr )

#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif
