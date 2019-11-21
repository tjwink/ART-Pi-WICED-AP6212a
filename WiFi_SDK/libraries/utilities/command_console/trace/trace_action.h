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

#ifndef TRACE_ACTION_H_
#define TRACE_ACTION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * This enumerator describes all of the possible scheduler actions that we may
 * want to trace.
 *
 * @remarks When writing trace actions to the buffer, it is assumed that this
 * enum will fit into 4-bits.
 */
typedef enum
{
#define TRACE_ACTION_T_START  0x0
    Trace_Invalid           = 0x0,
    Trace_Create            = 0x1,
    Trace_Delete            = 0x2,
    Trace_Suspend           = 0x3,
    Trace_Resume            = 0x4,
    Trace_ResumeFromISR     = 0x5,
    Trace_Delay             = 0x6,
    Trace_Die               = 0x7,
    Trace_PrioritySet       = 0x8,
    Trace_SwitchOut         = 0x9,
    Trace_SwitchIn          = 0xA,
    Trace_Executing         = 0xB
#define TRACE_ACTION_T_END    0xB
#define TRACE_ACTION_T_MASK   0xF
} trace_action_t;

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* TRACE_ACTION_H_ */
