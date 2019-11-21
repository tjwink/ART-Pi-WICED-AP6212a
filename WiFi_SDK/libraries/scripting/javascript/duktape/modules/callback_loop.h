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

#include "wiced_rtos.h"
#include "duktape.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

 #define WICED_DUKTAPE_CALLBACK_QUEUE_SIZE          (8)

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    WDCM_TIMER,
    WDCM_WEBSOCKET,
    WDCM_AUDIO,
    WDCM_XMLHTTPREQUEST,
} wiced_duktape_callback_module_id;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint16_t    module_id;
    uint16_t    event_id;
    void *      module_handle;
    void *      event_data1;
    uint32_t    event_data2;
} wiced_duktape_callback_queue_element_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/


/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_duktape_callback_post_event( wiced_duktape_callback_queue_element_t *new_entry );
wiced_result_t wiced_duktape_callback_loop_init( void );
wiced_result_t wiced_duktape_callback_loop_deinit(void);
void wiced_duktape_callback_loop(void);
void wiced_duktape_callback_loop_break(void);

#ifdef __cplusplus
} /* extern "C" */
#endif
