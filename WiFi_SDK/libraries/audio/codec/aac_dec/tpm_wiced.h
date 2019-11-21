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

/* THREADs PROFILING MACROs */

#ifndef _H_TPM_WICED_H_
#define _H_TPM_WICED_H_

#ifdef __cplusplus
extern "C" {
#endif


#if defined( TX_ENABLE_EVENT_TRACE )

/* ******************************** */
/* Trace Logging Macros : ENABLED   */
/* ******************************** */

    #include "TraceX.h"

typedef struct tpm_config_
{
    tpm_tool_t             tool_type;
    wiced_tracex_config_t* tool_config;
} tpm_config_t;


    #define TPM_START( err_ptr, tpm_config_ptr )              /* blank */
    #define TPM_STOP( err_ptr, tpm_config_ptr )               /* blank */
    #define TPM_REPORT( err_ptr, tpm_config_ptr )             /* blank */

    #define TPM_USRLOG( err, tpm_config_ptr, event_id, a, b, c, d )                 \
    do                                                                              \
    {                                                                               \
        if ( ( (tpm_config_t*) ( tpm_config_ptr ) )->tool_type == TPM_TOOL_TRACEX ) \
        {                                                                           \
            tx_trace_user_event_insert( ( event_id ), ( a ), ( b ), ( c ), ( d ) ); \
        }                                                                           \
    } while ( 0 );


#else /* TX_ENABLE_EVENT_TRACE */

/* ******************************** */
/* Trace Logging Macros : DISABLED  */
/* ******************************** */

typedef struct tpm_config_
{
    tpm_tool_t tool_type;
    void*      tool_config;
} tpm_config_t;


    #define TPM_START( err_ptr, config_ptr )                        /* blank */
    #define TPM_STOP( err_ptr, config_ptr )                         /* blank */
    #define TPM_REPORT( err_ptr, config_ptr )                       /* blank */
    #define TPM_USRLOG( err_ptr, config_ptr, event_id, a, b, c, d ) /* blank */
#endif /* TX_ENABLE_EVENT_TRACE */

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /*_H_TPM_WICED_H_*/
