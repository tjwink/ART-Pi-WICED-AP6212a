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
#include "http_server.h"
#include "big_http_server.h"

#ifdef BIG_INCLUDES_RESTFUL_SMART_SERVER
#include "restful_smart_uri.h"
#endif

#ifdef BIG_INCLUDES_BLE_MESH
#include "blemesh_uri.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BIG_HTTP_SERVER_STACK_SIZE  ( 5000 )

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_http_server_t http_server;

/* The order of the paths matters because of the use of wildcard character '*' */
static START_OF_HTTP_PAGE_DATABASE( web_pages )
#ifdef BIG_INCLUDES_RESTFUL_SMART_SERVER
    RESTFUL_SMART_URIS
#endif
#ifdef BIG_INCLUDES_BLE_MESH
    BLE_MESH_URIS
#endif
END_OF_HTTP_PAGE_DATABASE();

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t big_init_http_server( void )
{
    /* Start HTTP server */
    return wiced_http_server_start( &http_server, 80, 5, web_pages, WICED_STA_INTERFACE, BIG_HTTP_SERVER_STACK_SIZE );

}

wiced_result_t big_deinit_http_server( void )
{
    return wiced_http_server_stop( &http_server );
}
