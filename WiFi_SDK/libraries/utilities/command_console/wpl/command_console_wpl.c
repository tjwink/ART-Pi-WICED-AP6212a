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

#include <stdio.h>
#include <string.h>

#include "wiced_result.h"
#include "wpl_console.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define ERR_CMD_OK    0
#define ERR_UNKNOWN -1
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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/



/*!
 ******************************************************************************
 * @return    Console error code indicating if the command ran correctly.
 */
int cmd_wpl_set_no_console_mode( int argc, char *argv[] )
{
    wpl_set_mode( WPL_MODE_NO_CONSOLE );
    return ERR_CMD_OK;
}



/*!
 ******************************************************************************
 * @return    Console error code indicating if the command ran correctly.
 */
int cmd_wpl_send( int argc, char* argv[] )
{
    if ( WICED_SUCCESS == wpl_process_console_command( argc, argv ) )
        return ERR_CMD_OK;
    else
        return ERR_UNKNOWN;
}

/*!
 ******************************************************************************
 * @return    Console error code indicating if the command ran correctly.
 */
int cmd_wpl_debug( int argc, char* argv[] )
{
    wpl_process_enable_console_prints( argc, argv );
    return ERR_CMD_OK;
}

