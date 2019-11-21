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

#include <stdint.h>
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    JSON_STRING_TYPE,
    JSON_NUMBER_TYPE,
    JSON_VALUE_TYPE,
    JSON_ARRAY_TYPE,
    JSON_OBJECT_TYPE,
    JSON_BOOLEAN_TYPE,
    JSON_NULL_TYPE,
    UNKNOWN_JSON_TYPE
} wiced_JSON_types_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
#define OBJECT_START_TOKEN        '{'
#define OBJECT_END_TOKEN          '}'

#define ARRAY_START_TOKEN         '['
#define ARRAY_END_TOKEN           ']'

#define STRING_TOKEN              '"'

#define START_OF_VALUE            ':'

#define COMMA_SEPARATOR           ','

#define ESCAPE_TOKEN              '\\'

#define TRUE_TOKEN                't'

#define FALSE_TOKEN               'f'

#define NULL_TOKEN                'n'
/******************************************************
 *                    Structures
 ******************************************************/
typedef struct json_object{

    char*               object_string;
    uint8_t             object_string_length;
    wiced_JSON_types_t  value_type;
    char*               value;
    uint16_t            value_length;
    struct json_object* parent_object;

} wiced_json_object_t;

#define JSON_IS_NOT_ESC_CHAR( ch ) ( ( ch != '\b' )  &&  \
                                     ( ch != '\f' ) &&  \
                                     ( ch != '\n' ) &&  \
                                     ( ch != '\r' ) &&  \
                                     ( ch != '\t' ) &&  \
                                     ( ch != '\"' ) &&  \
                                     ( ch != '\\' ) )

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
typedef wiced_result_t (*wiced_json_callback_t)( wiced_json_object_t* json_object );
/** Register callback to be used by JSON parser
 *
 * @param[in] json_callback                  Callback to be called when JSON parser encounters an objects value. The callback will
 *                                           return the wiced_json_object_type, giving the object string name,value type the object
 *                                           and the value.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_JSON_parser_register_callback( wiced_json_callback_t json_callback );

/** Returns the current callback function registered with by JSON parser
 *
 * @return @ref wiced_json_callback_t
 */
wiced_json_callback_t wiced_JSON_parser_get_callback( void );

/** Parse JSON input string.
 *  This function will parse the JSON input string through a single parse, calling a callback whenever it encounters milestones
 *  an object, passing in object name, json value type, and a value (if value is string or number )
 *
 * @param[in] json_input   : JSON input array

 * @param[in] input_length : Length of JSON input
 *
 * @return @ref wiced_result_t
 *
 * @note: Currently escape values are not supported.
 */
wiced_result_t wiced_JSON_parser( const char* json_input, uint32_t input_length );


#ifdef __cplusplus
} /*extern "C" */
#endif
