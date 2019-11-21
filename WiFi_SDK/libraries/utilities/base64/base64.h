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


/** List of Base64 conversion standards
 *
 */
typedef enum
{
    BASE64_STANDARD                         = ( ( '+' << 16 ) | ( '/' << 8 ) | '=' ),  /* RFC 1421, 2045, 3548, 4648, 4880 */
    BASE64_NO_PADDING                       = ( ( '+' << 16 ) | ( '/' << 8 )       ),  /* RFC 1642, 3548, 4648 */
    BASE64_URL_SAFE_CHARSET                 = ( ( '-' << 16 ) | ( '_' << 8 )       ),  /* RFC 4648 */
    BASE64_URL_SAFE_CHARSET_WITH_PADDING    = ( ( '-' << 16 ) | ( '_' << 8 ) | '=' ),  /* RFC 4648 */
    BASE64_Y64                              = ( ( '.' << 16 ) | ( '_' << 8 ) | '-' ),
    BASE64_XML_TOKEN                        = ( ( '.' << 16 ) | ( '-' << 8 )       ),
    BASE64_XML_IDENTIFIER                   = ( ( '_' << 16 ) | ( ':' << 8 )       ),
    BASE64_PROG_IDENTIFIER1                 = ( ( '_' << 16 ) | ( '-' << 8 )       ),
    BASE64_PROG_IDENTIFIER2                 = ( ( '.' << 16 ) | ( '_' << 8 )       ),
    BASE64_REGEX                            = ( ( '!' << 16 ) | ( '-' << 8 )       ),
} base64_options_t;

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
/**
 *
 *  @addtogroup base64     Base64
 *  @ingroup utilities
 *
 * This library implements Base64 encoding and decoding routines and their associated helper
 * functions. For more information on Base64 encoding, see RFC4648.
 *
 *  @{
 */

/** Returns true if character whitespace
 *
 * I.e. if character is a Tab, Line-Feed, Vertical Tab, Form Feed, Carriage-Return or a Space
 *
 * @param[in] c : The character to be tested
 *
 * @return true if character whitespace
 */
int is_base64_space( int c );


/** Encodes data into Base-64 coding which can be sent safely as text
 *
 * Terminating null will be appended.
 *
 * @param[in] src         : A pointer to the source data to be converted
 * @param[in] src_length  : The length of data to be converted (or -1 if the data is a null terminated string
 * @param[out] target     : The buffer that will receive the encoded data. NOTE: src and target can't be pointing to the same buffer.
 * @param[in] target_size : The size of the output buffer in bytes - will need to be at least 4*(src_length+2)/3
 * @param[in] options     : Specifies which Base64 encoding standard to use - see @ref base64_options_t
 *
 * @return number of Base64 characters output (not including terminating null),  otherwise negative indicates an error
 */
int base64_encode( unsigned char const* src, int32_t src_length, unsigned char* target, uint32_t target_size, base64_options_t options );

/** Decodes data from Base-64 coding which can be sent safely as text
 *
 * Terminating null will be appended.
 *
 * @param[in] src         : A pointer to the source Base64 coded data to be decoded
 * @param[in] src_length  : The length of data to be converted (or -1 if the data is a null terminated string
 * @param[out] target     : The buffer that will receive the decoded data.
 * @param[in] target_size : The size of the output buffer in bytes - will need to be at least 3*(src_length+3)/4
 * @param[in] options     : Specifies which Base64 encoding standard to use - see @ref base64_options_t
 *
 * @return number of decoded characters output (not including terminating null),  otherwise negative indicates an error
 */
int base64_decode( unsigned char const* src, int32_t src_length, unsigned char* target, uint32_t target_size, base64_options_t options );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
