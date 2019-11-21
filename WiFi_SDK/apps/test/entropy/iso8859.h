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

/* ISO 8859/1 Latin-1 "ctype" macro replacements. */

#ifdef __cplusplus
extern "C" {
#endif


extern unsigned char isoalpha[ 32 ], isoupper[ 32 ], isolower[ 32 ];


#define isISOspace( x )    ( ( isascii( ( (unsigned char) ( x ) ) ) && isspace( ( (unsigned char) ( x ) ) ) ) || ( ( x ) == 0xA0 ) )
#define isISOalpha( x )    ( ( isoalpha[ ( ( (unsigned char) ( x ) ) ) / 8 ] & ( 0x80 >> ( ( ( (unsigned char) ( x ) ) ) % 8 ) ) ) != 0 )
#define isISOupper( x )    ( ( isoupper[ ( ( (unsigned char) ( x ) ) ) / 8 ] & ( 0x80 >> ( ( ( (unsigned char) ( x ) ) ) % 8 ) ) ) != 0 )
#define isISOlower( x )    ( ( isolower[ ( ( (unsigned char) ( x ) ) ) / 8 ] & ( 0x80 >> ( ( ( (unsigned char) ( x ) ) ) % 8 ) ) ) != 0 )
#define isISOprint( x )   ( ( ( ( x ) >= ' ' ) && ( ( x ) <= '~' ) ) || ( ( x ) >= 0xA0 ) )
#define toISOupper( x )   ( isISOlower( x ) ? ( isascii( ( (unsigned char) ( x ) ) ) ?                       \
                                                toupper( x ) : ( ( ( ( (unsigned char) ( x ) ) != 0xDF ) &&  \
                                                                   ( ( (unsigned char) ( x ) ) != 0xFF ) ) ? \
                                                                 ( ( (unsigned char) ( x ) ) - 0x20 ) : ( x ) ) ) : ( x ) )
#define toISOlower( x )   ( isISOupper( x ) ? ( isascii( ( (unsigned char) ( x ) ) ) ?                \
                                                tolower( x ) : ( ( (unsigned char) ( x ) ) + 0x20 ) ) \
                            : ( x ) )


#ifdef __cplusplus
} /* extern "C" */
#endif
