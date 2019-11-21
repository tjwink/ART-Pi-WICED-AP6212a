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
* Apple iBeacon header
*
* This file provides definitions and function prototypes for apple iBeacon device
*
*/

#ifdef __cplusplus
extern "C" {
#endif


#ifndef IBEACON_H
#define IBEACON_H


/* iBeacon application UUID */
#define UUID_IBEACON               0xd9, 0xb9, 0xec, 0x1f, 0x39, 0x25, 0x43, 0xd0, 0x80, 0xa9, 0x1e, 0x39, 0xd4, 0xce, 0xa9, 0x5c

/* iBeacon type */
#define IBEACON_TYPE               0x01
#define IBEACON_PROXIMITY          0x02, 0x15

/* iBeacon company ID */
#define IBEACON_COMPANY_ID_APPLE   0x4c, 0x00

/* iBeacon data length */
#define IBEACON_DATA_LENGTH        0x19

/* iBeacon Major & Minor number */
#define IBEACON_MAJOR_NUMBER       0x00, 0x01
#define IBEACON_MINOR_NUMBER       0x00, 0x01
#endif

#ifdef __cplusplus
}
#endif
