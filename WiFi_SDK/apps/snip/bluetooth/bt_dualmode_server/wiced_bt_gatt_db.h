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
 * Proximity Server Sample Application (GATT Server database definitions)
 *
 */
#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
*******************************************************/

/******************************************************
 *               Variables Definitions
 *****************************************************/

enum {
    // ***** Primary service 'Generic Attribute'
    HDLS_GENERIC_ATTRIBUTE      =  0x0100,
    HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED,
    HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE,

    // ***** Primary service 'Generic Access'
    HDLS_GENERIC_ACCESS,
    HDLC_GENERIC_ACCESS_DEVICE_NAME,
    HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
    HDLC_GENERIC_ACCESS_APPEARANCE,
    HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,

    // ***** Primary service 'Link Loss'
    HDLS_LINK_LOSS,
    HDLC_LINK_LOSS_ALERT_LEVEL,
    HDLC_LINK_LOSS_ALERT_LEVEL_VALUE,

    // ***** Primary service 'Immediate Alert'
    HDLS_IMMEDIATE_ALERT,
    HDLC_IMMEDIATE_ALERT_LEVEL,
    HDLC_IMMEDIATE_ALERT_LEVEL_VALUE,

    // ***** Primary service 'TX Power'
    HDLS_TX_POWER,
    HDLC_TX_POWER_LEVEL,
    HDLC_TX_POWER_LEVEL_VALUE,
};


extern const uint8_t  gatt_db[];
extern const uint16_t gatt_db_size;

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
}
#endif
