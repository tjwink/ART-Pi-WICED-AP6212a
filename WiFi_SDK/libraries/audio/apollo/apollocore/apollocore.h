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

#include "wiced_management.h"
#include "platform_dct.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_MULTICAST_IPV4_ADDRESS_DEFAULT MAKE_IPV4_ADDRESS(224, 0, 0, 55)

#define APOLLO_MAX_SPEAKERS         (24)
#define APOLLO_SPEAKER_NAME_LENGTH  (36)
#define APOLLO_PSP_RETRY_LIMIT      (7)
#define APOLLO_PSP_THRESHOLD        (3)
#define APOLLO_SWDIV_TIMEOUT        (50)

#define NANOSECONDS_PER_SECOND      (1000000000)
#define NANOSECONDS_PER_MILLISECOND (1000000)
#define MICROSECONDS_PER_SECOND     (1000000)
#define MILLISECONDS_PER_SECOND     (1000)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CHANNEL_MAP_NONE  = 0,          /* None or undefined     */
    CHANNEL_MAP_FL    = (1 << 0),   /* Front Left            */
    CHANNEL_MAP_FR    = (1 << 1),   /* Front Right           */
    CHANNEL_MAP_FC    = (1 << 2),   /* Front Center          */
    CHANNEL_MAP_LFE1  = (1 << 3),   /* LFE-1                 */
    CHANNEL_MAP_BL    = (1 << 4),   /* Back Left             */
    CHANNEL_MAP_BR    = (1 << 5),   /* Back Right            */
    CHANNEL_MAP_FLC   = (1 << 6),   /* Front Left Center     */
    CHANNEL_MAP_FRC   = (1 << 7),   /* Front Right Center    */
    CHANNEL_MAP_BC    = (1 << 8),   /* Back Center           */
    CHANNEL_MAP_LFE2  = (1 << 9),   /* LFE-2                 */
    CHANNEL_MAP_SIL   = (1 << 10),  /* Side Left             */
    CHANNEL_MAP_SIR   = (1 << 11),  /* Side Right            */
    CHANNEL_MAP_TPFL  = (1 << 12),  /* Top Front Left        */
    CHANNEL_MAP_TPFR  = (1 << 13),  /* Top Front Right       */
    CHANNEL_MAP_TPFC  = (1 << 14),  /* Top Front Center      */
    CHANNEL_MAP_TPC   = (1 << 15),  /* Top Center            */
    CHANNEL_MAP_TPBL  = (1 << 16),  /* Top Back Left         */
    CHANNEL_MAP_TPBR  = (1 << 17),  /* Top Back Right        */
    CHANNEL_MAP_TPSIL = (1 << 18),  /* Top Side Left         */
    CHANNEL_MAP_TPSIR = (1 << 19),  /* Top Side Right        */
    CHANNEL_MAP_TPBC  = (1 << 20),  /* Top Back Center       */
    CHANNEL_MAP_BTFC  = (1 << 21),  /* Bottom Front Center   */
    CHANNEL_MAP_BTFL  = (1 << 22),  /* Bottom Front Left     */
    CHANNEL_MAP_BTFR  = (1 << 23),  /* Bottom Front Right    */
    CHANNEL_MAP_TPLS  = (1 << 24),  /* Top Left Surround     */
    CHANNEL_MAP_TPRS  = (1 << 25),  /* Top Right Surround    */
    CHANNEL_MAP_LS    = (1 << 26),  /* Middle Left Surround  */
    CHANNEL_MAP_RS    = (1 << 27),  /* Middle Right Surround */
    CHANNEL_MAP_BLC   = (1 << 28),  /* Back Left Center      */
    CHANNEL_MAP_BRC   = (1 << 29),  /* Back Right Center     */
    CHANNEL_ZERO_030  = (1 << 30),  /* reserved: zero bit    */
    CHANNEL_VIRTUAL   = (1 << 31)   /* Virtual channel FLAG  */
} APOLLO_CHANNEL_MAP_T;

typedef enum
{
    APOLLO_ROLE_SOURCE,
    APOLLO_ROLE_SINK
} apollo_role_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/**
 *
 *  @defgroup multimedia     WICED Multimedia
 *  @ingroup  multimedia
 *
 *  @addtogroup apollocore   WICED Apollo Core API
 *  @ingroup multimedia
 *
 * This library implements core Apollo APIs
 *
 *  @{
 */
/*****************************************************************************/
/** Initialize the Apollo audio network.
 *
 * This routine should be used in place of wiced_network_init
 * for Apollo audio applications.
 *
 * @param interface          : WICED_STA_INTERFACE or WICED_AP_INTERFACE
 * @param ap_info            : AP entry for this network
 * @param static_ip_settings : used by the sender in the RMC group
 * @param role               : role of apollo device
 * @param other_ifaces       : bitmap of other interfaces in use.
 *
 * @return @ref wiced_result_t Status of the operation.
 */
wiced_result_t apollo_network_up_default( wiced_interface_t interface, wiced_config_ap_entry_t* ap_info,
                                          const wiced_ip_setting_t* static_ip_settings, const apollo_role_t role, uint32_t other_ifaces );

/** Deinitialize the Apollo audio network.
 *
 * This routine should be used in place of wiced_network_deinit
 * for Apollo audio applications.
 *
 * @param interface          : which interface to bring down (WICED_STA_INTERFACE or WICED_AP_INTERFACE)
 * @param bsstype            : WICED_BSS_TYPE_ADHOC or WICED_BSS_TYPE_INFRASTRUCTURE
 * @param role               : role of apollo device
 *
 * @return @ref wiced_result_t Status of the operation.
 */
wiced_result_t apollo_network_down( wiced_interface_t interface, wiced_bss_type_t bss_type, const apollo_role_t role );


/** Overwrite MAC address in Wi-Fi NVRAM data
 *
 * @return @ref wiced_result_t Status of the operation.
 */
wiced_result_t apollo_set_nvram_mac( void );

#ifdef __cplusplus
} /* extern "C" */
#endif
