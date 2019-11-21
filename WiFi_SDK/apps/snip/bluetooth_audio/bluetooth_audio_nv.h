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

#ifndef BT_NV_H
#define BT_NV_H

#include "wiced_bt_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/
#define BT_AUDIO_NV_MAX_LAST_PAIRED_DEVICES ( 8 )


/******************************************************
*                     Typedefs
******************************************************/
typedef    uint8_t  bt_audio_hash_table_t[BT_AUDIO_NV_MAX_LAST_PAIRED_DEVICES];

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *            Structures
 ******************************************************/
typedef struct
{
    uint32_t                       services_mask;      /**< Supported services*/
    wiced_bt_device_link_keys_t    device_link;            /**< BD address of the peer device. */
} bt_audio_paired_device_info_t;


typedef struct
{
    bt_audio_hash_table_t              bt_hash_table;
    bt_audio_paired_device_info_t      bt_paired_device_info[BT_AUDIO_NV_MAX_LAST_PAIRED_DEVICES];
    wiced_bt_local_identity_keys_t     bt_local_id_keys;
} bt_dct_t;

/******************************************************
 *            Function Declarations
 ******************************************************/

wiced_result_t bt_audio_nv_init( void );
void bt_audio_nv_deinit( void );
wiced_result_t bt_audio_nv_update_device_link_key( wiced_bt_device_link_keys_t *in_device );
wiced_result_t bt_audio_nv_update_last_connected_device( wiced_bt_device_address_t address );
wiced_result_t bt_audio_nv_get_device_info_by_addr( wiced_bt_device_address_t *address, bt_audio_paired_device_info_t *out_device );
wiced_result_t bt_audio_nv_get_device_info_by_index( uint8_t index, bt_audio_paired_device_info_t *out_device );
wiced_result_t bt_audio_nv_delete_device_info( wiced_bt_device_address_t address );
wiced_result_t bt_audio_nv_delete_device_info_list( void );
wiced_result_t bt_audio_nv_update_local_id_keys( wiced_bt_local_identity_keys_t *keys);
wiced_result_t bt_audio_nv_get_local_id_keys( wiced_bt_local_identity_keys_t *keys);

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif //BT_NV_H
