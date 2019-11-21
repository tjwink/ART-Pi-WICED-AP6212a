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
* WICED Firmware Upgrade
*
* This file provides definitions and function prototypes for Common functionality used in
* WICED Firmware Upgrade
*
*/

/*****************************************************************************/
/** @addtogroup wiced_Firmware_upgrade   WICED Firmware Upgrade
 *
 * @{
 */
/*****************************************************************************/

#ifndef WICED_BT_FW_UPGRADE_H
#define WICED_BT_FW_UPGRADE_H

#include "wiced_bt_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************************************
* Recommended firmware upgrade 4 MBit serial flash offsets
 * -------------------------------------------------------------------------------------------------------------------
 * |  SS1 (4K @ 0)  |  Fail safe area(4K @ 0x1000)  |  VS1 (4K @ 0x2000)  | VS2 (4K @ 0x3000)  | DS1 (248K @ 0x4000)  | DS2 (248K @ 0x42000)
 *  -------------------------------------------------------------------------------------------------------------------
 * For reference only.
 * uint32_t ss_locations       =    0x0000;
 * uint32_t vs_location1          = 0x2000;     // VS section occupies 1 sector
 * uint32_t vs_length1            = 0x1000;     // 4K = 1 SF sector
 * uint32_t vs_location2          = 0x3000;     // Double buffer for VS
 * uint32_t vs_length2            = 0x1000;     // 4K = 1 SF sector = vs_length1
 * uint32_t ds1_location          = 0x4000
 * uint32_t ds1_length            = 0x3E000     // 240K
 * uint32_t ds2_location          = 0x42000
 * uint32_t ds2_length            = 0x3E000     // 240K = ds1 length
 *******************************************************************************************************/
typedef struct
{
    uint32_t ss_loc;    /**< static section location */
    uint32_t ds1_loc;   /**< ds1 location */
    uint32_t ds1_len;   /**< ds1 length */
    uint32_t ds2_loc;   /**< ds2 location */
    uint32_t ds2_len;   /**< ds2 length */
    uint32_t vs1_loc;   /**< vendor specific location 1 */
    uint32_t vs1_len;   /**< vendor specific location 1 length */
    uint32_t vs2_loc;   /**< vendor specific location 2 */
    uint32_t vs2_len;   /**< vendor specific location 2 length */
} wiced_fw_upgrade_nv_loc_len_t;

// GATT handles used for by the FW upgrade service
#define HANDLE_OTA_FW_UPGRADE_SERVICE                           0xff00
#define HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT      0xff01
#define HANDLE_OTA_FW_UPGRADE_CONTROL_POINT                     0xff02
#define HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR   0xff03
#define HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA               0xff04
#define HANDLE_OTA_FW_UPGRADE_DATA                              0xff05
#define HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO           0xff06
#define HANDLE_OTA_FW_UPGRADE_APP_INFO                          0xff07

/**
 * \brief Initialize WICED Firmware Upgrade module
 * \ingroup wiced_firmware_upgrade
 *
 * \details This function is typically called by the application during initialization
 * to initialize upgrade module with serial flash offsets
 *
 * \param p_sflash_nv_loc_len Offsets of different sections present in serial flash
 * \param sflash_size   serial flash size present on the tag board.(default size 4MB )
 *
 */
wiced_bool_t wiced_firmware_upgrade_init(wiced_fw_upgrade_nv_loc_len_t *p_sflash_nv_loc_len, uint32_t sflash_size);

/**
 * \brief Initialize NV locations
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application calls this function during the start of the firmware download
 * to setup memory locations depending on which partition is being used
 *
 */
uint32_t     wiced_firmware_upgrade_init_nv_locations(void);

/**
 * \brief Store memory chunk to memory
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application can call this function to store the next memory chunk in the
 * none volatile memory.  Application does not need to know which type of memory is
 * used or which partition is being upgraded.
 *
 * \param offset Offset in the memory where data need to be stored
 * \param data   Pointer to the chunk of data to be stored
 * \param len    Size of the memory chunk that need to be stored
 *
 */
uint32_t   wiced_firmware_upgrade_store_to_nv(uint32_t offset, uint8_t *data, uint32_t len);

/**
 * \brief Retrieve memory chunk from memory
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application typically calls this function when the upgrade process has
 * been completed to verify that the data has been successfully stored.  Application
 * does not need to know which type of memory is used or which partition is being upgraded.
 *
 * \param offset Offset in the memory from where data need to be retrieved
 * \param data   Pointer to the location to retrieve the data
 * \param len    Size of the memory chunk to be retrieved
 *
 */
uint32_t   wiced_firmware_upgrade_retrieve_from_nv(uint32_t offset, uint8_t *data, uint32_t len);

/**
 * \brief Retrieve memory chunk from memory
 * \ingroup wiced_firmware_upgrade
 *
 * \details After download is completed and verified this function is called to
 * switch active partitions with the one that has been receiving the new image.
 *
 */
void     wiced_firmware_upgrade_finish(void);

/**
 * \brief Optional callback to be executed before library restarts the chip after the upgrade
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application can register the callback to be executed after new firmware
 * has been downloaded and verified and before the library executes the restart.  This
 * can be useful if application needs to drop all connections.
 *
 */
typedef void (wiced_firmware_upgrade_pre_reboot_callback_t)(void);

/**
 * \brief Initialize WICED OTA Firmware Upgrade module
 * \ingroup wiced_firmware_upgrade
 *
 * \details This function is typically called by the application during initialization
 *
 * \param p_pre_reboot_callback The callback to be issued at the end of the upgrade procedure just before the chip reboot.
 *
 */
wiced_bool_t wiced_ota_fw_upgrade_init(wiced_firmware_upgrade_pre_reboot_callback_t *p_pre_reboot_callback);

/**
 * \brief Connection Status Notification
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application should call this function to pass connection status notification to the
 * OTA Firmware Upgrade library.
 */
void wiced_ota_fw_upgrade_connection_status_event(wiced_bt_gatt_connection_status_t *p_status);

/**
 * \brief OTA FW Upgrade Read Request
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application should call this function to pass GATT read request to the
 * OTA Firmware Upgrade library for the handles that belong to the FW Upgrade Service.
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);

/**
 * \brief OTA FW Upgrade Write Request
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application should call this function to pass GATT write request to the
 * OTA Firmware Upgrade library for the handles that belong to the FW Upgrade Service.
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);

/**
 * \brief OTA FW Upgrade Indication Confirmation
 * \ingroup wiced_firmware_upgrade
 *
 * \details Application should call this function to pass GATT indication confirm request to the
 * OTA Firmware Upgrade library for the handles that belong to the FW Upgrade Service.
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
/**@} wiced_Firmware_upgrade */
