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
* WICED Bluetooth OTA2 Upgrade
*
* This file provides function required to support Over the Air WICED Upgrade.
*
* To download host sends command to download with length of the patch to be
* transmitted.  GATT Write Requests are used to send commands and portions of
* data.  In case of an error Error Response indicates failure to the host.
* Host sends fixed chunks of data.  After all the bytes has been downloaded
* and acknowledged host sends verify command that includes CRC32 of the
* whole patch.  During the download device saves data directly to the
* serial flash.  At the verification stage device reads data back from the
* NVRAM and calculates checksum of the data stored there.  Result of the
* verification is indicated in the Write Response or Error Response GATT message.
*
*/
#include "bt_types.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_firmware_upgrade.h"
#include "wiced_bt_fw_upgrade.h"
#include "wiced_ota2_image.h"
#include "platform_ocf.h"
#include "wiced_bt_trace.h"

/******************************************************
 *                      Constants
 ******************************************************/
//#define ENABLE_WICED_FW_DEBUG   1
#define OTA_FW_UPGRADE_READ_CHUNK                   128
#define OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT         512
#define OTA2_IMAGE_HEADER               256

#define EF_BASE_ADDR                (0x500000u)
#define FLASH_SECTOR_SIZE           (4*1024)
#define FW_UPGRADE_FLASH_SIZE       0x26000    //  ss + boot1 + boot2 = 152 KB

typedef struct
{
// device states during OTA FW upgrade
#define OTA_STATE_IDLE                   0
#define OTA_STATE_READY_FOR_DOWNLOAD     1
#define OTA_STATE_DATA_TRANSFER          2
#define OTA_STATE_VERIFICATION           3
#define OTA_STATE_VERIFIED               4
#define OTA_STATE_ABORTED                5
    int32_t         state;
    uint8_t         bdaddr[6];               // BDADDR of connected device
    uint16_t        client_configuration;    // characteristic client configuration descriptor
    uint8_t         status;                  // Current status
    uint16_t        current_offset;          // Offset in the image to store the data
    int32_t         total_len;               // Total length expected from the host
    int32_t         current_block_offset;
    int32_t         total_offset;
    uint32_t        crc32;
    uint32_t        recv_crc32;
    uint8_t         indication_sent;
    wiced_timer_t   reset_timer;
    uint8_t         read_buffer[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];
} ota_fw_upgrade_state_t;


/******************************************************
 *               Variables Definitions
 ******************************************************/
wiced_bool_t                                g_is_bootloader_image;
wiced_bool_t                                g_ota2_header_parsed;
wiced_ota2_image_type_t                 g_ota2_image_type;
uint8_t                                         g_ota2_buffer[OTA2_IMAGE_HEADER];
uint32_t                                        g_ota2_recv_bytes;

ota_fw_upgrade_state_t                        ota_fw_upgrade_state;
uint16_t                                      ota_client_config_descriptor = 0;
wiced_firmware_upgrade_pre_reboot_callback_t *ota_pre_reboot_callback = NULL;

static void                   ota_fw_upgrade_set_client_configuration(uint16_t client_config);
static wiced_bool_t           ota_fw_upgrade_handle_data(uint16_t conn_id, uint8_t *data, int32_t len);
static wiced_bool_t           ota_fw_upgrade_handle_command(uint16_t conn_id, uint8_t *data, int32_t len);
static wiced_bt_gatt_status_t ota_fw_upgrade_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val);
static void                   ota_fw_upgrade_reset_timeout(void *param);
static wiced_bool_t ota2_upgrade_handle_data (uint16_t conn_id, uint8_t *data, int32_t len);

void ota_fw_upgrade_init_data( void );
#ifdef ENABLE_WICED_FW_DEBUG
void dump_hex(uint8_t *p, uint32_t len);
#endif
void my_delayUs(UINT32 delay);
void utilslib_delayUs(UINT32 delay);
int32_t ota_fw_upgrade_verify( void );
extern UINT32 update_crc32( UINT32 crc, UINT8 *buf, UINT16 len );
extern void watchdog_PetWatchDog(void);
extern wiced_result_t wiced_hal_eflash_read( uint32_t offset, uint8_t* p_buffer, uint32_t length );
extern wiced_bool_t wiced_bt_l2cap_update_ble_conn_params (wiced_bt_device_address_t rem_bdRa, uint16_t min_int, uint16_t max_int, uint16_t latency, uint16_t timeout);

extern uint32_t wiced_ota2_image_get_offset( wiced_ota2_image_type_t ota_type );

/*
 * Initializes global data
 */
void ota_fw_upgrade_init_data(void)
{
    ota_fw_upgrade_state.state           = OTA_STATE_IDLE;
    ota_fw_upgrade_state.indication_sent = WICED_FALSE;

    g_is_bootloader_image = WICED_TRUE;
    g_ota2_header_parsed = WICED_FALSE;
    g_ota2_image_type = WICED_OTA2_IMAGE_TYPE_NONE;
    g_ota2_recv_bytes = 0;
    memset (g_ota2_buffer,0,OTA2_IMAGE_HEADER);
}

/*
 * Initialize peripheral UART upgrade procedure
 */
wiced_bool_t wiced_ota_fw_upgrade_init(wiced_firmware_upgrade_pre_reboot_callback_t *p_reboot_callback)
{
    wiced_fw_upgrade_nv_loc_len_t nv_loc_len;

    /* In Serial flash first 8K i.e (addr range 0x0000 to 0x2000)reserved for static section and for fail safe OTA block.
    Caution!!: Application should not modify these sections. */
    /* Remaining portion can be used for Vendor specific data followed by firmware
    It is applications choice to choose size for Vendor specific data and firmware.
    Apart from vendor specific portion, remaining area equally divided for active firmware and upgradable firmware.
    Below are the example offsets for 4M bit Serial flash
    Note: below configuration should not conflict with ConfigDSLocation, DLConfigVSLocation and DLConfigVSLength configured in
    platform sflash .btp file */

    // 1MB Serial flash offsets for firmware upgrade.
    nv_loc_len.ss_loc   = EF_BASE_ADDR + 0x0000;

    // Make SS size 2 * flash sector side to be able to erase ds1 and ds2
    nv_loc_len.vs1_loc  = EF_BASE_ADDR + (FLASH_SECTOR_SIZE * 2); // one sector after SS for fail safe
    nv_loc_len.vs1_len  = FLASH_SECTOR_SIZE;
    nv_loc_len.vs2_loc  = nv_loc_len.vs1_loc + nv_loc_len.vs1_len;
    nv_loc_len.vs2_len  = FLASH_SECTOR_SIZE;
    nv_loc_len.ds1_loc  = nv_loc_len.vs2_loc + nv_loc_len.vs2_len;
#if defined(USE_256K_SECTOR_SIZE)
    nv_loc_len.ds1_len  = FLASH_SECTOR_SIZE;
    nv_loc_len.ds2_loc  = nv_loc_len.ds1_loc + nv_loc_len.ds1_len;
    nv_loc_len.ds2_len  = FLASH_SECTOR_SIZE;
#else
    {
        uint32_t ds1_ds2_length = FW_UPGRADE_FLASH_SIZE - nv_loc_len.ds1_loc + EF_BASE_ADDR;
        nv_loc_len.ds1_len  = ds1_ds2_length/2;
        nv_loc_len.ds2_loc  = nv_loc_len.ds1_loc + nv_loc_len.ds1_len;
        nv_loc_len.ds2_len  = ds1_ds2_length/2;
    }
#endif

    WICED_BT_TRACE("ds1:0x%08x, len:0x%08x\n", nv_loc_len.ds1_loc, nv_loc_len.ds1_len);
    WICED_BT_TRACE("ds2:0x%08x, len:0x%08x\n", nv_loc_len.ds2_loc, nv_loc_len.ds2_len);

    if (wiced_firmware_upgrade_init(&nv_loc_len, FW_UPGRADE_FLASH_SIZE) == WICED_FALSE)
    {
        WICED_BT_TRACE("WARNING: Upgrade will fail - active DS is not one of the expected locations\n");
        WICED_BT_TRACE("WARNING: Please build with OTA_FW_UPGRADE=1 and download \n");
        return WICED_FALSE;
    }
    ota_fw_upgrade_init_data();
    ota_pre_reboot_callback = p_reboot_callback;


    return WICED_TRUE;
}

/*
 * Connection with peer device is established
 */
void wiced_ota_fw_upgrade_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        memcpy(ota_fw_upgrade_state.bdaddr, p_status->bd_addr, BD_ADDR_LEN);
    }
}

/*
 * Process GATT Read request
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    UNUSED_PARAMETER(conn_id);

    switch (p_read_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        if (*p_read_data->p_val_len < 2)
            return WICED_BT_GATT_INVALID_ATTR_LEN;

        if (p_read_data->offset == 1)
        {
            p_read_data->p_val[0] = (uint8_t)(ota_client_config_descriptor >> 8);
            *p_read_data->p_val_len = 1;
        }
        else
        {
            p_read_data->p_val[0] = (uint8_t)(ota_client_config_descriptor & 0xff);
            p_read_data->p_val[1] = (uint8_t)(ota_client_config_descriptor >> 8);
            *p_read_data->p_val_len = 2;
        }
        return WICED_BT_GATT_SUCCESS;
        break;
    default:
        break;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process GATT Write request
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    switch (p_write_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        if (!ota_fw_upgrade_handle_command(conn_id, p_write_data->p_val, p_write_data->val_len))
        {
            WICED_BT_TRACE("ota_handle_command failed.\n");
            return WICED_BT_GATT_ERROR;
        }
        break;

    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_write_data->val_len != 2)
        {
            WICED_BT_TRACE("ota client config wrong len %d\n", p_write_data->val_len);
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        ota_fw_upgrade_set_client_configuration((uint16_t)(p_write_data->p_val[0] + (p_write_data->p_val[1] << 8)));
        break;

    case HANDLE_OTA_FW_UPGRADE_DATA:
        if (!ota2_upgrade_handle_data(conn_id, p_write_data->p_val, p_write_data->val_len))
        {
            WICED_BT_TRACE("ota_handle_data failed.\n");
            return WICED_BT_GATT_INTERNAL_ERROR;
        }
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT indication confirmation
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;

    WICED_BT_TRACE("wsr_upgrade_indication_confirm state:%d \n", p_state->state);

    WICED_BT_TRACE("ota_fw_upgrade_indication_cfm_handler, conn %d hdl %d\n", conn_id, handle);

    /* for non bootloader, dont not swap DS section */
    if (g_is_bootloader_image == WICED_FALSE)
    {
        if (wiced_ota2_image_fakery(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT) != WICED_SUCCESS)
        {
            WICED_BT_TRACE (" Update status %d for image %d is Failed !!!\r\n",WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT, g_ota2_image_type);
        }

        ota_fw_upgrade_init_data();
//        wiced_waf_reboot();

        return WICED_BT_GATT_SUCCESS;
    }

    if (handle == HANDLE_OTA_FW_UPGRADE_CONTROL_POINT)
    {
        if (p_state->state == OTA_STATE_VERIFIED)
        {
            if (ota_pre_reboot_callback)
            {
                (*ota_pre_reboot_callback)();
            }
            wiced_firmware_upgrade_finish();
        }
        return WICED_BT_GATT_SUCCESS;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}

#ifdef ENABLE_WICED_FW_DEBUG
void dump_hex(uint8_t *p, uint32_t len)
{
    uint32_t i;
    char     buff1[100];

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 32; i++)
        {
            int s1 = (*p & 0xf0) >> 4;
            int s2 = *p & 0x0f;
            buff1[i * 3]     = (char)((s1 >= 0 && s1 <= 9) ? s1 + '0' : s1 - 10 + 'A');
            buff1[i * 3 + 1] = (char)((s2 >= 0 && s2 <= 9) ? s2 + '0' : s2 - 10 + 'A');
            buff1[i * 3 + 2] = ' ';
            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}

void my_delayUs(UINT32 delay)
{
    int i;
    for (i = 0; i < 300; i++)
        utilslib_delayUs(delay);
}
#endif

/*
 * verify function is called after all the data has been received and stored
 * in the NV.  The function reads back data from the NV and calculates the checksum.
 * Function returns TRUE if calculated CRC matches the one calculated by the host
 */
int32_t ota_fw_upgrade_verify( void )
{
    int32_t offset;
    uint32_t    base_ota2_offset=0;
    uint32_t crc32 = 0xffffffff;

    if (g_is_bootloader_image == WICED_FALSE)
    {
        base_ota2_offset = wiced_ota2_image_get_offset(g_ota2_image_type);
        WICED_BT_TRACE  ("%s: base offset=0x%x\n",__func__,base_ota2_offset);
    }

    for (offset = 0; offset < ota_fw_upgrade_state.total_len; offset += OTA_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read = ((offset + OTA_FW_UPGRADE_READ_CHUNK) < ota_fw_upgrade_state.total_len) ?
                                        OTA_FW_UPGRADE_READ_CHUNK : ota_fw_upgrade_state.total_len - offset;

        // read should be on the word boundary and in full words, we may read a bit more, but
        // include correct number of bytes in the CRC calculation

        wiced_firmware_upgrade_retrieve_from_nv((uint32_t)((uint32_t)offset+base_ota2_offset), memory_chunk, (uint32_t)((uint32_t)(bytes_to_read + 3) & 0xFFFFFFFC));

        crc32 = update_crc32(crc32, memory_chunk, (uint16_t)bytes_to_read);
#ifdef ENABLE_WICED_FW_DEBUG
        WICED_BT_TRACE("read offset:%x\n", offset);
        dump_hex(memory_chunk, (uint32_t)bytes_to_read);
        my_delayUs(25);
        watchdog_PetWatchDog();
#endif
    }
    crc32 = crc32 ^ 0xffffffff;

    WICED_BT_TRACE("stored crc:%4x received bytes crc:%4x recvd crc:%4x\n", crc32, ota_fw_upgrade_state.recv_crc32, ota_fw_upgrade_state.crc32);

    return (crc32 == ota_fw_upgrade_state.crc32);
}

/*
 * handle commands received over the control point
 */
wiced_bool_t ota_fw_upgrade_handle_command(uint16_t conn_id, uint8_t *data, int32_t len)
{
    uint8_t command = data[0];
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;

    WICED_BT_TRACE("OTA handle cmd:%d, state:%d\n", command, p_state->state);
    if (command == WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD)
    {
        wiced_bt_l2cap_update_ble_conn_params(ota_fw_upgrade_state.bdaddr, 6, 6, 0, 200);

        p_state->state = OTA_STATE_READY_FOR_DOWNLOAD;
        ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
        return (TRUE);
    }
    if (command == WICED_OTA_UPGRADE_COMMAND_ABORT)
    {
        p_state->state = OTA_STATE_ABORTED;
        ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
        return FALSE;
    }

    switch (p_state->state)
    {
    case OTA_STATE_IDLE:
        return (TRUE);

    case OTA_STATE_READY_FOR_DOWNLOAD:
        if (command == WICED_OTA_UPGRADE_COMMAND_DOWNLOAD)
        {
            // command to start upgrade should be accompanied by 4 bytes with the image size
            if (len < 5)
            {
                WICED_BT_TRACE("Bad Download len: %d \n", len);
                return (FALSE);
            }

            if (!wiced_firmware_upgrade_init_nv_locations())
            {
                WICED_BT_TRACE("failed init nv locations\n");
                value = WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
                ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
                return (FALSE);
            }

            p_state->current_offset       = 0;
            p_state->current_block_offset = 0;
            p_state->total_offset         = 0;
            p_state->total_len            = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
            p_state->recv_crc32           = 0xffffffff;
            p_state->state                = OTA_STATE_DATA_TRANSFER;

            WICED_BT_TRACE("state %d total_len %d \n", p_state->state, p_state->total_len);
            ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
            return (TRUE);
        }
        break;

    case OTA_STATE_DATA_TRANSFER:
        if (command == WICED_OTA_UPGRADE_COMMAND_VERIFY)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 5)
            {
                WICED_BT_TRACE("Bad Verify len %d \n", len);
                return (FALSE);
            }
            if (p_state->total_len != p_state->total_offset)
            {
                WICED_BT_TRACE("Verify failed received:%d out of %d\n", p_state->total_offset, p_state->total_len);
                p_state->state = OTA_STATE_ABORTED;
                value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
                return TRUE;
            }
            else
            {
                p_state->crc32 = (uint32_t)(data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24));
                if (ota_fw_upgrade_verify())
                {
                    WICED_BT_TRACE("Verify success\n");
                    p_state->state = OTA_STATE_VERIFIED;

                    // if we are able to send indication (good host) wait for the confirmation before the reboot
                    if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
                    {
                        if (wiced_bt_gatt_send_indication(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                        {
                            return TRUE;
                        }
                    }
                    // if we are unable to send indication, try to send notification and start 1 sec timer
                    if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
                    {
                        if (wiced_bt_gatt_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                        {
                            // notify application that we are going down
                            if (ota_pre_reboot_callback)
                            {
                                (*ota_pre_reboot_callback)();
                            }
                            // init timer for detect packet retransmission
                            wiced_rtos_init_timer( &ota_fw_upgrade_state.reset_timer, 1000, (timer_handler_t)ota_fw_upgrade_reset_timeout, NULL);
                            wiced_rtos_start_timer( &ota_fw_upgrade_state.reset_timer );

                            return TRUE;
                        }
                    }
                    WICED_BT_TRACE("failed to notify the app\n");
                    return FALSE;
                }
                else
                {
                    WICED_BT_TRACE("Verify failed\n");
                    p_state->state = OTA_STATE_ABORTED;
                    value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                    ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
                    return TRUE;
                }
            }
        }
        break;

    case OTA_STATE_ABORTED:
    default:
        break;
    }

    value = WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
    ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
    return FALSE;
}

wiced_bool_t ota2_upgrade_handle_data (uint16_t conn_id, uint8_t *data, int32_t len)
{
    /* read the 1st 256 bytes of the data. if it is a OTA2 image then
     * set the image type in ota2_image writer module */
     //uint8_t temp_data[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];
     wiced_ota2_image_header_t *ota2_image_header;

    if ( len <= 0 || data == NULL)
    {
        WICED_BT_TRACE ("Invalid data \r\n");
        return (FALSE);
    }

    if (g_ota2_header_parsed == WICED_TRUE )
    {
        return ota_fw_upgrade_handle_data(conn_id, data, len);
    }

    //else

    if ((g_ota2_recv_bytes + (uint32_t)len) >= OTA2_IMAGE_HEADER)
    {
        memcpy (&g_ota2_buffer[g_ota2_recv_bytes], data, (int)(OTA2_IMAGE_HEADER - g_ota2_recv_bytes));
        // update data pointer and len
        len = len -(int32_t)(OTA2_IMAGE_HEADER - g_ota2_recv_bytes);

        data = data + (OTA2_IMAGE_HEADER - g_ota2_recv_bytes);

        g_ota2_recv_bytes += (OTA2_IMAGE_HEADER - g_ota2_recv_bytes);
    }
    else
    {
        memcpy (&g_ota2_buffer[g_ota2_recv_bytes], data, len);
        g_ota2_recv_bytes += (uint32_t)len;
        data = data + len;
        return (TRUE); /* return here to get more data */
    }

     /* process the local buffer */
     ota2_image_header = (wiced_ota2_image_header_t*) g_ota2_buffer;
      /* swap the data to be correct for the platform */
    wiced_ota2_image_header_swap_network_order((wiced_ota2_image_header_t *)ota2_image_header, WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST);

    if (strncmp((char*)ota2_image_header->magic_string, WICED_OTA2_IMAGE_MAGIC_STRING, WICED_OTA2_IMAGE_MAGIC_STR_LEN) == 0)
    {
        // check for components. if coponents count is 1 then assume that this is a filesystem
        if (ota2_image_header->component_count == 1)
        {
            wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_FS);
            g_ota2_image_type = WICED_OTA2_IMAGE_TYPE_FS;
        }
        else
        {
            wiced_ota2_set_current_update(WICED_OTA2_IMAGE_TYPE_STAGED);
            g_ota2_image_type = WICED_OTA2_IMAGE_TYPE_STAGED;
        }
        g_is_bootloader_image = WICED_FALSE;
    }
    else
    {
        g_is_bootloader_image = WICED_TRUE;
    }

    g_ota2_header_parsed = WICED_TRUE;
    //correct the buffer alignment
    wiced_ota2_image_header_swap_network_order((wiced_ota2_image_header_t *)ota2_image_header, WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK);
    WICED_BT_TRACE ("recv_bytes=%d len=%d bootloader=%d img_type=%d\n",g_ota2_recv_bytes,len,g_is_bootloader_image,g_ota2_image_type);

    if (len > 0)
    {
        /* sends local buffer data first */
        ota_fw_upgrade_handle_data(conn_id, g_ota2_buffer, (int32_t)g_ota2_recv_bytes);
        return ota_fw_upgrade_handle_data(conn_id, data, len);
    }
    else
    {
        return ota_fw_upgrade_handle_data(conn_id, g_ota2_buffer, (int32_t)g_ota2_recv_bytes);
    }
}

/*
 * Process data chunk received from the host.  If received num of bytes equals to
 * OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT, save the data to NV.
 *
 */
wiced_bool_t ota_fw_upgrade_handle_data(uint16_t conn_id, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t *p = data;
    uint32_t base_ota2_offset=0;

    UNUSED_PARAMETER(conn_id);

    if (g_is_bootloader_image == WICED_FALSE)
    {
        base_ota2_offset = wiced_ota2_image_get_offset(g_ota2_image_type);
//        WICED_BT_TRACE  ("%s: base offset=0x%x\n",__func__,base_ota2_offset);
    }

    p_state->recv_crc32 = update_crc32(p_state->recv_crc32, data, (uint16_t)len);

    while (len)
    {
        int bytes_to_copy =
            (p_state->current_block_offset + len) < OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT ? len: (OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT - p_state->current_block_offset);

        if ((p_state->total_offset + p_state->current_block_offset + bytes_to_copy > p_state->total_len))
        {
            WICED_BT_TRACE("Too much data. size of the image %d \n", p_state->total_len);
            WICED_BT_TRACE("offset %d, block offset %d len rcvd %d \n",
                            p_state->total_offset, p_state->current_block_offset, len);
            return (FALSE);
        }

        memcpy (&(p_state->read_buffer[p_state->current_block_offset]), p, bytes_to_copy);
        p_state->current_block_offset += bytes_to_copy;

        /* Blink LED OFF while writing to FLASH (downloading an OTA2 Image) */
        wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);

        if ((p_state->current_block_offset == OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT) ||
            (p_state->total_offset + p_state->current_block_offset == p_state->total_len))
        {
#ifdef ENABLE_WICED_FW_DEBUG
            WICED_BT_TRACE("write offset:%x\n", p_state->total_offset);
            dump_hex(p_state->read_buffer, (uint32_t)p_state->current_block_offset);
#endif

            // write should be on the word boundary and in full words, we may write a bit more
            wiced_firmware_upgrade_store_to_nv((uint32_t)(p_state->total_offset)+base_ota2_offset, p_state->read_buffer, (uint32_t)(((uint32_t)(p_state->current_block_offset + 3)) & 0xFFFFFFFC));

            p_state->total_offset        += p_state->current_block_offset;
            p_state->current_block_offset = 0;

            if (p_state->total_offset == p_state->total_len)
            {
                p_state->recv_crc32 = p_state->recv_crc32 ^ 0xffffffff;
            }
        }

        len = len - bytes_to_copy;
        p = p + bytes_to_copy;

        /* Turn LED ON when done writing to FLASH */
        wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
        //WICED_BT_TRACE("remaining len: %d \n", len);
    }
    return (TRUE);
}

/*
 * Set new value for client configuration descriptor
 */
void ota_fw_upgrade_set_client_configuration(uint16_t client_config)
{
    ota_client_config_descriptor = client_config;
}

/*
 * Send Notification if allowed, or indication if allowed, or return error
 */
wiced_bt_gatt_status_t ota_fw_upgrade_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val)
{
    UNUSED_PARAMETER(conn_id);

    if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
        return wiced_bt_gatt_send_notification(conn_id, attr_handle, val_len, p_val);
    else if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
        return wiced_bt_gatt_send_indication(conn_id, attr_handle, val_len, p_val);
    return WICED_BT_GATT_ERROR;
}

/*
 * Process timeout started after the last notification to perform restart
 */
void ota_fw_upgrade_reset_timeout(void *param)
{
    UNUSED_PARAMETER(param);

    /* for non bootloader, dont not swap DS section */
    if (g_is_bootloader_image == WICED_FALSE)
    {
        /* update the header */
        wiced_ota2_image_fakery(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT);
        ota_fw_upgrade_init_data();
//        wiced_waf_reboot();
    }
    wiced_firmware_upgrade_finish();
}
