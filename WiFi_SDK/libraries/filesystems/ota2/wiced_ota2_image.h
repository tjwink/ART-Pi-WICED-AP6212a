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
 *  Wiced OTA Image structure and API.
 *
 *  OTA Image File consists of:
 *
 *  wiced_ota2_image_header_t            - info about the OTA Image
 *  wiced_ota2_image_component_t         - array of individual component info structures
 *  data                                - all the components
 *
 * start of file
 * +================================+
 * |        OTA Image Header        |   OTA Header
 * +================================+
 * |       Component 0 header       |
 * |----                        ----|
 * |       Component 1 header       |   Array of Component Headers
 * |----                        ----|
 * |             ......             |
 * |----                        ----|
 * |       Component n header       |
 * +================================+
 * |       Component 0 Data         |
 * +--------------------------------+
 * |       Component 1 Data         |   Component Data
 * +--------------------------------+
 * |             ......             |
 * +--------------------------------+
 * |       Component n Data         |
 * +================================+
 * end of file
 *
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>

#include <platform_dct.h>
/* Include platform-specific defines */
#if !defined(OTA2_BUILDER_UTILITY) && ((DCT_BOOTLOADER_SDK_VERSION >= DCT_BOOTLOADER_SDK_3_5_2) && defined(UPDATE_FROM_SDK))
#include "platform_ota2_image.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                     Macros
 ******************************************************/

/* To use a different CRC, define these values in "Wiced-SDK/platforms/<platform>/platform_ota2_image.h"
 *  NOTE: changing the CRC routine requires re-building
 *        the OTA2 Image Creation Utility Program mk_ota2_imagexx.exe !!!
 * OTA2_CRC_INIT_VALUE
 * OTA2_CRC_FUNCTION
 * OTA2_CRC_VAR
 * OTA2_CRC_HTON
 * OTA2_CRC_NTOH
 *
 */
#ifndef OTA2_CRC_INIT_VALUE
#include "../../utilities/crc/crc.h"
#define OTA2_CRC_INIT_VALUE                              CRC32_INIT_VALUE
#define OTA2_CRC_FUNCTION(address, size, previous_value) (uint32_t)crc32(address, size, previous_value)
typedef uint32_t    OTA2_CRC_VAR;
#define OTA2_CRC_HTON(value)                             htonl(value)
#define OTA2_CRC_NTOH(value)                             ntohl(value)
#endif

/* This needs to support both WIN32 and WICED
 * Adding the header files gets complicated, just make sure these are defined
 */
#ifndef __SWAP32__
#define __SWAP32__(val) ( (uint32_t) ((((val) & 0xFF000000) >> 24 ) | (((val) & 0x00FF0000) >> 8) \
             | (((val) & 0x0000FF00) << 8) | (((val) & 0x000000FF) << 24)) )

#define __SWAP16__(val) ( (uint16_t) ((((val) & 0xFF00) >> 8) | (((val) & 0x00FF) << 8)))


#ifndef htonl
#define htonl(val)  __SWAP32__(val)
#endif /* htonl */
#ifndef ntohl
#define ntohl(val)  __SWAP32__(val)
#endif /* htonl */

#ifndef htons
#define htons(val)  __SWAP16__(val)
#endif /*htons */

#ifndef ntohs
#define ntohs(val)  __SWAP16__(val)
#endif /*htons */
#endif /* ifndef htonl */

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_OTA2_IMAGE_VERSION             0x01

#define WICED_OTA2_PLATFORM_NAME_LEN        32

#define WICED_OTA2_IMAGE_MAGIC_STRING        "OTAimage"
#define WICED_OTA2_IMAGE_MAGIC_STR_LEN       8

#define WICED_OTA2_IMAGE_COMPONENT_NAME_LEN  32
#define WICED_OTA2_IMAGE_SECURE_SIGN_LEN     64


/* this is the same as the SFLASH sector size */
#ifndef SECTOR_SIZE
#define SECTOR_SIZE                     (4096)
#endif

/* let's get the FLASH base address - These are system addresses, not offsets! */
#define OTA_FLASH_CHIP_BASE              SI_SFLASH

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    OTA2_BOOT_NEVER_RUN_BEFORE = 0,     /* Do not change this ENUM location */
    OTA2_BOOT_NORMAL,                   /* Do not change this ENUM location */
    /* Use these after failsafe recovery to continue the ota2 extraction preocess */
    OTA2_BOOT_EXTRACT_FACTORY_RESET,    /* Renamed in SDK-4.0.1, but stays the same ENUM as SDK OTA2_BOOT_FACTORY_RESET */
    OTA2_BOOT_EXTRACT_UPDATE,           /* Renamed in SDK-4.0.1, but stays the same ENUM as SDK OTA2_BOOT_FACTORY_RESET */
    OTA2_BOOT_SOFTAP_UPDATE,            /* Do not change this ENUM location */
    OTA2_BOOT_LAST_KNOWN_GOOD,          /* Do not change this ENUM location */

    OTA2_BOOT_FACTORY_RESET,
    OTA2_BOOT_UPDATE,

    /* Use these before starting an extraction. If extraction is interrupted, use failsafe */
    OTA2_BOOT_FAILSAFE_FACTORY_RESET,
    OTA2_BOOT_FAILSAFE_UPDATE,

    OTA2_MAX_BOOT_TYPES     /* Not a valid boot type */
} ota2_boot_type_t;


typedef enum
{
    WICED_OTA2_IMAGE_TYPE_NONE  = 0,
    WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP,
    WICED_OTA2_IMAGE_TYPE_CURRENT_APP,
    WICED_OTA2_IMAGE_TYPE_LAST_KNOWN_GOOD,
    WICED_OTA2_IMAGE_TYPE_STAGED
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    ,
    WICED_OTA2_IMAGE_TYPE_FS
#endif
} wiced_ota2_image_type_t;

typedef enum {
    WICED_OTA2_IMAGE_INVALID             = 0,
    WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS,
    WICED_OTA2_IMAGE_DOWNLOAD_FAILED,
    WICED_OTA2_IMAGE_DOWNLOAD_UNSUPPORTED,
    WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE,
    WICED_OTA2_IMAGE_VALID,
    WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT,
    WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED,
} wiced_ota2_image_status_t;

typedef enum
{
    WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK = 0,
    WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST,
} ota2_image_swap_type_t;

typedef enum {
    WICED_OTA2_IMAGE_SIGN_NONE       = 0,
    WICED_OTA2_IMAGE_SIGN_CRC,
    WICED_OTA2_IMAGE_SIGN_SHA,

} wiced_ota2_image_sign_type_t;

typedef enum {
    WICED_OTA2_IMAGE_COMPONENT_LUT       = 0,
    WICED_OTA2_IMAGE_COMPONENT_FR_APP,
    WICED_OTA2_IMAGE_COMPONENT_DCT,
    WICED_OTA2_IMAGE_COMPONENT_OTA_APP,
    WICED_OTA2_IMAGE_COMPONENT_FILESYSTEM,
    WICED_OTA2_IMAGE_COMPONENT_WIFI_FW,
    WICED_OTA2_IMAGE_COMPONENT_APP0,
    WICED_OTA2_IMAGE_COMPONENT_APP1,
    WICED_OTA2_IMAGE_COMPONENT_APP2,
    WICED_OTA2_IMAGE_COMPONENT_APP0_XIP
} wiced_ota2_image_component_type_t;

typedef enum {
    WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_NONE       = 0,
    WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_LZW,
    WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_GZIP,
    WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_BZ2,
} wiced_ota2_image_component_compress_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#pragma pack(1)

typedef struct wiced_ota2_component_s {
        uint8_t         type;               /* wiced_ota2_image_component_type_t                */
        uint8_t         compression;        /* wiced_ota2_image_component_compress_t            */
        OTA2_CRC_VAR    crc;                /* crc on uncompressed component data               */
        uint32_t        source_offset;      /* offset within OTA Image Component Data section   */
        uint32_t        source_size;        /* size of data in OTA Image                        */
        uint32_t        destination;        /* absolute offset of destination in FLASH         */
        uint32_t        destination_size;   /* size of data                                     */
        uint8_t         name[WICED_OTA2_IMAGE_COMPONENT_NAME_LEN];   /* component name           */
} wiced_ota2_image_component_t;

typedef struct wiced_ota2_header_s {
        uint16_t        ota2_version;               /* OTA2 Image Version  (version of this format)                         */
        uint16_t        major_version;              /* Software Version Major (version of software contained in image)      */
        uint16_t        minor_version;              /* Software Version Minor (version of software contained in image)      */
        uint8_t         platform_name[WICED_OTA2_PLATFORM_NAME_LEN];    /* Platform name (31 char + NULL)                   */
        uint16_t        download_status;            /* Status of image download                                             */
        uint32_t        bytes_received;             /* bytes received (valid for WICED_OTA2_DOWNLOAD_IN_PROGRESS and
                                                                             WICED_OTA2_DOWNLOAD_COMPLETE)                  */
        uint8_t         magic_string[WICED_OTA2_IMAGE_MAGIC_STR_LEN];    /* Magic string "OTAImage"                         */

        OTA2_CRC_VAR    header_crc;                 /* CRC of OTA header and component headers,
                                                     excluding header_crc, download_status and bytes_received               */
        uint16_t        secure_sign_type;           /* Secure signature type                                                */
        uint8_t         secure_signature[WICED_OTA2_IMAGE_SECURE_SIGN_LEN]; /* depends on secure_sign_type  up to 256 bit   */

        uint32_t        image_size;                 /* total size of OTA image (including headers)                          */

        uint16_t        component_count;            /* number of components in the component list
                                                   (component list directly follows this structure)                         */
        uint32_t        data_start;                 /* offset in this file to start of data                                 */
} wiced_ota2_image_header_t;

#pragma pack()

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static inline void wiced_ota2_image_header_swap_network_order(wiced_ota2_image_header_t *ota2_header,
                                                              ota2_image_swap_type_t host_to_network )
{
    if (host_to_network == WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK)
    {
        /* convert 16 & 32 bit values to network order */
        ota2_header->ota2_version     = htons(ota2_header->ota2_version);
        ota2_header->major_version    = htons(ota2_header->major_version);
        ota2_header->minor_version    = htons(ota2_header->minor_version);
        ota2_header->download_status  = htons(ota2_header->download_status);
        ota2_header->bytes_received   = htonl(ota2_header->bytes_received);
        ota2_header->header_crc       = htonl(ota2_header->header_crc);
        ota2_header->secure_sign_type = htons(ota2_header->secure_sign_type);
        ota2_header->image_size       = htonl(ota2_header->image_size);
        ota2_header->component_count  = htons(ota2_header->component_count);
        ota2_header->data_start       = htonl(ota2_header->data_start);
    }
    else
    {
        /* convert 16 & 32 bit values to host order */
        ota2_header->ota2_version     = ntohs(ota2_header->ota2_version);
        ota2_header->major_version    = ntohs(ota2_header->major_version);
        ota2_header->minor_version    = ntohs(ota2_header->minor_version);
        ota2_header->download_status  = ntohs(ota2_header->download_status);
        ota2_header->bytes_received   = ntohl(ota2_header->bytes_received);
        ota2_header->header_crc       = ntohl(ota2_header->header_crc);
        ota2_header->secure_sign_type = ntohs(ota2_header->secure_sign_type);
        ota2_header->image_size       = ntohl(ota2_header->image_size);
        ota2_header->component_count  = ntohs(ota2_header->component_count);
        ota2_header->data_start       = ntohl(ota2_header->data_start);
    }
}
static inline void wiced_ota2_image_component_header_swap_network_order(wiced_ota2_image_component_t *component_header,
                                                                        ota2_image_swap_type_t host_to_network )
{
    if (host_to_network == WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK)
    {
        /* convert 16 & 32 bit values to network order */
        component_header->crc              = OTA2_CRC_HTON(component_header->crc);
        component_header->source_offset    = htonl(component_header->source_offset);
        component_header->source_size      = htonl(component_header->source_size);
        component_header->destination      = htonl(component_header->destination);
        component_header->destination_size = htonl(component_header->destination_size);

    }
    else
    {
        /* convert 16 & 32 bit values to host order */
        component_header->crc              = OTA2_CRC_NTOH(component_header->crc);
        component_header->source_offset    = ntohl(component_header->source_offset);
        component_header->source_size      = ntohl(component_header->source_size);
        component_header->destination      = ntohl(component_header->destination);
        component_header->destination_size = ntohl(component_header->destination_size);
    }
}

/**
 * Simple validation of the OTA Image
 *
 * Checks header version, magic string, size, # components
 *
 * @param[in]  ota_type     - OTA Image type
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_validate ( wiced_ota2_image_type_t ota_type );

/**
 * Get status of OTA Image at download location
 *
 * @param[in]  ota_type     - OTA Image type
 * @param[out] status       - Receives the OTA Image status.
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_get_status ( wiced_ota2_image_type_t ota_type, wiced_ota2_image_status_t *status );

/**
 * Extract OTA2 extractor (or Apps LUT) from the OTA2 Image to the current area
 *
 * NOTE: This is used by the OTA2 Failsafe code
 *
 * @param[in]  ota_type     - OTA Image type
 * @param[in]  component    - OTA Component to extract (ONLY WICED_OTA2_IMAGE_COMPONENT_LUT or WICED_OTA2_IMAGE_COMPONENT_OTA_APP)
 * @param[out] destination  - address extracted to in FLASH
 * @param[out] destination  - extracted size
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image, not fully downloaded
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_extract_uncompressed_component( wiced_ota2_image_type_t ota_type, wiced_ota2_image_component_type_t component,
                                                                uint32_t* destination, uint32_t* destination_size );

/**
 * Extract OTA Image to the current area
 * NOTE: All information regarding destination of data in the system is part of the OTA Image.
 *
 * @param[in]  ota_type     - OTA Image type
 * @param[in]  image_size   - Size of the OTA Image
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image, not fully downloaded
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_extract ( wiced_ota2_image_type_t ota_type );

/**
 * Write OTA Image to the Staging area (WICED_OTA2_IMAGE_TYPE_STAGED)
 * NOTE: The total size of the OTA image is included in a valid OTA image header.
 *       This function will update the status in the OTA image header by calling
 *       wiced_ota2_update_header() TODO: make this platform-specific
 *
 * @param[in]  data      - pointer to part or all of an OTA image to be stored in the staging area
 * @param[in]  offset    - offset from start of staging area to store this data
 * @param[in]  size      - size of the data to store
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_image_write_data(uint8_t* data, uint32_t offset, uint32_t size);

/** Get the OTA2 application software version from the header
 *
 * NOTE: Only updates the major & minor values on WICED_SUCCESS
 *
 * @param image_type [in]   : OTA2 Image type to get software app version
 * @param major [out]       : ptr to store software major version
 * @param minor [out]       : ptr to store software minor version
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_get_version(wiced_ota2_image_type_t image_type, uint16_t* major, uint16_t* minor);

/** Update the OTA image header after writing (parts of) the downloaded OTA image to FLASH  TODO: make this platform-specific
 *
 * @param delta_written - number of bytes written to the image
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_update_staged_header(uint32_t delta_written);

/** Update the OTA image header status
 *
 * @param total_bytes_received - number of bytes written to the image  TODO: make this platform-specific
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_update_staged_status(wiced_ota2_image_status_t new_status);

/** Call this to set the flag to force a facory reset on reboot
 *  NOTE: This is equal to holding the "factory reset button" for 5 seconds.
 *          Use this if there is no button on the system.
 *
 * @param   N/A
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_force_factory_reset_on_reboot( void );

/** Get the last boot type - did we update or have a factory reset?
 *
 * @param   N/A
 *
 * @return  ota2_boot_type_t
 */
ota2_boot_type_t wiced_ota2_get_boot_type( void );

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS

/** Set the current image type - this info is used for deciding writing area
 *
 * @param image_type [in]   : OTA2 Image type
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t  wiced_ota2_set_current_update (wiced_ota2_image_type_t image_type);
#endif

/* debugging only */
wiced_result_t wiced_ota2_image_fakery(wiced_ota2_image_status_t new_status);

#ifdef __cplusplus
} /*extern "C" */
#endif

