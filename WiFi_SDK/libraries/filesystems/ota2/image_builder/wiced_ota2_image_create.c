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
 *  Wiced OTA Image File Creation
 *
 *  This code creates a file that contains the components needed for an
 *  Over The Air Update of the entire Application and Resources for any product.
 *
 *  Process
 *      Read in and parse the configuration file
 *      Check file sizes for components and create file layout
 *      Copy the files from their sources to the OTA Image file
 *      Verify the OTA Image file
 *
 *
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
 *
 *  wiced_ota2_image_header_t            - info about the OTA Image
 *  Start of OTA Image header (also start of file)
 * +========================================================================================+
 * | uint16_t     | ota_version     |  OTA Image Version  (version of this format)          |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint16_t     | software_compatibility |  Software Compatibility (TBD)                  |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint16_t     | hardware_compatibility |  Hardware Compatibility (TBD)                  |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint16_t     | download_status |  Status of image download                             |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | bytes_received  |  bytes received  - Only valid for                     |
 * |              |                 |     WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS             |
 * |              |                 |     WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE)               |
 * |              |                 |     WICED_OTA2_IMAGE_EXTRACT_ON_REBOOT)               |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint8_t      | magic_string    |  Magic string "WicedOTA"                              |
 * +--------------+-----------------+-------------------------------------------------------+
 * | OTA2_CRC_VAR | header_crc      |  CRC of OTA header and component headers              |
 * |              |                 |      excludes:    header_crc                          |
 * |              |                 |                   download_status                     |
 * |              |                 |                   bytes_received                      |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint16_t     | secure_sign_type|  Secure signature type                                |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint8_t      | secure_signature|  Depending on secure_sign_type, up to 256 bit         |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | image_size      |  Total size of OTA image (including all headers)      |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint16_t     | component_count |  Number of components in the component list           |
 * |              |                 |    component headers directly follows this structure  |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | data_start      |  Offset in this file to start of data                 |
 * +========================================================================================+
 *
 *  wiced_ota2_image_component_t         - array of individual component info structures
 *  Individual Component headers
 * +========================================================================================+
 * | uint8_t      | type            |  wiced_ota_component_type_t                           |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint8_t      | compression     |  wiced_ota_component_compress_t                       |
 * +--------------+-----------------+-------------------------------------------------------+
 * | OTA2_CRC_VAR | crc             |  crc on uncompressed component data                   |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | source_offset   |  offset within OTA Image Component Data section       |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | source_size     |  size of data in OTA Image                            |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | destination     |  absolute offset of destination in FLASH              |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint32_t     | destination_size|  size of data                                         |
 * +--------------+-----------------+-------------------------------------------------------+
 * | uint8_t      | name            |  component name                                       |
 * +========================================================================================+
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <inttypes.h>
#include <sys/stat.h>

#include "wiced_result.h"

/* Include header for common structure defn between Builder and Extractor */

#include "wiced_ota2_image.h"
#include "wiced_ota2_image_create.h"

#include "platform_dct.h"

/******************************************************
 *                      Macros
 ******************************************************/
#if defined(OSX)

#define _stati64 stat

#elif !defined(_WIN32)
/*define_style_exception_start*/
#define off64_t __off64_t
#define _stati64 stat64
/*define_style_exception_end*/
#endif /* ifndef _WIN32 */


/******************************************************
 *                    Constants
 ******************************************************/
#define WICED_OTA_BUILD_MAX_PC_PATH     256

#define WICED_OTA_COPY_BUFFER_SIZE    (4 * 1024)       /* for copying components temporary for crc testing - match WICED sector size */

/* Strings to parse form the configuration file */
#define PLATFORM_NAME_STR           "PLATFORM_NAME"
#define APPS_LUT_LOC_STR            "APPS_LUT_LOC"
#define APPS_LUT_FILE_STR           "APPS_LUT_FILE"
#define FR_APP_LOC_STR              "FR_APP_LOC"
#define FR_APP_FILE_STR             "FR_APP_FILE"
#define DCT_LOC_STR                 "DCT_LOC"
#define DCT_FILE_STR                "DCT_FILE"
#define OTA_APP_LOC_STR             "OTA_APP_LOC"
#define OTA_APP_FILE_STR            "OTA_APP_FILE"
#define FILESYSTEM_LOC_STR          "FILESYSTEM_LOC"
#define FILESYSTEM_FILE_STR         "FILESYSTEM_FILE"
#define WIFI_FIRMWARE_LOC_STR       "WIFI_FIRMWARE_LOC"
#define WIFI_FIRMWARE_FILE_STR      "WIFI_FIRMWARE_FILE"
#define APPLICATION_0_LOC_STR       "APPLICATION_0_LOC"
#define APPLICATION_0_FILE_STR      "APPLICATION_0_FILE"
#define APPLICATION_1_LOC_STR       "APPLICATION_1_LOC"
#define APPLICATION_1_FILE_STR      "APPLICATION_1_FILE"
#define APPLICATION_2_LOC_STR       "APPLICATION_2_LOC"
#define APPLICATION_2_FILE_STR      "APPLICATION_2_FILE"

#define APPLICATION_0_XIP_LOC_STR   "APPLICATION_0_XIP_LOC"
#define APPLICATION_0_XIP_FILE_STR  "APPLICATION_0_XIP_FILE"

/******************************************************
 *                   Enumerations
 ******************************************************/
/* These are based on values in WICED/platform/include/platform_dct.h
 * We use idx 0 for the LUT table itself,
 * and the other values are the indexes in that file + 1
 */
typedef enum {
    WICED_OTA2_IMAGE_APPS_LUT_INDEX          = 0,
    WICED_OTA2_IMAGE_FR_APP_INDEX            = DCT_FR_APP_INDEX + 1,
    WICED_OTA2_IMAGE_DCT_IMAGE_INDEX         = DCT_DCT_IMAGE_INDEX + 1,
    WICED_OTA2_IMAGE_OTA_APP_INDEX           = DCT_OTA_APP_INDEX + 1,
    WICED_OTA2_IMAGE_FILESYSTEM_IMAGE_INDEX  = DCT_FILESYSTEM_IMAGE_INDEX + 1,
    WICED_OTA2_IMAGE_WIFI_FIRMWARE_INDEX     = DCT_WIFI_FIRMWARE_INDEX + 1,
    WICED_OTA2_IMAGE_APP0_INDEX              = DCT_APP0_INDEX + 1,
    WICED_OTA2_IMAGE_APP1_INDEX              = DCT_APP1_INDEX + 1,
    WICED_OTA2_IMAGE_APP2_INDEX              = DCT_APP2_INDEX + 1,
    WICED_OTA2_IMAGE_APP0_XIP_INDEX,
    WICED_OTA2_IMAGE_COMPONENT_MAX   /* must be last ! */
} wiced_ota2_image_image_idx_t;

#define WICED_OTA2_IMAGE_HEADERS_SIZE_MAX  ((uint32_t)(sizeof(wiced_ota2_image_header_t) + \
                                                       (sizeof(wiced_ota2_image_component_t) * WICED_OTA2_IMAGE_COMPONENT_MAX)))
#define WICED_OTA2_IMAGE_DATA_START_OFFSET ((uint32_t)(WICED_OTA2_IMAGE_HEADERS_SIZE_MAX + \
                                                       (SECTOR_SIZE - (WICED_OTA2_IMAGE_HEADERS_SIZE_MAX % SECTOR_SIZE))))

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct wiced_ota2_image_builder_component_s {
        wiced_ota2_image_component_t component;

        const char* loc_str;
        const char* file_str;

        /* extra fields for builder */
        uint32_t    pc_fs_size;         /* size in PC file system    */
        char        pc_fs_path[WICED_OTA_BUILD_MAX_PC_PATH];    /* path in pc file system   */

        int         present_in_config_file;
        size_t      sector_fill;
} wiced_ota2_image_builder_component_t;

typedef struct wiced_ota2_image_builder_info_s {
        int         verbose;
        int         factory_reset_image;

        uint16_t    major_version;
        uint16_t    minor_version;
        char        platform_name[WICED_OTA2_PLATFORM_NAME_LEN];

        /* spec file name & buffer */
        const char* spec_file_path;
        uint8_t*    spec_file_buff;

        uint16_t                            component_count;    /* actual # of components written to output file */
        wiced_ota2_image_builder_component_t build_components[WICED_OTA2_IMAGE_COMPONENT_MAX];

        /* the output buffer and pointers into the output buffer for various structs & data */
        const char*                 ota2_image_file_path;
        uint32_t                    ota2_image_size;
        wiced_ota2_image_header_t    ota_create_header;                                     /* OTA Image header     */
        wiced_ota2_image_component_t ota_create_components[WICED_OTA2_IMAGE_COMPONENT_MAX];  /* component info array */


        /* read back info for verifying the write */
        wiced_ota2_image_header_t            ota_read_ota_header;                                    /* OTA Image header     */
        wiced_ota2_image_component_t         ota_read_components[WICED_OTA2_IMAGE_COMPONENT_MAX]; /* component info array */

        /* used to copy the components from the PC source file to the OTA Image file */
        uint8_t                     component_copy_buffer[WICED_OTA_COPY_BUFFER_SIZE];

        /* used to verify the components from the OTA Image file */
        uint8_t                     component_read_buffer[WICED_OTA_COPY_BUFFER_SIZE];

} wiced_ota2_image_builder_info_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static int ota_configuration_parse( wiced_ota2_image_builder_info_t *ota_build_info );
static int ota_layout_ota2_image( wiced_ota2_image_builder_info_t *ota_inf );
static int ota_check_for_component_overlap( wiced_ota2_image_builder_info_t *ota_inf );
static int ota_write_ota2_image( wiced_ota2_image_builder_info_t *ota_inf );

static int ota_verify_ota2_image( wiced_ota2_image_builder_info_t *ota_inf );

static void print_ota_header_info(wiced_ota2_image_header_t *ota_header);
static int ota_print_ota_file_header_info( wiced_ota2_image_builder_info_t *ota_inf );

static int ota_build_print_info( wiced_ota2_image_builder_info_t *ota_inf, const char *message );
static void ota2_image_copy_present_info_to_create_structs(wiced_ota2_image_builder_info_t *ota_inf);

static int64_t fsize               ( const char* filename );

/******************************************************
 *               Variable Definitions
 ******************************************************/


static wiced_ota2_image_builder_component_t default_build_components[WICED_OTA2_IMAGE_COMPONENT_MAX] =
{
    [WICED_OTA2_IMAGE_APPS_LUT_INDEX] = {
        .component.name = "Apps LUT",
        .loc_str = APPS_LUT_LOC_STR,
        .file_str = APPS_LUT_FILE_STR,
    },
    [WICED_OTA2_IMAGE_FR_APP_INDEX] = {
        .component.name = "FR App",
        .loc_str = FR_APP_LOC_STR,
        .file_str = FR_APP_FILE_STR,
    },
    [WICED_OTA2_IMAGE_DCT_IMAGE_INDEX] = {
        .component.name = "DCT file",
        .loc_str = DCT_LOC_STR,
        .file_str = DCT_FILE_STR,
    },
    [WICED_OTA2_IMAGE_OTA_APP_INDEX] = {

        .component.name = "OTA App",
        .loc_str = OTA_APP_LOC_STR,
        .file_str = OTA_APP_FILE_STR,
    },
    [WICED_OTA2_IMAGE_FILESYSTEM_IMAGE_INDEX] = {
        .component.name = "File system",
        .loc_str = FILESYSTEM_LOC_STR,
        .file_str = FILESYSTEM_FILE_STR,
    },
    [WICED_OTA2_IMAGE_WIFI_FIRMWARE_INDEX] = {
        .component.name = "WiFi Firmware",
        .loc_str = WIFI_FIRMWARE_LOC_STR,
        .file_str = WIFI_FIRMWARE_FILE_STR,
    },
    [WICED_OTA2_IMAGE_APP0_INDEX] = {
        .component.name = "Application 0",
        .loc_str = APPLICATION_0_LOC_STR,
        .file_str = APPLICATION_0_FILE_STR,
    },
    [WICED_OTA2_IMAGE_APP1_INDEX] = {
        .component.name = "Application 1",
        .loc_str = APPLICATION_1_LOC_STR,
        .file_str = APPLICATION_1_FILE_STR,
    },
    [WICED_OTA2_IMAGE_APP2_INDEX] = {
        .component.name = "Application 2",
        .loc_str = APPLICATION_2_LOC_STR,
        .file_str = APPLICATION_2_FILE_STR,
    },
    [WICED_OTA2_IMAGE_APP0_XIP_INDEX] = {
        .component.name = "Application 0 XIP",
        .loc_str = APPLICATION_0_XIP_LOC_STR,
        .file_str = APPLICATION_0_XIP_FILE_STR,
    },
};

/******************************************************
 *               Function Definitions
 ******************************************************/

int create_wiced_ota2( const char* ota_spec_file, const char* ota2_image_name, int verbose )
{
    wiced_ota2_image_builder_info_t      *ota_build_info;    /* structure to hold OTA Image build info */

    /* Allocate our build info structure (including copy & verify buffer) */
    ota_build_info = malloc(sizeof(wiced_ota2_image_builder_info_t));
    if( ota_build_info == NULL )
    {
        printf( "Couldn't Allocate the OTA build info and output buffer structure!\n" );
        return -2;
    }
    memset(ota_build_info, 0x00, sizeof(wiced_ota2_image_builder_info_t));

    ota_build_info->verbose = verbose;
    ota_build_info->factory_reset_image = 0;    /* assume normal image */

    ota_build_info->spec_file_path = ota_spec_file;
    ota_build_info->ota2_image_file_path = ota2_image_name;

    /* component.name defaults - they are overwritten when present in the config file */
    memcpy(ota_build_info->build_components, default_build_components, sizeof(default_build_components));

    /* read and Parse the OTA configuration file */
    if( ota_configuration_parse( ota_build_info ) != 0 )
    {
        printf( "ota_configuration_parse() FAILED !\n" );
        return -2;
    }

    /* layout the image - this determines sizes of the components, offsets within the output file */
    ota_layout_ota2_image( ota_build_info );

    /* report info */
    if (ota_build_info->verbose != 0)
        ota_build_print_info( ota_build_info, "Layout Finished:" );

    /* verify no overlaps */
    if (ota_check_for_component_overlap( ota_build_info ) != 0)
    {
        return -2;
    }

    /* write the image */
    ota_write_ota2_image( ota_build_info );

    printf( "Written !!\n" );

    /* verify the image*/
    if( ota_verify_ota2_image( ota_build_info ) == 0)
    {
        printf( "Verified !!\n" );

        /* report info */
        if (ota_build_info->verbose != 0)
        {
            ota_build_print_info( ota_build_info, "Verified:" );
        }

        return 0;
    }
    else
    {
        printf( "Verify FAILED !!\n" );
    }

    return -1;
}




/******************************************************
 *               Static Function Definitions
 ******************************************************/

#if defined(LINUX)
static int64_t fsize( const char *filename )
{
    struct stat st;
    if (stat(filename, &st) == 0)
    {
        return st.st_size;
    }
    return -1;
}
#else
 static int64_t fsize( const char *filename )
{
    struct _stati64 st;
    if (_stati64(filename, &st) == 0)
    {
        return st.st_size;
    }
    return -1;
}
#endif

 /* Utilities copied from wiced_utilities.c so we do not include that in the build */
static inline char hexchar_to_nibble( char hexchar, uint8_t* nibble )
{
    if ( ( hexchar >= '0' ) && ( hexchar <= '9' ) )
    {
        *nibble = (uint8_t)( hexchar - '0' );
        return 0;
    }
    else if ( ( hexchar >= 'A' ) && ( hexchar <= 'F' ) )
    {
        *nibble = (uint8_t) ( hexchar - 'A' + 10 );
        return 0;
    }
    else if ( ( hexchar >= 'a' ) && ( hexchar <= 'f' ) )
    {
        *nibble = (uint8_t) ( hexchar - 'a' + 10 );
        return 0;
    }
    return -1;
}

 static size_t string_to_generic( const char* string, size_t str_length,  uint32_t* value_out, uint8_t is_unsigned, uint8_t is_hex )
 {
     uint8_t nibble;
     size_t characters_processed = 0;

     if (( string == NULL ) || (value_out == NULL))
     {
         return 0;
     }

     *value_out = 0;

     while ( ( characters_processed != str_length ) &&
             ( 0 == hexchar_to_nibble( *string, &nibble ) ) &&
             ( ( is_hex != 0 ) || ( nibble < 10 ) )
           )
     {
         if ( is_hex != 0 )
         {
             if ( ( ( *value_out > ( 0x7fffffff >> 4 ) ) && ( is_unsigned == 0 ) ) ||
                  ( *value_out > ( 0xffffffff >> 4 ) )
                )
             {
                 break;
             }
             *value_out = ( *value_out << 4 ) + nibble;
         }
         else
         {
             if ( ( ( *value_out > ( 0x7fffffff / 10 ) ) && ( is_unsigned == 0 ) ) ||
                  ( *value_out > ( 0xffffffff / 10 ) )
                )
             {
                 break;
             }
             *value_out = ( *value_out * 10 ) + nibble;
         }
         string++;
         characters_processed++;
     }

     return characters_processed;
 }

static uint32_t generic_string_to_unsigned( const char* str )
{
    uint32_t val = 0;
    uint8_t is_hex = 0;

    if ( strncmp( str, "0x", 2 ) == 0 )
    {
        is_hex = 1;
        str += 2;
    }

    string_to_generic( str, strlen(str),  &val, 1, is_hex );

    return val;
}

static int ota_build_parse_location(uint8_t* buff, wiced_ota2_image_builder_component_t *info)
{
    char *tag_ptr, *value_ptr;

    if (info == NULL)
    {
        return -1;
    }

    /* get location value */
    tag_ptr = strstr((char*)buff, info->loc_str);
    if (tag_ptr != NULL)
    {
        value_ptr = strchr(tag_ptr,'=');
        if (value_ptr != NULL)
        {
            info->component.destination = (uint32_t)generic_string_to_unsigned( (value_ptr + 1) );
            info->present_in_config_file++;
            return 0;
        }
    }
    return -1;
}

static int ota_build_parse_filepath(uint8_t* buff, wiced_ota2_image_builder_component_t *info)
{
    char *tag_ptr, *value_ptr, *eol_ptr;

    if (info == NULL)
    {
        return -1;
    }

    /* get file path */
    tag_ptr = strstr((char*)buff, info->file_str);
    if (tag_ptr != NULL)
    {
        value_ptr = strchr(tag_ptr,'=');
        if (value_ptr != NULL)
        {
            eol_ptr = strchr(value_ptr,'\r');
            if (eol_ptr == NULL)
            {
                eol_ptr = strchr(value_ptr, '\n');
            }
            if ((eol_ptr != NULL) && ((eol_ptr - value_ptr) > 1) && ((eol_ptr - value_ptr) < WICED_OTA_BUILD_MAX_PC_PATH))
            {
                size_t  name_len;
                char* fn_ptr;
                value_ptr++;

                if ((eol_ptr - value_ptr) >= WICED_OTA_BUILD_MAX_PC_PATH)
                {
                    printf("PARSE OTA: %s (or value) path TOO LONG!\n", info->file_str);
                    return -1;
                }

                memcpy(info->pc_fs_path, value_ptr, (size_t)(eol_ptr - value_ptr));
                /* get pc file size */
                info->pc_fs_size = (uint32_t)fsize( info->pc_fs_path );
                /* get just the filename for our component name back up from eol_ptr to first '\\' */
                fn_ptr = eol_ptr;
                while ((fn_ptr > value_ptr) && (*fn_ptr != '\\') && (*fn_ptr != '/')) fn_ptr--;
                fn_ptr++;   /* we don't want the slash */
                memset(info->component.name, 0x00, sizeof(info->component.name));
                name_len = (size_t)(eol_ptr - fn_ptr);
                if (name_len >= WICED_OTA2_IMAGE_COMPONENT_NAME_LEN)
                {
                    name_len = WICED_OTA2_IMAGE_COMPONENT_NAME_LEN - 1;
                }
                memcpy(info->component.name, fn_ptr, name_len);
                info->present_in_config_file++;
                return 0;
            }
        }
    }
    return -1;
}

static int16_t find_and_parse_int( const char* buffer, const char *tag)
{
    char *tag_ptr;
    char *value_ptr;

    if ( (buffer == NULL) || (tag == NULL))
    {
        return -1;
    }
    tag_ptr = strstr(buffer, tag);
    if (tag_ptr != NULL)
    {
        value_ptr = strchr(tag_ptr,'=');
        if (value_ptr != NULL)
        {
            return (int16_t)generic_string_to_unsigned( (value_ptr + 1) );
        }
    }
    return 0;
}

/*
 * Parse the configuration file written by the build process (wiced_apps.mk)
 *
 */
static int ota_configuration_parse( wiced_ota2_image_builder_info_t *ota_inf )
{
    size_t      config_file_size;
    FILE*       config_file_handle = NULL;
    size_t      bytes_read;
    int         retval = -2;    /* assume failure */
    int         idx;
    char        *platform_name_string;

    if( ota_inf == NULL )
    {
        printf( "ota_configuration_parse(0 ota_inf == NULL\n" );
        goto _parse_fail_exit;
    }

    /* get OTA Image configuration file size, malloc buffer, open and read */
    config_file_size = (size_t)fsize( ota_inf->spec_file_path );
    if (config_file_size <= 0)
    {
        printf( "ota_configuration_parse() configuration file size %d!\n", config_file_size );
        goto _parse_fail_exit;
    }

    ota_inf->spec_file_buff = malloc(config_file_size);
    if (ota_inf->spec_file_buff == NULL)
    {
        printf( "ota_configuration_parse() malloc spec file buffer failed !\n" );
        goto _parse_fail_exit;
    }
    memset(ota_inf->spec_file_buff, 0x00, config_file_size);

    config_file_handle = fopen( ota_inf->spec_file_path, "r+b" );
    if( config_file_handle == NULL )
    {
        printf( "ota_configuration_parse() Couldn't open configuration file %s\n", ota_inf->spec_file_path );
        goto _parse_fail_exit;
    }

    bytes_read = (uint32_t) fread( ota_inf->spec_file_buff, 1, config_file_size, config_file_handle );
    if (bytes_read != config_file_size)
    {
        printf( "ota_configuration_parse() Couldn't open configuration file %s\n", ota_inf->spec_file_path );
        goto _parse_fail_exit;
    }

    /* are we a factory reset image? */
    ota_inf->factory_reset_image = (uint16_t)find_and_parse_int((char*)ota_inf->spec_file_buff, "FACTORY_RESET");

    /* Parse the values */
    ota_inf->major_version = (uint16_t)find_and_parse_int((char*)ota_inf->spec_file_buff, "MAJOR_VERSION");
    ota_inf->minor_version = (uint16_t)find_and_parse_int((char*)ota_inf->spec_file_buff, "MINOR_VERSION");

    memset(ota_inf->platform_name, 0x00, sizeof(ota_inf->platform_name));
    platform_name_string = strstr((char*)ota_inf->spec_file_buff, PLATFORM_NAME_STR);
    if (platform_name_string != NULL)
    {
        char *eol_ptr;
        platform_name_string = strchr(platform_name_string,'=');
        if (platform_name_string != NULL)
        {
            platform_name_string++;
        }

        eol_ptr = strchr(platform_name_string,'\r');
        if (eol_ptr == NULL)
        {
            eol_ptr = strchr(platform_name_string, '\n');
        }

        if ((platform_name_string != NULL) && (platform_name_string < eol_ptr))
        {
            size_t len = (size_t)(eol_ptr - platform_name_string);
            if (len >= WICED_OTA2_PLATFORM_NAME_LEN)
            {
                len = WICED_OTA2_PLATFORM_NAME_LEN - 1;
            }

            memcpy( ota_inf->platform_name, platform_name_string, len);
        }
    }

    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        ota_build_parse_location(ota_inf->spec_file_buff, &ota_inf->build_components[idx] );
        ota_build_parse_filepath(ota_inf->spec_file_buff, &ota_inf->build_components[idx] );

        if (ota_inf->build_components[idx].present_in_config_file != 2)
        {
            if (ota_inf->verbose != 0)
                printf( "%s Not included in OTA Image\n", ota_inf->build_components[idx].file_str );

            ota_inf->build_components[idx].present_in_config_file = 0;
        }
    }

    goto _parse_exit;

_parse_fail_exit:

    if (ota_inf->spec_file_buff != NULL)
    {
        free(ota_inf->spec_file_buff);
    }
    ota_inf->spec_file_buff = NULL;

_parse_exit:
    /* Close the Wiced OTA image spec file */
    if (config_file_handle != NULL)
    {
        retval = fclose( config_file_handle );
        if ( retval != 0 )
        {
            printf( "ota_configuration_parse() Error closing config file\n" );
        }
    }

    return retval;
}

/*
 *  Layout the files linearly for the output file
 */
static int ota_layout_ota2_image( wiced_ota2_image_builder_info_t *ota_inf )
{
    uint16_t    idx, component_count;
    uint32_t    offset_into_data_area;

    if( ota_inf == NULL )
    {
        printf( "ota_layout_ota2_image() ota_inf == NULL\n" );
        return -2;
    }

    /* count the actual components */
    component_count = 0;
    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
            component_count++;
    }
    if ((component_count < 1) || (component_count > WICED_OTA2_IMAGE_COMPONENT_MAX))
    {
        printf( "ota_layout_ota2_image() bad component count %d\n", component_count);
        return -2;
    }

    ota_inf->component_count = component_count;
    offset_into_data_area = 0;                  /* into data area always starts as 0 */

    /* build info about the components */
    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
        {
            switch( idx )
            {
            case WICED_OTA2_IMAGE_APPS_LUT_INDEX:
                ota_inf->build_components[idx].component.type = WICED_OTA2_IMAGE_COMPONENT_LUT;
                break;

            case WICED_OTA2_IMAGE_DCT_IMAGE_INDEX:
                ota_inf->build_components[idx].component.type = WICED_OTA2_IMAGE_COMPONENT_DCT;
                break;

            case WICED_OTA2_IMAGE_FILESYSTEM_IMAGE_INDEX:
                ota_inf->build_components[idx].component.type = WICED_OTA2_IMAGE_COMPONENT_FILESYSTEM;
                break;

            case WICED_OTA2_IMAGE_APP0_INDEX:
                ota_inf->build_components[idx].component.type = WICED_OTA2_IMAGE_COMPONENT_APP0;
                break;

            case WICED_OTA2_IMAGE_APP0_XIP_INDEX:
                ota_inf->build_components[idx].component.type = WICED_OTA2_IMAGE_COMPONENT_APP0_XIP;
                break;
            default:
                ota_inf->build_components[idx].component.type = (uint8_t)idx;
                break;
            }

            ota_inf->build_components[idx].component.compression = 0;  /* TODO */

            ota_inf->build_components[idx].component.source_offset = offset_into_data_area;
            ota_inf->build_components[idx].component.source_size  = ota_inf->build_components[idx].pc_fs_size;

            /* ota_inf->build_components[idx].component.destination gathered from spec file */
            ota_inf->build_components[idx].component.destination_size = ota_inf->build_components[idx].pc_fs_size;

            /* todo: if we are compressing individual components, we need to compress source before we get here
             * and figure out the crc from the uncompressed source */
            offset_into_data_area += ota_inf->build_components[idx].component.source_size;

            ota_inf->build_components[idx].sector_fill = 0;

            /* Round up to next sector */
            if ((offset_into_data_area % SECTOR_SIZE) != 0)
            {
                ota_inf->build_components[idx].sector_fill = SECTOR_SIZE - (offset_into_data_area % SECTOR_SIZE);
            }
            offset_into_data_area += (uint32_t)ota_inf->build_components[idx].sector_fill;    /* round up to 4k sector size */

        }
    }
    ota_inf->ota2_image_size = WICED_OTA2_IMAGE_DATA_START_OFFSET + offset_into_data_area;

    ota2_image_copy_present_info_to_create_structs(ota_inf);

    return 0;
}

/*
 * Read the OTA Image header (not the components) from the OTA Image file
 *
 */
static size_t  wiced_ota2_image_read_ota_header(FILE* infile, wiced_ota2_image_builder_info_t *ota_inf )
{
    /* seek & read the ota header data */
    if ( fseek( infile, 0, SEEK_SET) != 0)
    {
        printf( "Error seeking to read OTA Header\n");
        return 0;
    }

    if(fread( (char*)&ota_inf->ota_read_ota_header, sizeof(wiced_ota2_image_header_t), 1, infile ) != 1)
    {
        printf( "Error reading OTA Header\n");
        return 0;
    }

    /* network to host */
    wiced_ota2_image_header_swap_network_order(&ota_inf->ota_read_ota_header, WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST );
    return sizeof(wiced_ota2_image_header_t);
}

/*
 * Write the OTA Image header (not the components) to the OTA Image file
 *
 */
static size_t  ota2_image_write_ota_header(FILE* outfile, wiced_ota2_image_builder_info_t *ota_inf )
{
    /* seek & write the ota header data */
    if ( fseek( outfile, 0, SEEK_SET) != 0)
    {
        printf( "Error seeking to OTA Header\n");
        return 0;
    }

    /* host to network */
    wiced_ota2_image_header_swap_network_order(&ota_inf->ota_create_header, WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK );
    if (fwrite( (char*)&ota_inf->ota_create_header, sizeof(wiced_ota2_image_header_t), 1, outfile ) != 1)
    {
        printf( "Error Writing OTA Header\n");
        return 0;
    }

    return sizeof(wiced_ota2_image_header_t);
}


/*
 * Read the Component headers from the OTA Image file
 *
 */
static size_t  ota2_image_read_component_headers(FILE* infile, wiced_ota2_image_builder_info_t *ota_inf )
{
    int idx;
    size_t read_size;

    /* seek & read the ota component header data */
    if ( fseek( infile, sizeof(wiced_ota2_image_header_t), SEEK_SET) != 0)
    {
        printf( "Error seeking to read OTA component Header\n");
        return 0;
    }

    read_size = sizeof(wiced_ota2_image_component_t) * ota_inf->ota_read_ota_header.component_count;
    if(fread( (char*)&ota_inf->ota_read_components, read_size, 1, infile ) != 1)
    {
        printf( "Error reading OTA component Header\n");
        return 0;
    }

    /* network to host */
    for (idx = 0; idx < ota_inf->ota_read_ota_header.component_count; idx++)
    {
        wiced_ota2_image_component_header_swap_network_order(&ota_inf->ota_read_components[idx], WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST );
    }
    return read_size;
}


/*
 * Write the Component headers to the OTA Image file
 *
 */
static size_t  ota2_image_write_component_headers(FILE* outfile, wiced_ota2_image_builder_info_t *ota_inf)
{
    int idx;
    size_t write_size;

    /* seek & write the component header data */
    if ( fseek( outfile, sizeof(wiced_ota2_image_header_t), SEEK_SET) != 0)
    {
        printf( "Error seeking to OTA Component Header\n");
        return 0;
    }

    /* network to host */
    for (idx = 0; idx < ota_inf->component_count; idx++)
    {
        wiced_ota2_image_component_header_swap_network_order(&ota_inf->ota_create_components[idx], WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK);
    }

    write_size = sizeof(wiced_ota2_image_component_t) * ota_inf->component_count;
    if (fwrite( (char*)&ota_inf->ota_create_components, write_size, 1, outfile ) != 1)
    {
        printf( "Error Writing OTA Component Headers\n");
        return 0;
    }

    return write_size;
}

/*
 * Compute the CRC for the headers
 *
 */
static OTA2_CRC_VAR ota_compute_ota_header_crc( wiced_ota2_image_builder_info_t *ota_inf )
{
    wiced_ota2_image_header_t*       ota_header;
    wiced_ota2_image_component_t*    ota_component_header;

    OTA2_CRC_VAR     header_crc;
    uint32_t        i;

    ota_header = &ota_inf->ota_create_header;

    /* make sure the download_status, bytes_received, and header_crc are cleared */
    ota_header->header_crc       = 0;
    if (ota_inf->factory_reset_image == 0)
    {
        ota_header->download_status = WICED_OTA2_IMAGE_INVALID;    /* this is changed only when the Image is downloading */
        ota_header->bytes_received   = 0;
    }
    else
    {
        ota_header->download_status = WICED_OTA2_IMAGE_VALID;    /* unless it is a factory reset image */
        ota_header->bytes_received   = ota_header->image_size;
    }
    header_crc = OTA2_CRC_INIT_VALUE;
    header_crc = OTA2_CRC_FUNCTION((uint8_t*)ota_header, sizeof(wiced_ota2_image_header_t), header_crc);

    for (i = 0; i < ota_header->component_count; i++)
    {
        ota_component_header = &ota_inf->ota_create_components[i];
        header_crc = OTA2_CRC_FUNCTION((uint8_t*)ota_component_header, sizeof(wiced_ota2_image_component_t), header_crc);
    }

    return header_crc;
}

/*
 *  Update the component info from the Build structure to the Create Structure
 */
static void ota2_image_copy_present_info_to_create_structs(wiced_ota2_image_builder_info_t *ota_inf)
{
    uint16_t idx, count;

    /* create OTA Image header */
    ota_inf->ota_create_header.ota2_version    = WICED_OTA2_IMAGE_VERSION;
    ota_inf->ota_create_header.major_version   = ota_inf->major_version;
    ota_inf->ota_create_header.minor_version   = ota_inf->minor_version;
    memcpy( &ota_inf->ota_create_header.platform_name, ota_inf->platform_name, sizeof(ota_inf->ota_create_header.platform_name));
    if (ota_inf->factory_reset_image == 0)
    {
        ota_inf->ota_create_header.download_status = WICED_OTA2_IMAGE_INVALID;    /* this is changed only when the Image is downloading */
        ota_inf->ota_create_header.bytes_received  = 0;
    }
    else
    {
        ota_inf->ota_create_header.download_status = WICED_OTA2_IMAGE_VALID;    /* unless it is a factory reset image */
        ota_inf->ota_create_header.bytes_received  = ota_inf->ota_create_header.image_size;
//        printf("Factory Reset - setting status %d & bytes_received 0x%lx!\r\n", ota_inf->ota_create_header.download_status, ota_inf->ota_create_header.bytes_received);
    }
    memcpy(&ota_inf->ota_create_header.magic_string, WICED_OTA2_IMAGE_MAGIC_STRING, WICED_OTA2_IMAGE_MAGIC_STR_LEN);
    ota_inf->ota_create_header.header_crc          = 0;
    ota_inf->ota_create_header.secure_sign_type    = 0x00;       /* TODO: */
    /* ota_inf->ota_create_header.secure_signature = 0x00;        TODO: */

    /* update actual component count in the ota_header */
    ota_inf->ota_create_header.component_count = ota_inf->component_count;
    ota_inf->ota_create_header.image_size = ota_inf->ota2_image_size;
    ota_inf->ota_create_header.data_start = WICED_OTA2_IMAGE_DATA_START_OFFSET;

    count = 0;
    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file != 0)
        {
            memcpy( &ota_inf->ota_create_components[count],
                    &ota_inf->build_components[idx].component, sizeof(wiced_ota2_image_component_t));
            count++;
        }
    }

    /* sanity check */
    if (ota_inf->component_count != count )
    {
        printf("ERROR: ota2_image_copy_present_info_to_create_structs() BAD COMPONENT COUNT %d != %d\n", ota_inf->component_count, count);
        exit(-1);
    }

    ota_inf->ota_create_header.header_crc = ota_compute_ota_header_crc(ota_inf);
}

static int ota_check_for_component_overlap( wiced_ota2_image_builder_info_t *ota_inf )
{
    int     retval = 0;
    int     idx, inner_idx;

    if( ota_inf == NULL )
    {
        printf( "ota_write_ota2_image() ota_inf == NULL\n" );
        return -2;
    }

    if (ota_inf->verbose != 0)
        printf( "Checking for component overlap in OTA Image File %s\n", ota_inf->ota2_image_file_path );

    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
        {
            uint32_t start_addr, padded_size, end_addr;

            /* start and end address of component we are checking against */
            start_addr = ota_inf->build_components[idx].component.destination;
            end_addr = ota_inf->build_components[idx].component.destination + ota_inf->build_components[idx].component.destination_size;

            /* round up to next sector */
            if ((end_addr % SECTOR_SIZE) != 0)
            {
                end_addr += SECTOR_SIZE - (end_addr % SECTOR_SIZE);
            }
            padded_size = end_addr - start_addr;

            if (ota_inf->verbose != 0)
                printf( "start: 0x%08lx size: 0x%08lx   end: 0x%08lx   idx:%d  %s\n", start_addr, padded_size, end_addr, idx, ota_inf->build_components[idx].file_str );

            /* check position/size against other components */
            for (inner_idx = idx + 1; inner_idx < WICED_OTA2_IMAGE_COMPONENT_MAX; inner_idx++)
            {
                if ( ota_inf->build_components[inner_idx].present_in_config_file > 0)
                {
                    uint32_t inner_start_addr, inner_padded_size, inner_end_addr;
                    inner_start_addr = ota_inf->build_components[inner_idx].component.destination;
                    inner_end_addr = ota_inf->build_components[inner_idx].component.destination + ota_inf->build_components[inner_idx].component.destination_size;

                    /* round up to next sector */
                    if ((inner_end_addr % SECTOR_SIZE) != 0)
                    {
                        inner_end_addr += SECTOR_SIZE - (inner_end_addr % SECTOR_SIZE);
                    }
                    inner_padded_size = inner_end_addr - inner_start_addr;

                    if ( ((inner_start_addr >= start_addr) && (inner_start_addr < end_addr)) ||
                         ((start_addr >= inner_start_addr) && (start_addr < inner_end_addr)) )
                    {
                        uint32_t overlap = 0;

                        if ((inner_start_addr >= start_addr) && (inner_start_addr < end_addr))
                        {
                            overlap = end_addr - inner_start_addr;

                        }
                        if ((start_addr >= inner_start_addr) && (start_addr < inner_end_addr))
                        {
                            overlap = inner_end_addr - start_addr;
                        }

                        /* overlap here (indexes are 1 less than in the default_build_components, used for names) */
                        printf( "\n     OTA2 component %s overlaps %s by 0x%lx bytes Adjust platform's \"ota2_image_defines.mk file\" !!\n", ota_inf->build_components[idx].file_str, ota_inf->build_components[inner_idx].file_str, overlap );
                        printf( "          start: 0x%08lx padded size: 0x%08lx  end: 0x%08lx :: %s (%s)\n", start_addr, padded_size, end_addr, ota_inf->build_components[idx].file_str, ota_inf->build_components[idx].pc_fs_path );
                        printf( "          start: 0x%08lx padded size: 0x%08lx  end: 0x%08lx :: %s (%s)\n\n", inner_start_addr, inner_padded_size, inner_end_addr, ota_inf->build_components[inner_idx].file_str, ota_inf->build_components[inner_idx].pc_fs_path );
                        retval = -1;
                    }
                }
            }
        }
    }

    return retval;
}

/*
 * Write out the OTA Image file
 */
static int ota_write_ota2_image( wiced_ota2_image_builder_info_t *ota_inf )
{
    FILE*   component_handle = NULL;
    FILE*   ota_file_handle = NULL;
    int     retval, idx, component_count;
    size_t  ota_header_size, component_header_size;
    size_t  bytes_read, bytes_to_write, total_written, chunk_size;

    if( ota_inf == NULL )
    {
        printf( "ota_write_ota2_image() ota_inf == NULL\n" );
        return -2;
    }

    total_written = 0;

    /* Open the Wiced OTA Image file for writing */
    ota_file_handle = fopen( ota_inf->ota2_image_file_path, "w+b" );
    if( ota_file_handle == NULL )
    {
        printf( "Couldn't open output file %s\n", ota_inf->ota2_image_file_path );
        return -2;
    }

    /* write out the OTA Image pieces! */
    if (ota_inf->verbose != 0)
        printf( "Writing out OTA Image File %s\n", ota_inf->ota2_image_file_path );

    /* update component info */
    ota2_image_copy_present_info_to_create_structs(ota_inf);

    /* write OTA Image headers */
    ota_header_size = ota2_image_write_ota_header(ota_file_handle , ota_inf);
    if (ota_header_size == 0)
    {
        printf( "Error Writing OTA Image File - OTA Header! \n");
        goto _write_fail_exit;
    }
    total_written += ota_header_size;
    if (ota_inf->verbose != 0)
        printf( "  Writing OTA Image OTA Header %d -> %d  ftell():%d! \n", ota_header_size, total_written, ftell(ota_file_handle));

    component_header_size = ota2_image_write_component_headers(ota_file_handle , ota_inf);
    if (component_header_size == 0)
    {
        printf( "Error Writing OTA Image File - OTA Component Headers! \n");
        goto _write_fail_exit;
    }
    total_written += component_header_size;
    if (ota_inf->verbose != 0)
        printf( "  Writing OTA Image COMPONENT Header %d -> %d  ftell():%d! \n", component_header_size, total_written, ftell(ota_file_handle));

    /* write 0x00 up to next sector */
    chunk_size =  WICED_OTA2_IMAGE_DATA_START_OFFSET - total_written;
    if (ota_inf->verbose != 0)
        printf( "  Writing Pad to next sector %d -> %d  ftell():%d! \n", chunk_size, total_written, ftell(ota_file_handle));

    memset(ota_inf->component_copy_buffer, 0x00, chunk_size);
    retval = (int)fwrite( ota_inf->component_copy_buffer, chunk_size, 1, ota_file_handle );
    if ( retval != 1)
    {
        printf( "Error writing to fill initial sector \n" );
        retval = -2;
        goto _write_fail_exit;
    }
    total_written += chunk_size;
    if (ota_inf->verbose != 0)
        printf( "  Writing Pad to next sector %d -> %d  ftell():%d! \n", chunk_size, total_written, ftell(ota_file_handle));

    /* write out individual components */
    component_count = 0;
    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
        {
            component_handle = fopen( ota_inf->build_components[idx].pc_fs_path, "r+b" );
            if ( component_handle == NULL )
            {
                printf( "ERROR -- Couldn't open component file %s\n", ota_inf->build_components[idx].pc_fs_path );
                goto _write_fail_exit;
            }

            /* calculate crc on uncompressed data while copying */
            ota_inf->build_components[idx].component.crc = OTA2_CRC_INIT_VALUE;

            bytes_to_write = ota_inf->build_components[idx].pc_fs_size;

            if (ota_inf->verbose != 0)
                printf( "  OTA Component %d %s\n", bytes_to_write, ota_inf->build_components[idx].pc_fs_path );

            retval = fseek( ota_file_handle, (long int)(WICED_OTA2_IMAGE_DATA_START_OFFSET + ota_inf->build_components[idx].component.source_offset), SEEK_SET);
            while (bytes_to_write > 0)
            {
                /* start with 0x0 */
                memset(ota_inf->component_copy_buffer, 0x0, WICED_OTA_COPY_BUFFER_SIZE);

                chunk_size = bytes_to_write;
                if (chunk_size > WICED_OTA_COPY_BUFFER_SIZE )
                {
                    chunk_size = WICED_OTA_COPY_BUFFER_SIZE;
                }

                bytes_read = fread(ota_inf->component_copy_buffer, chunk_size, 1, component_handle);
                if ( bytes_read != 1 )
                {
                    printf( "Error reading component %s\n", ota_inf->build_components[idx].pc_fs_path);
                    retval = -2;
                    goto _write_fail_exit;
                }

                ota_inf->build_components[idx].component.crc = OTA2_CRC_FUNCTION(ota_inf->component_copy_buffer, (uint32_t)chunk_size,
                                                                 ota_inf->build_components[idx].component.crc);

                retval = (int)fwrite( ota_inf->component_copy_buffer, chunk_size, 1, ota_file_handle );
                if ( retval != 1)
                {
                    printf( "Error writing component %s\n", ota_inf->build_components[idx].file_str );
                    retval = -2;
                    goto _write_fail_exit;
                }

                bytes_to_write -= chunk_size;
                total_written += chunk_size;
            } /* while bytes to write */

            if (ota_inf->verbose != 0)
                printf( "  Writing OTA component %d -> %d  ftell():%d %s! \n", ota_inf->build_components[idx].pc_fs_size, total_written, ftell(ota_file_handle), ota_inf->build_components[idx].file_str);

            printf( "  Writing OTA component crc:0x%lx %s! \n", ota_inf->build_components[idx].component.crc, ota_inf->build_components[idx].file_str);

            /* close component file */
            retval = fclose( component_handle );
            if ( retval != 0 )
            {
                printf( "Error closing component %s\n", ota_inf->build_components[idx].file_str);
            }
            component_handle = NULL;

            /* fill in the rest of the sector in the OTA Image File */
            memset(ota_inf->component_copy_buffer, 0, WICED_OTA_COPY_BUFFER_SIZE);

            bytes_to_write = ota_inf->build_components[idx].sector_fill;
            if (ota_inf->verbose != 0)
                printf( "  fill to next sector %d\n", bytes_to_write );

            while (bytes_to_write > 0)
            {
                chunk_size = bytes_to_write;
                if (chunk_size > WICED_OTA_COPY_BUFFER_SIZE )
                {
                    chunk_size = WICED_OTA_COPY_BUFFER_SIZE;
                }

                retval = (int)fwrite( ota_inf->component_copy_buffer, chunk_size, 1, ota_file_handle );
                if ( retval != 1)
                {
                    printf( "Error writing sector fill for component %s\n", ota_inf->build_components[idx].pc_fs_size );
                    retval = -2;
                    goto _write_fail_exit;
                }
                bytes_to_write -= chunk_size;
                total_written += chunk_size;
            }

            if (ota_inf->verbose != 0)
                    printf( "    Component %s done size:%d crc:0x%lx\n", ota_inf->build_components[idx].component.name,
                            ota_inf->build_components[idx].component.destination_size, ota_inf->build_components[idx].component.crc);

            component_count++;
        }   /* if in configuration file */
    } /* idx loop */

    if (ota_inf->verbose != 0)
    {
        printf( "  Done total:%d \n", total_written);
        printf( "  Re-Write OTA Header & Components total:%d \n", total_written);
    }

    /* copy any updated info  (like crc) & re-write headers */
    ota2_image_copy_present_info_to_create_structs(ota_inf);
    ota_header_size = ota2_image_write_ota_header(ota_file_handle , ota_inf);
    if (ota_header_size == 0)
    {
        printf( "Error Writing OTA Image File - OTA Header! \n");
        goto _write_fail_exit;
    }
    component_header_size = ota2_image_write_component_headers(ota_file_handle , ota_inf);
    if (component_header_size == 0)
    {
        printf( "Error Writing OTA Image File - OTA Component Headers! \n");
        goto _write_fail_exit;
    }

_write_fail_exit:

    /* close component file if still open */
    if (component_handle != NULL )
    {
        retval = fclose( component_handle );
        if ( retval != 0 )
        {
            printf( "Error closing output\n" );
        }
    }

    /* Close the Wiced OTA image file */
    if (ota_file_handle != NULL )
    {
        fflush(ota_file_handle);
        retval = fclose( ota_file_handle );
        if ( retval != 0 )
        {
            printf( "Error closing output\n" );
        }
    }

    if (ota_inf->verbose != 0) printf("Total bytes written: %d\n", total_written );
    return 0;
}

/*
 * Verify the OTA file:
 *  - reading the headers & checking the version & CRC
 *  - Read each individual component, and check CRC
 */
static int ota_verify_ota2_image( wiced_ota2_image_builder_info_t *ota_inf )
{
    FILE*    ota_file_handle;
    FILE*    component_handle;
    size_t   chunk_size, ota_file_size;
    int      idx, component_count, retval;
    size_t   file_size, bytes_to_read;
    size_t   ota_header_size, component_header_size;
    long     offset, ota_offset;
    OTA2_CRC_VAR file_crc;
    OTA2_CRC_VAR ota_crc;
    OTA2_CRC_VAR header_crc, save_header_crc;

    if( ota_inf == NULL )
    {
        printf( "ota_verify_ota2_image() ota_inf == NULL\n" );
        return -2;
    }

    printf( "Verifying ...\n" );

    retval = 0;
    ota_file_handle = NULL;
    component_handle = NULL;

    /* read in the header */
    /* check image file size */
    ota_file_size = (size_t)fsize( ota_inf->ota2_image_file_path );
    if ( ota_file_size < sizeof(wiced_ota2_image_header_t))
    {
        printf( "Input file size too small! %d %s\n", ota_file_size, ota_inf->ota2_image_file_path );
        return -2;
    }

    /* Open the Wiced OTA Image file for reading */
    ota_file_handle = fopen( ota_inf->ota2_image_file_path, "r+b" );
    if( ota_file_handle == NULL )
    {
        printf( "Couldn't open input file %s\n", ota_inf->ota2_image_file_path );
        return -2;
    }

    if (ota_inf->verbose != 0) printf( "Verifying OTA Image File %s\n", ota_inf->ota2_image_file_path );

    /* Read OTA Image header */
    ota_header_size = wiced_ota2_image_read_ota_header( ota_file_handle, ota_inf );
    if (ota_inf->verbose != 0) printf( "  ota_header_size: %d\n", ota_header_size );

    /* read component info */
    component_header_size = ota2_image_read_component_headers( ota_file_handle, ota_inf);
    if (ota_inf->verbose != 0) printf( "  component_header_size: %d\n", component_header_size );

    /* check the OTA header CRC */
    save_header_crc = ota_inf->ota_read_ota_header.header_crc;
    ota_inf->ota_read_ota_header.header_crc      = 0;
    /* for FACTORY reset image, we compute with download status in place */
    if (ota_inf->factory_reset_image == 0)
    {
        ota_inf->ota_read_ota_header.download_status = WICED_OTA2_IMAGE_INVALID;
        ota_inf->ota_read_ota_header.bytes_received  = 0;
    }
    else
    {
        ota_inf->ota_read_ota_header.download_status = WICED_OTA2_IMAGE_VALID;    /* unless it is a factory reset image */
        ota_inf->ota_read_ota_header.bytes_received  = ota_inf->ota_read_ota_header.image_size;
    }
    header_crc = OTA2_CRC_INIT_VALUE;
    header_crc = OTA2_CRC_FUNCTION((uint8_t*)&ota_inf->ota_read_ota_header, sizeof(wiced_ota2_image_header_t), header_crc);
    for (idx = 0; idx < ota_inf->ota_read_ota_header.component_count; idx++)
    {
        header_crc = OTA2_CRC_FUNCTION((uint8_t*)&ota_inf->ota_read_components[idx], sizeof(wiced_ota2_image_component_t), header_crc);
    }
    ota_inf->ota_read_ota_header.header_crc = save_header_crc;

    if (save_header_crc != header_crc )
    {
        printf("OTA Header CRC does not match!  file: 0x%lx != computed: 0x%lx\n", save_header_crc, header_crc );
        ota_print_ota_file_header_info(ota_inf);
        retval = -3;
        goto _read_fail_exit;
    }


    /* go through the components we marked used in creation & check them vs. the output file */
    component_count = 0;
    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
        {
            if (ota_inf->verbose != 0) printf( "idx:%d %d %s\n", idx, component_count, ota_inf->build_components[idx].file_str);
            /* verify component file size */
            file_size = (size_t)fsize( ota_inf->build_components[idx].pc_fs_path );
            if ((file_size != ota_inf->build_components[idx].pc_fs_size) ||
                (file_size != ota_inf->ota_read_components[component_count].source_size))
            {
                printf( "ERROR --File size mismatch! PC source:%d build header:%d OTA file:%d %s\n",
                        file_size,
                        ota_inf->build_components[idx].pc_fs_size,
                        ota_inf->ota_read_components[component_count].source_size,
                        ota_inf->ota2_image_file_path );
            }


            /* open the source file from the PC */
            component_handle = fopen( ota_inf->build_components[idx].pc_fs_path, "r+b" );
            if ( component_handle == NULL )
            {
                printf( "ERROR -- Couldn't open component file %s\n", ota_inf->build_components[idx].pc_fs_path );
                goto _read_fail_exit;
            }


            /* TODO: This code only works for uncompressed components.
             * For compressed components, the component will need to be decompressed
             * before checking against the source.
             */

            /* point to the component in the OTA image file */
            offset = 0;
            ota_offset = (long)(WICED_OTA2_IMAGE_DATA_START_OFFSET + ota_inf->ota_read_components[component_count].source_offset);

            file_crc = OTA2_CRC_INIT_VALUE;
            ota_crc = OTA2_CRC_INIT_VALUE;

            bytes_to_read = ota_inf->build_components[idx].pc_fs_size;
            if (ota_inf->verbose != 0)
                printf( "  OTA Component %d %d %s\n", offset, bytes_to_read, ota_inf->build_components[idx].pc_fs_path );

            while (bytes_to_read > 0)
            {
                memset(ota_inf->component_copy_buffer, 0, sizeof(ota_inf->component_copy_buffer));
                memset(ota_inf->component_read_buffer, 0, sizeof(ota_inf->component_read_buffer));

                chunk_size = bytes_to_read;
                if (chunk_size > WICED_OTA_COPY_BUFFER_SIZE )
                {
                    chunk_size = WICED_OTA_COPY_BUFFER_SIZE;
                }

                /* read the PC source file of the component */
                retval = fseek( component_handle, offset, SEEK_SET);
                if ( retval == -1 )
                {
                    printf( "Error -- fseek component %s\n", ota_inf->build_components[idx].pc_fs_path);
                    retval = -2;
                    goto _read_fail_exit;
                }
                retval = (int)fread(ota_inf->component_copy_buffer, chunk_size, 1, component_handle);
                if ( retval != 1 )
                {
                    printf( "Error -- verify read component %s\n", ota_inf->build_components[idx].pc_fs_path);
                    retval = -2;
                    goto _read_fail_exit;
                }

                /* read from the OTA Image file */
                retval = fseek( ota_file_handle, ota_offset, SEEK_SET);
                if ( retval == -1 )
                {
                    printf( "Error -- fseek component %s\n", ota_inf->build_components[idx].pc_fs_path);
                    retval = -2;
                    goto _read_fail_exit;
                }
                retval = (int)fread( ota_inf->component_read_buffer, chunk_size, 1, ota_file_handle );
                if ( retval != 1)
                {
                    printf( "Error -- reading component %s\n", ota_inf->build_components[idx].pc_fs_size );
                    retval = -2;
                    goto _read_fail_exit;
                }

                file_crc = OTA2_CRC_FUNCTION(ota_inf->component_copy_buffer, (uint32_t)chunk_size, file_crc);
                ota_crc = OTA2_CRC_FUNCTION(ota_inf->component_read_buffer, (uint32_t)chunk_size, ota_crc);

                /* compare the data */
                if ((memcmp(ota_inf->component_copy_buffer, ota_inf->component_read_buffer, chunk_size) != 0) ||
                    (file_crc != ota_crc))
                {
                    int i, j;
                    printf( "Error -- Component mismatch PC file <-> OTA Image file offset:%d(0x%x) ota:%d(0x%x) %s\n", offset, file_crc, ota_offset, ota_crc, ota_inf->build_components[idx].pc_fs_path);
                    for (i = 0; i < (int)chunk_size; i+= 16)
                    {
                        printf( "PC file : ");
                        for (j = 0; (j < 16) && ((i+j) < (int)chunk_size); j++)
                        {
                            printf( "0x%02x ", ota_inf->component_copy_buffer[i+j]);
                        }
                        printf( "\n");

                        printf( "OTA file: ");
                        for (j = 0; (j < 16) && ((i+j) < (int)chunk_size); j++)
                        {
                            printf( "0x%02x ", ota_inf->component_read_buffer[i+j]);
                        }
                        printf( "\n");

                        for (j = 0; (j < 16) && ((i+j) < (int)chunk_size); j++)
                        {
                            if (ota_inf->component_copy_buffer[i+j] != ota_inf->component_read_buffer[i+j])
                            {
                                retval = -1;
                                goto _read_fail_exit;
                            }
                        }
                    }
                }

                bytes_to_read -= chunk_size;
                offset += (long)chunk_size;
                ota_offset += (long)chunk_size;
            } /* while bytes to read */

            /* close component file */
            retval = fclose( component_handle );
            if ( retval != 0 )
            {
                printf( "Error closing component %s\n", ota_inf->build_components[idx].pc_fs_size);
            }
            component_handle = NULL;

            if ((file_crc != ota_crc) ||
                (ota_inf->ota_read_components[component_count].crc != ota_crc) ||
                (ota_inf->build_components[idx].component.crc != ota_crc))
            {
                printf(" ERROR - CRCs do not match! file:0x%lx ota_file:0x%lx comp header:0x%lx build header:0x%x %s %s\n",
                        file_crc, ota_crc,
                        ota_inf->ota_read_components[component_count].crc,
                        ota_inf->build_components[idx].component.crc,
                        ota_inf->ota_read_components[component_count].name,
                        ota_inf->build_components[idx].component.name );
            }
            component_count++;
        }
    }

    retval = 0;

_read_fail_exit:

    /* Close the Wiced OTA image file */
    if (component_handle != NULL )
    {
        retval = fclose( component_handle );
        if ( retval != 0 )
        {
            printf( "Error closing component %s\n", ota_inf->build_components[idx].pc_fs_size);
        }
    }
    component_handle = NULL;

    /* Close the Wiced OTA image file */
    if (ota_file_handle != NULL )
    {
        retval = fclose( ota_file_handle );
        if ( retval != 0 )
        {
            printf( "Error closing output\n" );
        }
    }

    return retval;
}

static void print_component_info(wiced_ota2_image_component_t *component)
{
    printf("          name: %s\n", component->name);
    printf("          type: %d\n", component->type);
    printf("          comp: %d\n", component->compression);
    printf("           crc: 0x%08lx\n", component->crc);
    printf("      src_size: %8d 0x%x\n", component->source_size, component->source_size);
    printf("       src_off: %8d 0x%x\n", component->source_offset, component->source_offset);
    printf("      dst_size: %8d 0x%x\n", component->destination_size, component->destination_size);
    printf("          dest: %8d 0x%x\n", component->destination, component->destination);
}

static void print_ota_header_info(wiced_ota2_image_header_t *ota_header)
{
    printf("       ota_version: %d\n", ota_header->ota2_version);
    printf("    software_major: %d\n", ota_header->major_version);
    printf("    software_minor: %d\n", ota_header->minor_version);
    printf("            status: %d\n", ota_header->download_status);
    printf("    bytes_received: %d\n", ota_header->bytes_received);
    printf("      magic_string: %c%c%c%c%c%c%c%c\n", ota_header->magic_string[0], ota_header->magic_string[1],
                                                 ota_header->magic_string[2], ota_header->magic_string[3],
                                                 ota_header->magic_string[4], ota_header->magic_string[5],
                                                 ota_header->magic_string[6], ota_header->magic_string[7] );
    printf("        header_crc: 0x%lx\n", ota_header->header_crc);
    printf("       secure_type: %d\n", ota_header->secure_sign_type);
    printf("  secure_signature: TODO \n"); /* TODO */
    printf("        image_size: %d\n", ota_header->image_size);
    printf("   component_count: %d\n", ota_header->component_count);
}

static int ota_print_ota_file_header_info( wiced_ota2_image_builder_info_t *ota_inf )
{
    int idx;
    if( ota_inf == NULL )
    {
        return -2;
    }

    printf("OTA Image Information (read from out File): %s\n", ota_inf->ota2_image_file_path);
    print_ota_header_info( &ota_inf->ota_read_ota_header);

    for (idx = 0; (idx < ota_inf->ota_read_ota_header.component_count) && (idx < WICED_OTA2_IMAGE_COMPONENT_MAX); idx++)
    {
        print_component_info( &ota_inf->ota_read_components[idx]);
    }

    return 0;
}


static int ota_build_print_info( wiced_ota2_image_builder_info_t *ota_inf, const char* message )
{
    int idx;
    if( ota_inf == NULL )
    {
        return -2;
    }

    printf("OTA Image Information %s: %s\n", ((message != NULL) ? message : " " ), ota_inf->ota2_image_file_path);
    printf("MAX OTA Header size = (%d) + (8 * %d) = %d\r\n", sizeof(wiced_ota2_image_header_t), sizeof(wiced_ota2_image_component_t),
            sizeof(wiced_ota2_image_header_t) + (8 * sizeof(wiced_ota2_image_component_t)) );
    print_ota_header_info( &ota_inf->ota_create_header);

    for (idx = 0; idx < WICED_OTA2_IMAGE_COMPONENT_MAX; idx++)
    {
        if (ota_inf->build_components[idx].present_in_config_file > 0)
        {
            printf("  component %d: %s\n", idx, ota_inf->build_components[idx].component.name);
            printf("          path: %s\n", ota_inf->build_components[idx].pc_fs_path);
            printf("       pc size: %8d\n", ota_inf->build_components[idx].pc_fs_size);
            print_component_info( &ota_inf->build_components[idx].component);
        }
    }

    return 0;
}

