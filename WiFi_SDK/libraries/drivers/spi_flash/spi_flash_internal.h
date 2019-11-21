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
#ifndef INCLUDED_SPI_FLASH_INTERNAL_H
#define INCLUDED_SPI_FLASH_INTERNAL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "spi_flash.h"
#include <stdint.h>

#ifndef SFLASH_SUPPORT_MACRONIX_PARTS
#define SFLASH_SUPPORT_MACRONIX_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_ISSI_LP_PARTS
#define SFLASH_SUPPORT_ISSI_LP_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_ISSI_CQ_PARTS
#define SFLASH_SUPPORT_ISSI_CQ_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_MICRON_PARTS
#define SFLASH_SUPPORT_MICRON_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_WINBOND_PARTS
#define SFLASH_SUPPORT_WINBOND_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_SST_PARTS
#define SFLASH_SUPPORT_SST_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_EON_PARTS
#define SFLASH_SUPPORT_EON_PARTS    0
#endif

#ifndef SFLASH_SUPPORT_CY_PARTS
#define SFLASH_SUPPORT_CY_PARTS  0
#endif

/* Status Register bit definitions */
#define SFLASH_STATUS_REGISTER_BUSY                          ( (unsigned char) 0x01 )
#define SFLASH_STATUS_REGISTER_WRITE_ENABLED                 ( (unsigned char) 0x02 )
#define SFLASH_STATUS_REGISTER_BLOCK_PROTECTED_0             ( (unsigned char) 0x04 )
#define SFLASH_STATUS_REGISTER_BLOCK_PROTECTED_1             ( (unsigned char) 0x08 )
#define SFLASH_STATUS_REGISTER_BLOCK_PROTECTED_2             ( (unsigned char) 0x10 ) /* SST & Macronix Only */
#define SFLASH_STATUS_REGISTER_BLOCK_PROTECTED_3             ( (unsigned char) 0x20 ) /* SST & Macronix Only */
#define SFLASH_STATUS_REGISTER_AUTO_ADDRESS_INCREMENT        ( (unsigned char) 0x40 ) /* SST Only */
#define SFLASH_STATUS_REGISTER_BLOCK_PROTECT_BITS_READ_ONLY  ( (unsigned char) 0x80 ) /* SST Only */
#define SFLASH_STATUS_REGISTER_QUAD_ENABLE                   ( (unsigned char) 0x40 ) /* Macronix Only */
#define SFLASH_STATUS_REGISTER_WRITE_PROTECT_PIN_ENABLE      ( (unsigned char) 0x80 ) /* Macronix Only */

/* MICRON - Enhanced Volatile Configuration Register bit definitions */
#define MICRON_SFLASH_ENH_VOLATILE_STATUS_REGISTER_HOLD      ( (unsigned char) 0x10 ) /* HOLD# */

/* WINBOND - Status Register-2 bit definitions */
#define WINBOND_SFLASH_STATUS_REGISTER2_QUAD_ENABLE          ( (unsigned char) 0x02 ) /* Quad Enable*/

/* Command definitions */
typedef enum
{

    SFLASH_WRITE_STATUS_REGISTER        = 0x01, /* WRSR                   */
    SFLASH_WRITE                        = 0x02,
    SFLASH_READ                         = 0x03,
    SFLASH_WRITE_DISABLE                = 0x04, /* WRDI                   */
    SFLASH_READ_STATUS_REGISTER         = 0x05, /* RDSR                   */
    SFLASH_WRITE_ENABLE                 = 0x06, /* WREN                   */
    SFLASH_FAST_READ                    = 0x0B,
    SFLASH_SECTOR_ERASE                 = 0x20, /* SE                     */
    SFLASH_READ_STATUS_REGISTER2        = 0x35, /* RDSR-2 - Winbond only  */
    SFLASH_BLOCK_ERASE_MID              = 0x52, /* SE                     */
    SFLASH_BLOCK_ERASE_LARGE            = 0xD8, /* SE                     */
    SFLASH_READ_ID1                     = 0x90, /* data size varies       */
    SFLASH_READ_ID2                     = 0xAB, /* data size varies       */
    SFLASH_READ_JEDEC_ID                = 0x9F, /* RDID                   */
    SFLASH_CHIP_ERASE1                  = 0x60, /* CE                     */
    SFLASH_CHIP_ERASE2                  = 0xC7, /* CE                     */
    SFLASH_ENABLE_WRITE_STATUS_REGISTER = 0x50, /* EWSR   - SST only      */
    SFLASH_READ_SECURITY_REGISTER       = 0x2B, /* RDSCUR - Macronix only */
    SFLASH_WRITE_SECURITY_REGISTER      = 0x2F, /* WRSCUR - Macronix only */
    SFLASH_ENTER_SECURED_OTP            = 0xB1, /* ENSO   - Macronix only */
    SFLASH_EXIT_SECURED_OTP             = 0xC1, /* EXSO   - Macronix only */
    SFLASH_DEEP_POWER_DOWN              = 0xB9, /* DP     - Macronix only */
    SFLASH_RELEASE_DEEP_POWER_DOWN      = 0xAB, /* RDP    - Macronix only */
    SFLASH_QUAD_READ                    = 0x6B, /* QREAD  - Macronix only */
    SFLASH_QUAD_WRITE                   = 0x32, /* QWRITE - ISSI only     */
    SFLASH_X4IO_WRITE                   = 0x38, /* 4PP    - Macronix only */
    SFLASH_X4IO_READ                    = 0xEB, /* 4READ  - Macronix only */
    SFLASH_DIE_ERASE_MICRON             = 0xC4, /* Die Erase - Micron only */
    SFLASH_SECTOR_ERASE_MICRON          = 0xD8, /* Sector Erase (64KB) - Micron only */
    SFLASH_CLEAR_FLAG_STATUS_REGISTER   = 0x50,  /* Micro Only */
    SFLASH_READ_FLAG_STATUS_REGISTER    = 0x70,  /* Micron Only */
    SFLASH_WRITE_ENH_VOLATILE_REGISTER  = 0x61, /* WRITE Enhance status register, Micron only */
    SFLASH_READ_ENH_VOLATILE_REGISTER   = 0x65, /* READ Enhance status register, Micron only */
} sflash_command_t;


#define SFLASH_DUMMY_BYTE ( 0xA5 )



#define SFLASH_MANUFACTURER( id )      ( (uint8_t) ( ( (id) & 0x00ff0000 ) >> 16 ) )
#define SFLASH_SUPPORTED(NAME)         ( SFLASH_SUPPORT_##NAME##_PARTS )
#define SFLASH_MANUFACTURER_SUPPORTED( id, NAME )  ( SFLASH_SUPPORTED( NAME ) && (SFLASH_MANUFACTURER( id ) == SFLASH_MANUFACTURER_##NAME ) )

#define SFLASH_MANUFACTURER_SST        ( (uint8_t) 0xBF )
#define SFLASH_MANUFACTURER_MACRONIX   ( (uint8_t) 0xC2 )
#define SFLASH_MANUFACTURER_EON        ( (uint8_t) 0x1C )
#define SFLASH_MANUFACTURER_ISSI_CQ    ( (uint8_t) 0x7F )
#define SFLASH_MANUFACTURER_ISSI_LP    ( (uint8_t) 0x9D )
#define SFLASH_MANUFACTURER_MICRON     ( (uint8_t) 0x20 )
#define SFLASH_MANUFACTURER_WINBOND    ( (uint8_t) 0xEF )

#define SFLASH_ID_MX25L8006E           ( (uint32_t) 0xC22014 )
#define SFLASH_ID_MX25L1606E           ( (uint32_t) 0xC22015 )
#define SFLASH_ID_MX25L6433F           ( (uint32_t) 0xC22017 )
#define SFLASH_ID_MX25L12835F          ( (uint32_t) 0xC22018 )
#define SFLASH_ID_MX25L25635F          ( (uint32_t) 0xC22019 )
#define SFLASH_ID_MX25U1635F           ( (uint32_t) 0xC22535 )
#define SFLASH_ID_MX66U51235F          ( (uint32_t) 0xC2253A )
#define SFLASH_ID_SST25VF080B          ( (uint32_t) 0xBF258E )
#define SFLASH_ID_EN25QH16             ( (uint32_t) 0x1C3015 )
#define SFLASH_ID_ISSI25CQ032          ( (uint32_t) 0x7F9D46 )
#define SFLASH_ID_N25Q512A             ( (uint32_t) 0x20BB20 )
#define SFLASH_ID_ISSI25LP064          ( (uint32_t) 0x9D6017 )
#define SFLASH_ID_N25Q064A             ( (uint32_t) 0x20BA17 )
#define SFLASH_ID_W25Q64FV             ( (uint32_t) 0xEF4017 )
#define SFLASH_ID_CY15B104Q            ( (uint32_t) 0x7F7F7F )

#define SFLASH_ID_DEFAULT              ( (uint32_t) 0x000000 )
#define SFLASH_ID_MAX_PARTS  8


#define SFLASH_SIZE_1MByte                0x100000
#define SFLASH_SIZE_2MByte                0x200000
#define SLFASH_SIZE_32MByte               0x2000000
#define SFLASH_SIZE_64MByte               0x4000000
#define SFLASH_SIZE_512KByte              0x80000

typedef struct
{
        unsigned char id[3];
} device_id_t;

typedef struct
{
        uint32_t device_id;       /* device id of the flash */
        unsigned long size;       /* size of the flash      */
} device_id_to_flash_size_t;

int sflash_read_ID              ( const sflash_handle_t* handle, /*@out@*/ /*@dependent@*/ device_id_t* data_addr );
int sflash_write_enable         ( const sflash_handle_t* handle );
int sflash_read_status_register ( const sflash_handle_t* handle, /*@out@*/  /*@dependent@*/ unsigned char* dest_addr );
int sflash_read_status_register2 ( const sflash_handle_t* handle, /*@out@*/  /*@dependent@*/ unsigned char* dest_addr );
int sflash_write_status_register( const sflash_handle_t* handle, unsigned char value );




#ifdef __cplusplus
}
#endif

#endif /* INCLUDED_SPI_FLASH_INTERNAL_H */
