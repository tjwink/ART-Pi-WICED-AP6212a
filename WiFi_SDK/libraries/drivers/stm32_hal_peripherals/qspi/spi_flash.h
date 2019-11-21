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
#ifndef __STM32H7_QSPI_H
#define __STM32H7_QSPI_H

#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_assert.h"
#include "wwd_assert.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

 /*
  * secure sflash uses SECURE_SECTOR_METADATA_SIZE bytes out of each SFLASH_SECTOR to
  * store metadata (for example the SHA265 Hash of the sector data), so effectively
  * each sector can store only  ( SECTOR_SIZE - SECURE_SECTOR_METADATA_SIZE )
  * bytes of data.
  *
  */
 #define SWAP( T, x, y )                     do { T temp = x; x = y; y = temp; } while ( 0 )
 #define SFLASH_SHA256_HASH_SIZE             ( 32 )
 #define SECURE_SECTOR_METADATA_SIZE         ( SFLASH_SHA256_HASH_SIZE )
 #define SECURE_SECTOR_DATA_SIZE             ( SECURE_SECTOR_SIZE - SECURE_SECTOR_METADATA_SIZE )
 #define SECURE_SECTOR_SIZE                  ( 4096 ) /* SECURE_SECTOR_DATA_SIZE + SECURE_SECTOR_METADATA_SIZE */

  /* input : sflash physical address
   * Convert input physical address to nearest smaller sector aligned address
   */
 #define ALIGN_TO_SECTOR_ADDRESS( x )        ( ( ( x ) / SECURE_SECTOR_SIZE ) * SECURE_SECTOR_SIZE )

 /* input : size in bytes
  * For n input bytes, calculate the total secure sflash metadata overhead bytes
  * needed to store n input bytes in secure sflash
  */
 #define SECURE_SFLASH_METADATA_SIZE( x )    ( ( ( x ) / SECURE_SECTOR_SIZE ) * SECURE_SECTOR_METADATA_SIZE )

 /* input : sflash physical address
  * Convert sflash physical address to secure sflash physical address
  * The mapping is :
  * ( 0 )                           to  ( SECURE_SECTOR_DATA_SIZE - 1 )         ----->  0th Sector
  * ( SECURE_SECTOR_DATA_SIZE )     to  ( ( SECURE_SECTOR_DATA_SIZE * 2 ) -1 )  ----->  1st Sector
  * ( SECURE_SECTOR_DATA_SIZE * 2 ) to  ( ( SECURE_SECTOR_DATA_SIZE * 3 ) -1 )  ----->  2nd Sector
  * ...
  */
 #define SECURE_SECTOR_ADDRESS( x )           ( ( ( x ) / SECURE_SECTOR_DATA_SIZE ) * SECURE_SECTOR_SIZE)

 /* input : sflash physical address
  * Derive the offset of a given address within a secure sector
  */
 #define OFFSET_WITHIN_SECURE_SECTOR( x )    ( ( x ) % SECURE_SECTOR_DATA_SIZE )

struct sflash_capabilities;

typedef enum
{
    SFLASH_WRITE_NOT_ALLOWED = 0,
    SFLASH_WRITE_ALLOWED     = 1,
} sflash_write_allowed_t;

typedef struct securesflash_handle securesflash_handle_t;

typedef /*@abstract@*/ /*@immutable@*/ struct
{
    uint32_t device_id;
    void * platform_peripheral;
    const struct sflash_capabilities* capabilities;
    sflash_write_allowed_t write_allowed;
    securesflash_handle_t* securesflash_handle;
} sflash_handle_t;


typedef int ( *sflash_write_t ) ( const sflash_handle_t* const handle, unsigned long device_address,
        /*@observer@*/ const void* const data_addr, unsigned int size );
typedef int ( *sflash_read_t ) ( const sflash_handle_t* const handle, unsigned long device_address,
        /*@out@*/ /*@dependent@*/ void* data_addr, unsigned int size );

struct securesflash_handle
{
    /* Buffer to Read from Sflash and encrypt/decrypt data */
    uint8_t         hwcrypto_buffer[ SECURE_SECTOR_SIZE * 2 ];
#ifdef __IAR_SYSTEMS_ICC__
    #pragma pack(push, 32)
    uint8_t         scratch_pad [ 64 ];
    #pragma pack(pop)
#else
    uint8_t         scratch_pad [ 64 ] ALIGNED(32);
#endif
    uint8_t         hmac_key[ 32 ]; /* HMAC Key size */
    uint8_t         aes128_key[ 16 ]; /* AES CBC 128 Key size */
    sflash_read_t   sflash_secure_read_function;
    sflash_write_t  sflash_secure_write_function;
};

/**
 *  Initializes a SPI Flash chip
 *
 *  Internally this initializes the associated SPI port, then
 *  reads the chip ID from the SPI flash to determine what type it is.
 *
 * @param[in] handle            Handle structure that will be used for this sflash instance - allocated by caller.
 * @param[in] peripheral_id     An ID value which is passed to the underlying sflash_platform_init function
 * @param[in] write_allowed_in  Determines whether writing will be allowed to this sflash handle
 *
 * @return @ref wiced_result_t
 */
int init_sflash         ( /*@out@*/ sflash_handle_t* const handle, /*@shared@*/ void* peripheral_id, sflash_write_allowed_t write_allowed_in );

/**
 *  De-initializes a SPI Flash chip
 *
 * @param[in] handle            Handle structure that will be used for this sflash instance - allocated by caller.
 *
 * @return @ref wiced_result_t
 */
int deinit_sflash       ( /*@out@*/ sflash_handle_t* const handle);

/**
 *  Reads data from a SPI Flash chip
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where reading will start
 * @param[out] data_addr         Destination buffer in memory that will receive the data
 * @param[in]  size              Number of bytes to read from the chip
 *
 * @return @ref wiced_result_t
 */
int sflash_read         ( const sflash_handle_t* const handle, unsigned long device_address, /*@out@*/  /*@dependent@*/ void* data_addr, unsigned int size );

/**
 *  Write data to a SPI Flash chip
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where writing will start
 * @param[in]  data_addr         Pointer to the buffer in memory that contains the data being written
 * @param[in]  size              Number of bytes to write to the chip
 *
 * @return @ref wiced_result_t
 */
int sflash_write        ( const sflash_handle_t* const handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size );

/**
 *  Secure-Write data to a SPI Flash chip (Encrypt and Authenticate data sector by sector before being written)
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where writing will start
 * @param[in]  data_addr         Pointer to the buffer in memory that contains the data being written
 * @param[in]  size              Number of bytes to write to the chip
 *
 * @return @ref wiced_result_t
 */

int sflash_write_secure ( const sflash_handle_t* const handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size );

/**
 *  Reads data from DCT, Decrypts/Authenticates the data read, returns error if
 *  Authentication fails
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where reading will start
 * @param[out] data_addr         Destination buffer in memory that will receive the data
 * @param[in]  size              Number of bytes to read from the chip
 *
 * @return @ref wiced_result_t
 */
int sflash_dct_read_secure( const sflash_handle_t* const sflash_handle, unsigned long device_address, /*@out@*/ /*@dependent@*/ void* data_addr, unsigned int size );

/**
 *  Writes data to DCT, Encrypts and Sign the data
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where reading will start
 * @param[out] data_addr         Destination buffer in memory that will receive the data
 * @param[in]  size              Number of bytes to read from the chip
 *
 * @return @ref wiced_result_t
 */
int sflash_dct_write_secure ( const sflash_handle_t* const sflash_handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size );

/**
 *  Erase the contents of a SPI Flash chip
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 *
 * @return @ref wiced_result_t
 */
int sflash_chip_erase   ( const sflash_handle_t* const handle );


/**
 *  Erase one sector of a SPI Flash chip
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the sflash chip of the first byte of the sector to erase
 *
 * @return @ref wiced_result_t
 */
int sflash_sector_erase ( const sflash_handle_t* const handle, unsigned long device_address );


int sflash_block_erase  ( const sflash_handle_t* const handle, unsigned long device_address );

/**
 *  Erase one sector of a SPI Flash chip
 *
 * @param[in]  handle    Handle structure that was initialized with @ref init_sflash
 * @param[out] size      Variable which will receive the capacity size in bytes of the sflash chip
 *
 * @return @ref wiced_result_t
 */
int sflash_get_size     ( const sflash_handle_t* const handle, /*@out@*/ unsigned long* size );

/**
 *  Reads data from a SPI Flash chip, Decrypts/Authenticates the data read, returns error if
 *  Authentication fails
 *
 * @param[in]  handle            Handle structure that was initialized with @ref init_sflash
 * @param[in]  device_address    The location on the SPI flash where reading will start
 * @param[out] data_addr         Destination buffer in memory that will receive the data
 * @param[in]  size              Number of bytes to read from the chip
 *
 * @return @ref wiced_result_t
 */

int sflash_read_secure( const sflash_handle_t* const handle, unsigned long device_address, /*@out@*/ /*@dependent@*/ void* data_addr, unsigned int size );


/* Definition for QSPI clock resources */
#define QSPI_CLK_ENABLE()              __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()             __HAL_RCC_QSPI_CLK_DISABLE()
#define QSPI_CS_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_BK1_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define QSPI_BK1_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()

#define QSPI_MDMA_CLK_ENABLE()         __HAL_RCC_MDMA_CLK_ENABLE()

#define QSPI_FORCE_RESET()             __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()           __HAL_RCC_QSPI_RELEASE_RESET()

#define PACKET_SIZE                    ( 256u )
#define WICED_SECTOR_SIZE              ( 4096 )


#endif /* __STM32H7_QSPI_H */
