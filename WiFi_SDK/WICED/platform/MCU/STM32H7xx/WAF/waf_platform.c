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
 * Defines STM32M7xx WICED application framework functions
 */
#include "waf_platform.h"

#define PLATFORM_FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define PLATFORM_FLASH_END_ADDR       (uint32_t)(0x081FFFFF)
#define PLATFORM_FLASH_WORD_SIZE      (uint32_t)(32)
#define PLATFORM_FLASH_WORD_MASK      (PLATFORM_FLASH_WORD_SIZE - 1)
#define PLATFORM_FLASH_SECTOR_SIZE    (uint32_t)(128 * 1024)
#define PLATFORM_FLASH_SECTOR_MASK    (PLATFORM_FLASH_SECTOR_SIZE - 1)
#define PLATFORM_FLASH_SECTOR_COUNT   (uint16_t)(16)

extern void* spare_start_addr_loc;
extern void* spare_size_loc;
#define SPARE_FLASH_START_ADDR ((uint32_t)&spare_start_addr_loc)
#define SPARE_FLASH_SIZE       ((uint32_t)&spare_size_loc)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define PLATFORM_APP_START_SECTOR      ( PLATFORM_FLASH_SECTOR_4  )
#define PLATFORM_APP_END_SECTOR        ( PLATFORM_FLASH_SECTOR_15 )

#define APP_CODE_START_ADDR   ((uint32_t)&app_code_start_addr_loc)
#define SRAM_START_ADDR       ((uint32_t)&sram_start_addr_loc)
extern void* app_code_start_addr_loc;
extern void* sram_start_addr_loc;

typedef struct
{
    uint32_t bank;
    uint32_t sector;
    uint32_t address;
} flash_bank_sector_t;

static flash_bank_sector_t flash_bank_sector_mapping[] =
{
    [PLATFORM_FLASH_SECTOR_0]  = {FLASH_BANK_1, FLASH_SECTOR_0, ADDR_FLASH_SECTOR_0_BANK1},
    [PLATFORM_FLASH_SECTOR_1]  = {FLASH_BANK_1, FLASH_SECTOR_1, ADDR_FLASH_SECTOR_1_BANK1},
    [PLATFORM_FLASH_SECTOR_2]  = {FLASH_BANK_1, FLASH_SECTOR_2, ADDR_FLASH_SECTOR_2_BANK1},
    [PLATFORM_FLASH_SECTOR_3]  = {FLASH_BANK_1, FLASH_SECTOR_3, ADDR_FLASH_SECTOR_3_BANK1},
    [PLATFORM_FLASH_SECTOR_4]  = {FLASH_BANK_1, FLASH_SECTOR_4, ADDR_FLASH_SECTOR_4_BANK1},
    [PLATFORM_FLASH_SECTOR_5]  = {FLASH_BANK_1, FLASH_SECTOR_5, ADDR_FLASH_SECTOR_5_BANK1},
    [PLATFORM_FLASH_SECTOR_6]  = {FLASH_BANK_1, FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6_BANK1},
    [PLATFORM_FLASH_SECTOR_7]  = {FLASH_BANK_1, FLASH_SECTOR_7, ADDR_FLASH_SECTOR_7_BANK1},
    [PLATFORM_FLASH_SECTOR_8]  = {FLASH_BANK_2, FLASH_SECTOR_0, ADDR_FLASH_SECTOR_0_BANK2},
    [PLATFORM_FLASH_SECTOR_9]  = {FLASH_BANK_2, FLASH_SECTOR_1, ADDR_FLASH_SECTOR_1_BANK2},
    [PLATFORM_FLASH_SECTOR_10] = {FLASH_BANK_2, FLASH_SECTOR_2, ADDR_FLASH_SECTOR_2_BANK2},
    [PLATFORM_FLASH_SECTOR_11] = {FLASH_BANK_2, FLASH_SECTOR_3, ADDR_FLASH_SECTOR_3_BANK2},
    [PLATFORM_FLASH_SECTOR_12] = {FLASH_BANK_2, FLASH_SECTOR_4, ADDR_FLASH_SECTOR_4_BANK2},
    [PLATFORM_FLASH_SECTOR_13] = {FLASH_BANK_2, FLASH_SECTOR_5, ADDR_FLASH_SECTOR_5_BANK2},
    [PLATFORM_FLASH_SECTOR_14] = {FLASH_BANK_2, FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6_BANK2},
    [PLATFORM_FLASH_SECTOR_15] = {FLASH_BANK_2, FLASH_SECTOR_7, ADDR_FLASH_SECTOR_7_BANK2},
};

#if defined ( __ICCARM__ )

static inline void __jump_to( uint32_t addr )
{
    __asm( "ORR R0, R0, #1" );  /* Last bit of jump address indicates whether destination is Thumb or ARM code */
    __asm( "BX R0" );
}

#elif defined ( __GNUC__ )

__attribute__( ( always_inline ) ) static __INLINE void __jump_to( uint32_t addr )
{
    addr |= 0x00000001;  /* Last bit of jump address indicates whether destination is Thumb or ARM code */
  __ASM volatile ("BX %0" : : "r" (addr) );
}

#endif

void platform_start_app( uint32_t entry_point )
{

    /* Simulate a reset for the app: */
    /*   Switch to Thread Mode, and the Main Stack Pointer */
    /*   Change the vector table offset address to point to the app vector table */
    /*   Set other registers to reset values (esp LR) */
    /*   Jump to the reset vector */


    if ( entry_point == 0 )
    {
        uint32_t* vector_table =  (uint32_t*) APP_CODE_START_ADDR;
        entry_point = vector_table[1];
    }


    __asm( "MOV LR,        #0xFFFFFFFF" );
    __asm( "MOV R1,        #0x01000000" );
    __asm( "MSR APSR_nzcvq,     R1" );
    __asm( "MOV R1,        #0x00000000" );
    __asm( "MSR PRIMASK,   R1" );
    __asm( "MSR FAULTMASK, R1" );
    __asm( "MSR BASEPRI,   R1" );
    __asm( "MSR CONTROL,   R1" );

/*  Now rely on the app crt0 to load VTOR / Stack pointer

    SCB->VTOR = vector_table_address; - Change the vector table to point to app vector table
    __set_MSP( *stack_ptr ); */

    __jump_to( entry_point );

}

__attribute__ ((section (".fast")))
static platform_result_t platform_get_flash_sector( uint32_t address, uint16_t* flash_sector )
{
    uint16_t sector;

    if ( flash_sector == NULL )
    {
        return PLATFORM_BADARG;
    }

    if ( (address < ADDR_FLASH_SECTOR_1_BANK1) && (address >= ADDR_FLASH_SECTOR_0_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_0;
    }
    else if ( (address < ADDR_FLASH_SECTOR_2_BANK1) && (address >= ADDR_FLASH_SECTOR_1_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_1;
    }
    else if ( (address < ADDR_FLASH_SECTOR_3_BANK1) && (address >= ADDR_FLASH_SECTOR_2_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_2;
    }
    else if ( (address < ADDR_FLASH_SECTOR_4_BANK1) && (address >= ADDR_FLASH_SECTOR_3_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_3;
    }
    else if ( (address < ADDR_FLASH_SECTOR_5_BANK1) && (address >= ADDR_FLASH_SECTOR_4_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_4;
    }
    else if ( (address < ADDR_FLASH_SECTOR_6_BANK1) && (address >= ADDR_FLASH_SECTOR_5_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_5;
    }
    else if ( (address < ADDR_FLASH_SECTOR_7_BANK1) && (address >= ADDR_FLASH_SECTOR_6_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_6;
    }
    else if ( (address < ADDR_FLASH_SECTOR_0_BANK2) && (address >= ADDR_FLASH_SECTOR_7_BANK1) )
    {
        sector = PLATFORM_FLASH_SECTOR_7;
    }
    else if ( (address < ADDR_FLASH_SECTOR_1_BANK2) && (address >= ADDR_FLASH_SECTOR_0_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_8;
    }
    else if ( (address < ADDR_FLASH_SECTOR_2_BANK2) && (address >= ADDR_FLASH_SECTOR_1_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_9;
    }
    else if ( (address < ADDR_FLASH_SECTOR_3_BANK2) && (address >= ADDR_FLASH_SECTOR_2_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_10;
    }
    else if ( (address < ADDR_FLASH_SECTOR_4_BANK2) && (address >= ADDR_FLASH_SECTOR_3_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_11;
    }
    else if ( (address < ADDR_FLASH_SECTOR_5_BANK2) && (address >= ADDR_FLASH_SECTOR_4_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_12;
    }
    else if ( (address < ADDR_FLASH_SECTOR_6_BANK2) && (address >= ADDR_FLASH_SECTOR_5_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_13;
    }
    else if ( (address < ADDR_FLASH_SECTOR_7_BANK2) && (address >= ADDR_FLASH_SECTOR_6_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_14;
    }
    else if ( (address <= PLATFORM_FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_7_BANK2) )
    {
        sector = PLATFORM_FLASH_SECTOR_15;
    }
    else
    {
        return PLATFORM_ERROR;
    }

    *flash_sector = sector;

    return PLATFORM_SUCCESS;
}

__attribute__ ((section (".fast")))
static platform_result_t platform_erase_flash_bank_sectors( uint32_t flash_bank, uint32_t start_sector, uint32_t end_sector )
{
    uint32_t erase_error;
    uint32_t erase_sector;
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t erase_address;
    uint32_t erase_index;
    uint32_t erase_required;

    /* Unlock the Flash */
    HAL_FLASH_Unlock( );

    /* Clear any error flags */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGSERR );

    for ( erase_sector = start_sector ; erase_sector <= end_sector ; erase_sector++ )
    {
        erase_address  = flash_bank_sector_mapping[erase_sector].address;
        erase_required = 0;

        for ( erase_index = 0 ; erase_index < PLATFORM_FLASH_SECTOR_SIZE ; erase_index++ )
        {
            if ( ((uint8_t*)erase_address)[erase_index] != 0xFF )
            {
                erase_required = 1;
                break;
            }
        }

        if ( erase_required == 1 )
        {
            platform_watchdog_kick( );

            flash_erase.Banks        = flash_bank;
            flash_erase.Sector       = erase_sector;
            flash_erase.NbSectors    = 1;
            flash_erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
            flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

            if ( HAL_FLASHEx_Erase( &flash_erase, &erase_error ) != HAL_OK )
            {
                /* Lock the Flash */
                HAL_FLASH_Lock( );

                /* Error occurred during erase */
                return PLATFORM_ERROR;
            }

            platform_watchdog_kick( );
        }
    }

    /* Lock the Flash */
    HAL_FLASH_Lock( );

    return PLATFORM_SUCCESS;
}

/*
 * Programs a flash word at a (previously erased) flash address.
 * The flash should be unlocked before this function is called.
 */
__attribute__ ((section (".fast")))
static platform_result_t platform_program_flash_word( uint32_t word_address, uint64_t* word_data )
{
    uint32_t word_idx;
    uint32_t flash_pgrm;

    flash_pgrm = 0;

    for ( word_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx++ )
    {
        if ( ((uint8_t*)word_data)[word_idx] != 0xFF )
        {
            flash_pgrm = 1;
            break;
        }
    }

    if ( flash_pgrm == 1 )
    {
        if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, word_address, (uint64_t)((uint32_t)word_data)) != HAL_OK )
        {
            /* Error occurred during program */
            return PLATFORM_ERROR;
        }
    }

    return PLATFORM_SUCCESS;
}

__attribute__ ((section (".fast")))
static platform_result_t platform_copy_flash_sector( uint16_t src_sector, uint16_t dst_sector )
{
    uint32_t src_address;
    uint32_t dst_address;
    uint32_t data_idx;
    uint32_t word_idx;
    uint32_t sector_idx;
    uint64_t flash_word[PLATFORM_FLASH_WORD_SIZE / 8];

    if ( (src_sector >= PLATFORM_FLASH_SECTOR_COUNT) || (dst_sector >= PLATFORM_FLASH_SECTOR_COUNT) )
    {
        return PLATFORM_BADARG;
    }

    if ( src_sector == dst_sector )
    {
        return PLATFORM_BADARG;
    }

    if ( platform_erase_flash(dst_sector, dst_sector) != PLATFORM_SUCCESS )
    {
        return PLATFORM_ERROR;
    }

    src_address = flash_bank_sector_mapping[src_sector].address;
    dst_address = flash_bank_sector_mapping[dst_sector].address;

    /* Unlock the Flash */
    HAL_FLASH_Unlock();

    /* Clear any error flags */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGSERR );

    for ( sector_idx = 0 ; sector_idx < PLATFORM_FLASH_SECTOR_SIZE ; sector_idx = sector_idx + PLATFORM_FLASH_WORD_SIZE )
    {
        for ( word_idx = 0, data_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE; word_idx = word_idx + 8, data_idx++ )
        {
            flash_word[data_idx] = *((uint64_t*)(src_address + sector_idx + word_idx));
        }

        if ( platform_program_flash_word( (dst_address + sector_idx), flash_word ) != PLATFORM_SUCCESS )
        {
            /* Lock the Flash */
            HAL_FLASH_Lock( );

            /* Error occurred during program */
            return PLATFORM_ERROR;
        }
    }

    /* Lock the Flash */
    HAL_FLASH_Lock();

    return PLATFORM_SUCCESS;
}

__attribute__ ((section (".fast")))
platform_result_t platform_erase_flash( uint16_t start_sector, uint16_t end_sector )
{
    platform_result_t result;

    if ( (start_sector >= PLATFORM_FLASH_SECTOR_COUNT) || (end_sector >= PLATFORM_FLASH_SECTOR_COUNT) )
    {
        return PLATFORM_BADARG;
    }

    if ( end_sector < start_sector )
    {
        /*
         * On this platform, DCT_SIZE (16K) is smaller than SECTOR_SIZE (128K),
         * which results in end_sector to be computed as smaller than start_sector
         * when called from wiced_dct_internal_common.c, hence this workaround.
         * TODO:
         * Fix wiced_dct_internal_common.c to correctly compute start_sector
         * and end_sector when the DCT_SIZE is smaller than the SECTOR_SIZE.
         */
        end_sector = start_sector;
    }

    if ( flash_bank_sector_mapping[start_sector].bank == flash_bank_sector_mapping[end_sector].bank )
    {
        result = platform_erase_flash_bank_sectors(flash_bank_sector_mapping[start_sector].bank,
                                                   flash_bank_sector_mapping[start_sector].sector,
                                                   flash_bank_sector_mapping[end_sector].sector);
    }
    else
    {
        result = platform_erase_flash_bank_sectors(flash_bank_sector_mapping[start_sector].bank,
                                                   flash_bank_sector_mapping[start_sector].sector,
                                                   flash_bank_sector_mapping[PLATFORM_FLASH_SECTOR_7].sector);

        if ( result == PLATFORM_SUCCESS )
        {
            result = platform_erase_flash_bank_sectors(flash_bank_sector_mapping[end_sector].bank,
                                                       flash_bank_sector_mapping[PLATFORM_FLASH_SECTOR_8].sector,
                                                       flash_bank_sector_mapping[end_sector].sector);
        }
    }

    return result;
}

__attribute__ ((section (".fast")))
platform_result_t platform_write_flash_chunk( uint32_t address, const void* data, uint32_t size )
{
    uint16_t start_sector;
    uint16_t end_sector;
    uint16_t spare_sector;
    uint32_t start_byte_addr;
    uint32_t end_byte_addr;

    uint32_t src_address;
    uint32_t dst_address;
    uint32_t src_word_addr;
    uint32_t dst_word_addr;
    uint32_t data_idx;
    uint32_t word_idx;
    uint32_t sector_idx;

    uint8_t* data_buf;
    uint8_t* data_ptr  = (uint8_t*) data;
    uint64_t flash_word[PLATFORM_FLASH_WORD_SIZE / 8];

    start_byte_addr = address;
    end_byte_addr   = address + size - 1;

    if ( memcmp((void*)address, (void*)data, size) == 0 )
    {
        return PLATFORM_SUCCESS;
    }

    if ( platform_get_flash_sector(start_byte_addr, &start_sector) != PLATFORM_SUCCESS )
    {
        return PLATFORM_BADARG;
    }

    if ( platform_get_flash_sector(end_byte_addr, &end_sector) != PLATFORM_SUCCESS )
    {
        return PLATFORM_BADARG;
    }

    if ( platform_get_flash_sector(SPARE_FLASH_START_ADDR, &spare_sector) != PLATFORM_SUCCESS )
    {
        return PLATFORM_BADARG;
    }

    /* Currently only supports writes within a single sector */
    if ( start_sector != end_sector )
    {
        /* TODO: Implement writes crossing a sector boundary */
        return PLATFORM_UNSUPPORTED;
    }

    /* Try to allocate SECTOR_SIZE (128K) bytes on the heap */
    data_buf = (uint8_t*)calloc( PLATFORM_FLASH_SECTOR_SIZE, 1 );

    if ( data_buf != NULL )
    {
        /* Use data buffer on the heap */
        src_address = flash_bank_sector_mapping[start_sector].address;
        dst_address = (uint32_t)data_buf;

        for ( sector_idx = 0 ; sector_idx < PLATFORM_FLASH_SECTOR_SIZE ; sector_idx = sector_idx + PLATFORM_FLASH_WORD_SIZE )
        {
            for ( word_idx = 0, data_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx = word_idx + 8, data_idx++ )
            {
                flash_word[data_idx] = *((uint64_t*)(src_address + sector_idx + word_idx));
            }

            for ( word_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx++ )
            {
                data_buf[sector_idx + word_idx] = ((uint8_t*)flash_word)[word_idx];
            }
        }

        src_address = (uint32_t)data_buf;
        dst_address = flash_bank_sector_mapping[start_sector].address;
    }
    else
    {
        /* Use spare sector on the flash */
        src_address = flash_bank_sector_mapping[start_sector].address;
        dst_address = flash_bank_sector_mapping[spare_sector].address;

        if ( platform_copy_flash_sector(start_sector, spare_sector) != PLATFORM_SUCCESS )
        {
            return PLATFORM_ERROR;
        }

        src_address = flash_bank_sector_mapping[spare_sector].address;
        dst_address = flash_bank_sector_mapping[start_sector].address;
    }

    if ( platform_erase_flash(start_sector, start_sector) != PLATFORM_SUCCESS )
    {
        if ( data_buf != NULL )
        {
            free( data_buf );
        }

        return PLATFORM_ERROR;
    }

    /* Unlock the Flash */
    HAL_FLASH_Unlock();

    /* Clear any error flags */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGSERR );

    for ( sector_idx = 0 ; sector_idx < PLATFORM_FLASH_SECTOR_SIZE ; sector_idx = sector_idx + PLATFORM_FLASH_WORD_SIZE )
    {
        src_word_addr = src_address + sector_idx;
        dst_word_addr = dst_address + sector_idx;

        if ( data_buf != NULL )
        {
            /* Read data from heap buffer */
            for ( word_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx++ )
            {
                ((uint8_t*)flash_word)[word_idx] = data_buf[sector_idx + word_idx];
            }
        }
        else
        {
            /* Read data from spare sector */
            for ( word_idx = 0, data_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx = word_idx + 8, data_idx++ )
            {
                flash_word[data_idx] = *((uint64_t*)(src_word_addr + word_idx));
            }
        }

        /* Prepare the flash word */
        for ( word_idx = 0 ; word_idx < PLATFORM_FLASH_WORD_SIZE ; word_idx++ )
        {
            data_idx = dst_word_addr + word_idx;

            if ( (data_idx >= start_byte_addr) && (data_idx <= end_byte_addr) )
            {
                ((uint8_t*)flash_word)[word_idx] = *data_ptr;
                data_ptr++;
            }
        }

        /* Program the flash word */
        if ( platform_program_flash_word( dst_word_addr, flash_word ) != PLATFORM_SUCCESS )
        {
            /* Lock the Flash */
            HAL_FLASH_Lock( );

            if ( data_buf != NULL )
            {
                free( data_buf );
            }

            /* Error occurred during program */
            return PLATFORM_ERROR;
        }
    }

    /* Lock the Flash */
    HAL_FLASH_Lock();

    if ( data_buf != NULL )
    {
        free( data_buf );
    }

    return PLATFORM_SUCCESS;
}

__attribute__ ((section (".fast")))
void platform_erase_app_area( uint32_t physical_address, uint32_t size )
{
    /* if app in RAM, no need for erase */
    if ( physical_address < SRAM_START_ADDR )
    {
        if (physical_address == (uint32_t)DCT1_START_ADDR)
        {
            platform_erase_flash( PLATFORM_DCT_COPY1_START_SECTOR, PLATFORM_DCT_COPY1_END_SECTOR );
        }
        else
        {
            platform_erase_flash( PLATFORM_APP_START_SECTOR, PLATFORM_APP_END_SECTOR );
        }
    }
    (void) size;
}

/* The function would copy data from serial flash to internal flash.
 * The function assumes that the program area is already erased (for now).
 * TODO: Adding erasing the required area
 */
__attribute__ ((section (".fast")))
static wiced_result_t platform_copy_app_to_iflash( const image_location_t* app_header_location, uint32_t offset, uint32_t physical_address, uint32_t size )
{
    /* Bootloader doesn't support BSS sections. */
    uint8_t buff[ 64 ];

    while ( size > 0 )
    {
        uint32_t write_size = MIN( sizeof(buff), size);
        wiced_apps_read( app_header_location, buff, offset, write_size );
        platform_write_flash_chunk( (uint32_t) physical_address, buff, write_size );
        if (memcmp((char *)physical_address, buff, write_size))
        {
            offset = 0;
            return WICED_ERROR;
        }
        offset           += write_size;
        physical_address += write_size;
        size             -= write_size;
    }
    return WICED_SUCCESS;
}

__attribute__ ((section (".fast")))
void platform_load_app_chunk( const image_location_t* app_header_location, uint32_t offset, void* physical_address, uint32_t size )
{
    if ( (uint32_t) physical_address < SRAM_START_ADDR )
    {
        platform_copy_app_to_iflash( app_header_location, offset, (uint32_t) physical_address, size );
    }
    else
    {
        wiced_apps_read( app_header_location, physical_address, offset, size );
    }
}
