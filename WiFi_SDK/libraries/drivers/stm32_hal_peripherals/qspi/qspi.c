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
 * STM32 HAL based QSPI implementation
 */

#include "spi_flash.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef PRINT_DEBUG
#if !defined ( BOOTLOADER )
#define DEBUG_PRINTS(a) printf a
#else /* !defined ( BOOTLOADER ) */
#define DEBUG_PRINTS(a)
#endif /* !defined ( BOOTLOADER ) */
#else /* PRINT_DEBUG */
#define DEBUG_PRINTS(a)
#endif /* PRINT_DEBUG */

#define PRINT_ERROR
#ifdef PRINT_ERROR
#if !defined ( BOOTLOADER )
#define ERROR_PRINTS(a) printf a
#else /* !defined ( BOOTLOADER ) */
#define ERROR_PRINTS(a)
#endif /* !defined ( BOOTLOADER ) */
#else /* PRINT_DEBUG */
#define ERROR_PRINTS(a)
#endif /* PRINT_DEBUG */

#define VERIFY_RESULT( x )     { HAL_StatusTypeDef verify_result; verify_result = (x); if ( verify_result != HAL_OK ) {ERROR_PRINTS(("Returning error from %s %d\n", __func__, __LINE__)); return verify_result; }}

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
 *               Static Function Declarations
 ******************************************************/

static int sflash_write_enable(QSPI_HandleTypeDef *hqspi);
static int sflash_wait_busy_done(QSPI_HandleTypeDef *hqspi, uint32_t timeout_ms);
static int sflash_page_write(const sflash_handle_t* const handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size);
static int sflash_erase(const sflash_handle_t* handle, unsigned long device_address, unsigned int size);

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern const platform_gpio_t wiced_qspi_flash[];
QSPI_HandleTypeDef qspihandle = {0};
uint32_t backup_sector_address = ( 60 * 256 * 1024 ); /* 6th sector , each sector 256KB */

/******************************************************
 *               Function Definitions
 ******************************************************/

/* Called by STM32 HAL HAL_QSPI_Init() */
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    static MDMA_HandleTypeDef hmdma = {0};

    /* Enable the QuadSPI memory interface clock */
    QSPI_CLK_ENABLE();

    /* Reset the QuadSPI memory interface */
    QSPI_FORCE_RESET();
    QSPI_RELEASE_RESET();

    /* Enable DMA clock */
    QSPI_MDMA_CLK_ENABLE();

    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_CS ].port, wiced_qspi_flash[ WICED_QSPI_PIN_CS ].pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF9_QUADSPI);
    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_CLK].port, wiced_qspi_flash[ WICED_QSPI_PIN_CLK].pin_number, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_QUADSPI);
    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_D0 ].port, wiced_qspi_flash[ WICED_QSPI_PIN_D0 ].pin_number, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_QUADSPI);
    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_D1 ].port, wiced_qspi_flash[ WICED_QSPI_PIN_D1 ].pin_number, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_QUADSPI);
    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_D2 ].port, wiced_qspi_flash[ WICED_QSPI_PIN_D2 ].pin_number, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_QUADSPI);
    platform_gpio_set_alternate_function(wiced_qspi_flash[ WICED_QSPI_PIN_D3 ].port, wiced_qspi_flash[ WICED_QSPI_PIN_D3 ].pin_number, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_QUADSPI);

    /* NVIC configuration for QSPI interrupt */
    HAL_NVIC_SetPriority(QUADSPI_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);

    /* Enable MDMA clock */
    /* Input MDMA */
    /* Set the parameters to be configured */
    hmdma.Init.Request = MDMA_REQUEST_QUADSPI_FIFO_TH;
    hmdma.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    hmdma.Init.Priority = MDMA_PRIORITY_HIGH;
    hmdma.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;

    hmdma.Init.SourceInc = MDMA_SRC_INC_BYTE;
    hmdma.Init.DestinationInc = MDMA_DEST_INC_DISABLE;
    hmdma.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    hmdma.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma.Init.BufferTransferLength = 4;
    hmdma.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma.Init.DestBurst = MDMA_DEST_BURST_SINGLE;

    hmdma.Init.SourceBlockAddressOffset = 0;
    hmdma.Init.DestBlockAddressOffset = 0;

    hmdma.Instance = MDMA_Channel1;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hqspi, hmdma, hmdma);

    /* DeInitialize the MDMA Stream */
    HAL_MDMA_DeInit(&hmdma);
    /* Initialize the MDMA stream */
    HAL_MDMA_Init(&hmdma);

    /* Enable and set QuadSPI interrupt to the lowest priority */
    HAL_NVIC_SetPriority(MDMA_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(MDMA_IRQn);
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
    static MDMA_HandleTypeDef hmdma;

    HAL_NVIC_DisableIRQ(QUADSPI_IRQn);

    /* De-configure DMA channel */
    HAL_MDMA_DeInit(&hmdma);

    /* De-Configure QSPI pins */
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_CS ]);
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_CLK]);
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_D0 ]);
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_D1 ]);
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_D2 ]);
    platform_gpio_deinit(&wiced_qspi_flash[ WICED_QSPI_PIN_D3 ]);

    /* Reset the QuadSPI memory interface */
    QSPI_FORCE_RESET();
    QSPI_RELEASE_RESET();

    /* Disable the QuadSPI memory interface clock */
    QSPI_CLK_DISABLE();
}

int init_sflash( /*@out@*/ sflash_handle_t* const handle, /*@shared@*/ void* peripheral_id, sflash_write_allowed_t write_allowed_in )
{

    QSPI_CommandTypeDef scommand = {0};
    uint8_t sflash_id[16] = {0};

    UNUSED_PARAMETER(peripheral_id);
    UNUSED_PARAMETER(write_allowed_in);

    qspihandle.Instance = QUADSPI;
    qspihandle.Init.ClockPrescaler     = 4;
    qspihandle.Init.FifoThreshold      = 4;
    qspihandle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
    qspihandle.Init.FlashSize          = QSPI_FLASH_SIZE;
    qspihandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    qspihandle.Init.ClockMode          = QSPI_CLOCK_MODE_0;
    qspihandle.Init.FlashID            = QSPI_FLASH_ID_1;
    qspihandle.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

    VERIFY_RESULT(HAL_QSPI_Init(&qspihandle));
    handle->platform_peripheral = (void*)&qspihandle;

    scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    scommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Read Sflash ID */
    scommand.Instruction = READ_ID_CMD;
    scommand.AddressMode = QSPI_ADDRESS_NONE;
    scommand.DataMode    = QSPI_DATA_1_LINE;
    scommand.NbData      = sizeof(sflash_id);
    scommand.DummyCycles = 0;

    VERIFY_RESULT(HAL_QSPI_Command(&qspihandle, &scommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));
    VERIFY_RESULT(HAL_QSPI_Receive(&qspihandle, &sflash_id[0], HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

    if ( (SFLASH_MANUFACTURER_ID != sflash_id[0]) ||
         (SFLASH_DEVICE_ID_MSB   != sflash_id[1]) ||
         (SFLASH_DEVICE_ID_LSB   != sflash_id[2]) )
    {
        ERROR_PRINTS(("%s(): Incorrect SFLASH ID read: [%x] [%x] [%x] expected: [%x] [%x] [%x]\n", __func__, sflash_id[0], sflash_id[1], sflash_id[2], SFLASH_MANUFACTURER_ID, SFLASH_DEVICE_ID_MSB, SFLASH_DEVICE_ID_LSB));
        return 1;
    }

    return 0;
}

static int sflash_write_enable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     scommand = {0};
  QSPI_AutoPollingTypeDef sconfig  = {0};

  /* Enable write operations */
  scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  scommand.Instruction       = WRITE_ENABLE_CMD;
  scommand.AddressMode       = QSPI_ADDRESS_NONE;
  scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  scommand.DataMode          = QSPI_DATA_NONE;
  scommand.DummyCycles       = 0;
  scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  VERIFY_RESULT(HAL_QSPI_Command(hqspi, &scommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

  /* Configure automatic polling mode to wait for write enabling */
  sconfig.Match           = 0x02;
  sconfig.Mask            = 0x02;
  sconfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sconfig.StatusBytesSize = 1;
  sconfig.Interval        = 0x10;
  sconfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  scommand.Instruction    = READ_STATUS_REG_CMD;
  scommand.DataMode       = QSPI_DATA_1_LINE;

  VERIFY_RESULT(HAL_QSPI_AutoPolling(hqspi, &scommand, &sconfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

  return HAL_OK;
}

static int sflash_wait_busy_done(QSPI_HandleTypeDef *hqspi, uint32_t timeout_ms)
{
  QSPI_CommandTypeDef     sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  platform_watchdog_kick();

  VERIFY_RESULT(HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, timeout_ms));

  return HAL_OK;
}

int deinit_sflash       ( /*@out@*/ sflash_handle_t* const handle)
{
    HAL_QSPI_DeInit((QSPI_HandleTypeDef*)handle->platform_peripheral);
    return 0;
}

int sflash_read( const sflash_handle_t* const handle, unsigned long device_address, /*@out@*/  /*@dependent@*/ void* data_addr, unsigned int size )
{

    QSPI_CommandTypeDef scommand = {0};

    scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    scommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    scommand.Instruction       = READ_CMD;
    scommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    scommand.Address           = device_address;
    scommand.DataMode          = QSPI_DATA_1_LINE;
    scommand.NbData            = size;
    scommand.DummyCycles       = 0;

    platform_watchdog_kick();

    VERIFY_RESULT(HAL_QSPI_Command((QSPI_HandleTypeDef*)handle->platform_peripheral, &scommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));
    VERIFY_RESULT(HAL_QSPI_Receive((QSPI_HandleTypeDef*)handle->platform_peripheral, (uint8_t *)data_addr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

    /* Wait till busy flag is set */
    VERIFY_RESULT(sflash_wait_busy_done((QSPI_HandleTypeDef*)handle->platform_peripheral, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));
    return 0;
}

static int sflash_page_write( const sflash_handle_t* const handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size )
{
    QSPI_CommandTypeDef scommand = {0};

    if ( size == 0)
    {
        return 0;
    }

    /* Write enable */
    VERIFY_RESULT(sflash_write_enable(((QSPI_HandleTypeDef*)handle->platform_peripheral)));

    scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    scommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    scommand.Instruction       = PAGE_PROG_CMD;
    scommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    scommand.Address           = device_address;
    scommand.DataMode          = QSPI_DATA_1_LINE;
    scommand.NbData            = size;
    scommand.DummyCycles       = 0;

    platform_watchdog_kick();

    VERIFY_RESULT(HAL_QSPI_Command((QSPI_HandleTypeDef*)handle->platform_peripheral, &scommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));
    VERIFY_RESULT(HAL_QSPI_Transmit((QSPI_HandleTypeDef*)handle->platform_peripheral, (uint8_t *)data_addr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

    /* Wait till busy flag is set */
    VERIFY_RESULT(sflash_wait_busy_done((QSPI_HandleTypeDef*)handle->platform_peripheral, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));
    return 0;
}

int sflash_write( const sflash_handle_t* const handle, unsigned long device_address,  /*@observer@*/ const void* const data_addr, unsigned int size )
{
    uint32_t number_of_pages = 0, NumOfSingle = 0, address = 0, count = 0, temp = 0, page_offset = 0;
    uint32_t write_size = 0;
    char *buffer = (char *)data_addr;

    address = device_address;
    page_offset = device_address % QSPI_PAGE_SIZE;
    count = QSPI_PAGE_SIZE - page_offset;
    number_of_pages = size / QSPI_PAGE_SIZE;
    NumOfSingle = size % QSPI_PAGE_SIZE;

    if ( page_offset == 0 ) /* Address is QSPI_PAGESIZE aligned  */
    {
        if ( number_of_pages == 0 ) /* Bytes to write < QSPI_PAGESIZE */
        {
            write_size = size;
            VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
        }
        else /* size > QSPI_PAGESIZE */
        {
            while ( number_of_pages-- )
            {
                write_size = QSPI_PAGE_SIZE;
                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
                address += QSPI_PAGE_SIZE;
                buffer += QSPI_PAGE_SIZE;
            }

            write_size = NumOfSingle;
            VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
        }
    }
    else /* address is not QSPI_PAGESIZE aligned  */
    {
        if ( number_of_pages == 0 ) /* size < QSPI_PAGESIZE */
        {
            if ( NumOfSingle > count ) /* (size + address) > QSPI_PAGESIZE */
            {
                temp = NumOfSingle - count;
                write_size = count;
                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
                address += count;
                buffer += count;

                write_size = temp;
                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
            }
            else
            {
                write_size = size;
                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
            }
        }
        else /* size > QSPI_PAGESIZE */
        {
            size -= count;
            number_of_pages = size / QSPI_PAGE_SIZE;
            NumOfSingle = size % QSPI_PAGE_SIZE;

            write_size = count;

            VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
            address += count;
            buffer += count;

            while ( number_of_pages-- )
            {
                write_size = QSPI_PAGE_SIZE;

                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
                address += QSPI_PAGE_SIZE;
                buffer += QSPI_PAGE_SIZE;
            }

            if ( NumOfSingle != 0 )
            {
                write_size = NumOfSingle;

                VERIFY_RESULT( sflash_page_write( handle, address, buffer, write_size ) );
            }
        }
    }

    return 0;
}

int sflash_get_size( const sflash_handle_t* handle, unsigned long* size )
{
    UNUSED_PARAMETER(handle);
    *size = SFLASH_SIZE_16MBYTE;
    return 0;
}

int sflash_chip_erase( const sflash_handle_t* handle )
{
    QSPI_CommandTypeDef scommand = {0};

    /* Write enable */
    VERIFY_RESULT(sflash_write_enable(((QSPI_HandleTypeDef*)handle->platform_peripheral)));

    scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    scommand.Instruction       = BULK_ERASE_CMD;
    scommand.AddressMode       = QSPI_ADDRESS_NONE;
    scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    scommand.DataMode          = QSPI_DATA_NONE;
    scommand.DummyCycles       = 0;
    scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    VERIFY_RESULT(HAL_QSPI_Command((QSPI_HandleTypeDef*)handle->platform_peripheral, &scommand, BULK_ERASE_TIME_MS));

    /* Wait till busy flag is set */
    VERIFY_RESULT(sflash_wait_busy_done((QSPI_HandleTypeDef*)handle->platform_peripheral, BULK_ERASE_TIME_MS));
    return 0;
}

static int platform_check_for_non_ff ( uint8_t *loc, uint32_t size )
{
    uint32_t i;

    for ( i = 0; i < size; i++ )
    {
        if ( loc[i] != 0xff )
        {
            return 1;
        }
    }

    return 0;
}

static int sflash_need_to_erase ( const sflash_handle_t* handle, unsigned long device_address, unsigned int size )
{
    uint8_t read_mem_buf[PACKET_SIZE];
    uint32_t completed_check_cnt = 0;
    uint32_t cur_addr = device_address;
    uint32_t cur_read_size;
    while ( completed_check_cnt < size )
    {
        if ( ( size - completed_check_cnt ) > PACKET_SIZE )
        {
            cur_read_size = PACKET_SIZE;
        }
        else
        {
            cur_read_size = size - completed_check_cnt;
        }

        VERIFY_RESULT(sflash_read( handle, device_address, read_mem_buf , cur_read_size ));

        if ( platform_check_for_non_ff ( read_mem_buf, cur_read_size ) )
        {
             DEBUG_PRINTS(("\r\nNEED TO ERASE!!!!! %s: Address: 0x%04lX, Size: %u\r\n\r\n", __func__, device_address, size));
             return 1;
        }

        completed_check_cnt += cur_read_size;
        cur_addr += cur_read_size;
    }

    return 0;
}

static int platform_get_backup_sector( uint32_t* backup_sector_start, uint32_t* backup_sector_size )
{
   *backup_sector_start = backup_sector_address;
   *backup_sector_size = QSPI_SECTOR_SIZE;

   return 0;
}

static int platform_copy_sflash_src_to_dest ( const sflash_handle_t* handle, unsigned long src, unsigned long dest, unsigned int size )
{
    uint8_t read_mem_buf[PACKET_SIZE];
    uint32_t completed_check_cnt = 0;
    uint32_t cur_src_addr = src;
    uint32_t cur_dst_addr = dest;
    uint32_t cur_read_size;
    while ( completed_check_cnt < size )
    {
        if ( ( size - completed_check_cnt ) > PACKET_SIZE )
        {
            cur_read_size = PACKET_SIZE;
        }
        else
        {
            cur_read_size = size - completed_check_cnt;
        }

        VERIFY_RESULT(sflash_read( handle, cur_src_addr, read_mem_buf, cur_read_size ));

        if ( platform_check_for_non_ff ( read_mem_buf, cur_read_size ) )
        {
            VERIFY_RESULT(sflash_write( handle, cur_dst_addr, read_mem_buf, cur_read_size ));
        }
        else
        {
             DEBUG_PRINTS( ( "%s: No need to copy from: 0x%04lX to: 0x%04lX\r\n", __func__, cur_src_addr, cur_dst_addr ) );
        }
        completed_check_cnt += cur_read_size;
        cur_src_addr += cur_read_size;
        cur_dst_addr += cur_read_size;
    }

    return ( 0 );
}

int sflash_sector_erase( const sflash_handle_t* handle, unsigned long device_address )
{
    VERIFY_RESULT(sflash_erase( handle, device_address, WICED_SECTOR_SIZE) );
    return 0;
}

/* Erases Sector , size of sector is dependent on the external flash */
static int platform_sector_erase( const sflash_handle_t* handle, unsigned long device_address, unsigned int size )
{
    QSPI_CommandTypeDef scommand = {0};

    /* Write enable */
    VERIFY_RESULT(sflash_write_enable(((QSPI_HandleTypeDef*)handle->platform_peripheral)));

    scommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    scommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    scommand.Instruction       = SECTOR_ERASE_CMD;
    scommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    scommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    scommand.DataMode          = QSPI_DATA_NONE;
    scommand.DummyCycles       = 0;
    scommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    scommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    scommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    scommand.Address           = device_address;

    VERIFY_RESULT(HAL_QSPI_Command((QSPI_HandleTypeDef*)handle->platform_peripheral, &scommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

    /* Wait till busy flag is set */
    VERIFY_RESULT(sflash_wait_busy_done((QSPI_HandleTypeDef*)handle->platform_peripheral, HAL_QPSI_TIMEOUT_DEFAULT_VALUE));

    return 0;
}

static int sflash_erase( const sflash_handle_t* handle, unsigned long device_address, unsigned int size )
{
    uint32_t backup_sector_start, backup_sector_size, offset_in_sector;
    uint32_t save_first_start_address, save_first_end_address, save_second_start_address, save_second_end_address;
    uint32_t save_first_copy_size, save_second_copy_size;
    uint32_t erase_sector_size = QSPI_SECTOR_SIZE;

    if ( sflash_need_to_erase (handle, device_address, WICED_SECTOR_SIZE) == 0 )
    {
        DEBUG_PRINTS( ( "\r\n%s: Address: 0x%04lX, Size: %u, No Need to Earse!!!!!\r\n\r\n", __func__, device_address, size ) );
        return 0;
    }

    if ( ( device_address % erase_sector_size == 0 ) && ( size == erase_sector_size ) )
    {
        DEBUG_PRINTS( ( "\r\n%s: Address: 0x%04lX, Size: %u, Only Erase - No need to save/restore other parts of flash!!!!!\r\n\r\n", __func__, device_address, size) );
        VERIFY_RESULT(platform_sector_erase( handle, device_address, size ));
    }
    else /* Need to back up */
    {
        save_first_start_address = ( device_address / erase_sector_size ) * erase_sector_size;
        save_first_end_address = device_address;
        save_first_copy_size = save_first_end_address - save_first_start_address;
        save_second_start_address = device_address + size;
        save_second_end_address = ( ( device_address / erase_sector_size ) + 1 ) * erase_sector_size;
        save_second_copy_size = save_second_end_address - save_second_start_address;

        DEBUG_PRINTS( ( "\r\n%s: Address: 0x%08lX, Size: %u, Needs save/restore other parts of flash!!!!!\r\n\r\n", __func__, device_address, size ) );
        DEBUG_PRINTS( ( "\r\n%s: Save first start Address: 0x%08lX, Save first Address End: 0x%08lX\r\n", __func__, save_first_start_address, save_first_end_address ) );
        DEBUG_PRINTS( ( "\r\n%s: Save second start Address: 0x%08lX, Save second Address End: 0x%08lX\r\n", __func__, save_second_start_address, save_second_end_address ) );

        platform_get_backup_sector(&backup_sector_start, &backup_sector_size);

        VERIFY_RESULT(platform_sector_erase(handle, backup_sector_start, backup_sector_size));

        DEBUG_PRINTS( ( "\r\n%s: Backup Sector Start: 0x%08lX, Backup Sector Size: 0x%08lX\r\n", __func__, backup_sector_start, backup_sector_size ) );
        DEBUG_PRINTS( ( "Saving Sector contents to Backup before Erase!!\r\n" ) );
        DEBUG_PRINTS( ( "\r\n%s: First Part!! Copy from: 0x%08lX, Copy To: 0x%08lX, Size: 0x%08lX\r\n", __func__,
                        save_first_start_address, backup_sector_start, save_first_copy_size ) );

        VERIFY_RESULT(platform_copy_sflash_src_to_dest( handle, save_first_start_address, backup_sector_start, save_first_copy_size ));

        offset_in_sector = ( save_second_start_address & (erase_sector_size - 1) );

        DEBUG_PRINTS( ( "\r\n%s: Second Part!! Copy from: 0x%08lX, Copy To: 0x%08lX, Size: 0x%08lX\r\n", __func__,
                     save_second_start_address, ( backup_sector_start + offset_in_sector ), save_second_copy_size ) );

        platform_copy_sflash_src_to_dest( handle, save_second_start_address, ( backup_sector_start + offset_in_sector ),
                         save_second_copy_size );

        VERIFY_RESULT(platform_sector_erase( handle, device_address, size ));

        DEBUG_PRINTS( ( "Restoring Sector contents from Backup after Erase!!\r\n" ) );
        DEBUG_PRINTS( ( "\r\n%s: First Part!! Copy from: 0x%08lX, Copy To: 0x%08lX, Size: 0x%08lX\r\n", __func__,
                      backup_sector_start, save_first_start_address, save_first_copy_size ) );

        VERIFY_RESULT(platform_copy_sflash_src_to_dest( handle, backup_sector_start, save_first_start_address, save_first_copy_size ));

        DEBUG_PRINTS( ( "\r\n%s: Second Part!! Copy from: 0x%08lX, Copy To: 0x%08lX, Size: 0x%08lX\r\n", __func__,
                     ( backup_sector_start + offset_in_sector ), save_second_start_address, save_second_copy_size ) );

        VERIFY_RESULT(platform_copy_sflash_src_to_dest ( handle, ( backup_sector_start + offset_in_sector ), save_second_start_address, save_second_copy_size ));

        DEBUG_PRINTS( ( " Sector Erase With Store & Restore Completed!!\r\n" ) );
    }

    return 0;
}
