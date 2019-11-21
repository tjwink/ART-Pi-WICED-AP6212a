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
 * WICED Power Logger(WPL) is a tool to estimate the power of the target
 * using software. It is divided in to two components as below
 * Host WPL : UI based tool to visualize power events graphically.
 * Target WPL : Under WICED/WPL, which will provide the power events
 * data to Host WPL.
 * This header will expose two API's as below
 * API : WICED_POWER_LOGGER --> Tapping function, required to log the events.
 * API : wpl_start --> Required to start WPL on the target.
 */

#ifndef __CPL_H_
#define __CPL_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 * @brief Processor ID's
 *
 */
typedef enum {
    EVENT_PROC_ID_MCU,       /**< MCU Processor ID. */
    EVENT_PROC_ID_WIFI,      /**< Wi-Fi Processor ID. */
    EVENT_PROC_ID_BT,        /**< BT Processor ID. */
    EVENT_PROC_ID_MAX,       /**< Processor ID Count. */
} cpl_procid_t;

/**
 * @brief Event ID's
 *
 */
typedef enum {
    EVENT_ID_POWERSTATE,     /**< Power State Event ID. */
    EVENT_ID_FLASH,          /**< Flash Event ID. */
    EVENT_ID_UART,           /**< UART Event ID. */
    EVENT_ID_WIFI_DATA,      /**< Wi-Fi Event ID. */
    EVENT_ID_I2S,            /**< I2S Event ID. */
    EVENT_ID_PROFILING,      /**< Function Profiling Event ID. */
    EVENT_ID_BT_DATA,        /**< BT Event ID. */
    EVENT_ID_I2C,            /**< I2C Event ID. */
    EVENT_ID_SPI_SFLASH,     /**< SPI_SFLASH Event ID. */
    EVENT_ID_SDIO,           /**< SDIO Event ID. */
    EVENT_ID_SPI_1,          /**< SPI_1 Event ID. */
    EVENT_ID_MAX,            /**< Event ID Count. */
} cpl_event_id_t;

/**
 * @brief MCU Power Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_POWER_ACTIVE1,     /**< Active State-1 Descriptor ID. */
    EVENT_DESC_POWER_ACTIVE2,     /**< Active State-2 Descriptor ID. */
    EVENT_DESC_POWER_SLEEP,       /**< Sleep State Descriptor ID. */
    EVENT_DESC_POWER_DEEPSLEEP,   /**< Deep Sleep State Descriptor ID. */
    EVENT_DESC_POWER_OFF,         /**< Off State Descriptor ID. */
    EVENT_DESC_POWER_HIBERNATE,   /**< Hibernate State Descriptor ID. */
    EVENT_DESC_POWER_PDS,         /**< PDS State Descriptor ID. */
    EVENT_DESC_MAX,               /**< Power Descriptor ID Count. */
} cpl_event_power_state_t;

/**
 * @brief Bluetooth Event's Descriptor's
 *
 */
typedef enum
{
    EVENT_DESC_BT_POWER_OFF,            /**< BT Off State Descriptor ID. */
    EVENT_DESC_BT_POWER_IDLE,           /**< BT Idle State Descriptor ID. */
    EVENT_DESC_BT_POWER_TX,             /**< BT Tx Descriptor ID. */
    EVENT_DESC_BT_POWER_RX,             /**< BT Rx Descriptor ID. */
    EVENT_DESC_BT_POWER_SLEEP,          /**< BT Sleep State Descriptor ID. */
    EVENT_DESC_BT_POWER_DEEP_SLEEP,     /**< BT Deep Sleep Descriptor ID. */
    EVENT_DESC_BT_MAX,                  /**< BT Descriptor ID Count. */
} cpl_event_bt_power_state_t;

/**
 * @brief UART Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_UART_IDLE,            /**< UART Idle State Descriptor ID. */
    EVENT_DESC_UART_TX,              /**< UART tx State Descriptor ID. */
    EVENT_DESC_UART_RX,              /**< UART rx State Descriptor ID. */
    EVENT_DESC_UART_MAX,             /**< UART Descriptor ID Count. */
} cpl_event_uart_state_t;

/**
 * @brief I2C Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_I2C_IDLE,            /**< I2C Idle State Descriptor ID. */
    EVENT_DESC_I2C_TX,              /**< I2C tx State Descriptor ID. */
    EVENT_DESC_I2C_RX,              /**< I2C rx State Descriptor ID. */
    EVENT_DESC_I2C_MAX,             /**< I2C Descriptor ID Count. */
} cpl_event_i2c_state_t;

/**
 * @brief SPI-SFLASH Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_SPI_SFLASH_IDLE,     /**< SPI-SFLASH Idle State Descriptor ID. */
    EVENT_DESC_SPI_SFLASH_READ,     /**< SPI-SFLASH Read State Descriptor ID. */
    EVENT_DESC_SPI_SFLASH_WRITE,    /**< SPI-SFLASH Write State Descriptor ID. */
    EVENT_DESC_SPI_SFLASH_ERASE,    /**< SPI-SFLASH Erase State Descriptor ID. */
    EVENT_DESC_SPI_SFLASH_MAX,      /**< I2C Descriptor ID Count. */
} cpl_event_spi_sflash_state_t;

/**
 * @brief SDIO Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_SDIO_IDLE,     /**< SDIO Idle State Descriptor ID. */
    EVENT_DESC_SDIO_READ,     /**< SDIO Read State Descriptor ID. */
    EVENT_DESC_SDIO_WRITE,    /**< SDIO Write State Descriptor ID. */
    EVENT_DESC_SDIO_MAX,      /**< SDIO Descriptor ID Count. */
} cpl_event_sdio_state_t;

/**
 * @brief SDIO Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_SPI_OFF,      /**< SPI OFF State Descriptor ID. */
    EVENT_DESC_SPI_IDLE,     /**< SPI Idle State Descriptor ID. */
    EVENT_DESC_SPI_READ,     /**< SPI Read State Descriptor ID. */
    EVENT_DESC_SPI_WRITE,    /**< SPI Write State Descriptor ID. */
    EVENT_DESC_SPI_MAX,      /**< SPI Descriptor ID Count. */
} cpl_event_spi_state_t;

/**
 * @brief Function Profiling Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_FUNC_IDLE,            /**< Function Idle State Descriptor ID. */
    EVENT_DESC_FUNC_TIME,            /**< Function Time State Descriptor ID. */
} cpl_event_profiling_state_t;

/**
 * @brief Wi-Fi Power Event Descriptor's
 *
 */
typedef enum {
    EVENT_DESC_WIFI_IDLE,            /**< Wi-Fi idle State Descriptor ID. */
    EVENT_DESC_WIFI_BAND,            /**< Wi-Fi Band Descriptor ID. */
    EVENT_DESC_WIFI_BW,              /**< Wi-Fi Bandwidth Descriptor ID. */
    EVENT_DESC_WIFI_PMMODE,          /**< Wi-Fi PM mode Descriptor ID. */
    EVENT_DESC_WIFI_RATE_TYPE,       /**< Wi-Fi Rate Type Descriptor ID. */
    EVENT_DESC_WIFI_RATE0,           /**< Wi-Fi Rate0 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE1,           /**< Wi-Fi Rate1 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE2,           /**< Wi-Fi Rate2 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE3,           /**< Wi-Fi Rate3 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE4,           /**< Wi-Fi Rate4 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE5,           /**< Wi-Fi Rate5 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE6,           /**< Wi-Fi Rate6 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE7,           /**< Wi-Fi Rate7 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE8,           /**< Wi-Fi Rate8 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_RATE9,           /**< Wi-Fi Rate9 tx/rx counters data Descriptor ID. */
    EVENT_DESC_WIFI_MAX,             /**< Wi-Fi Descriptor ID Count. */
} cpl_event_wifi_state_t;

typedef enum {
    EVENT_DESC_WIFI_MCS_RATE = 1,    /**< Wi-Fi MCS Rate Type ID. */
    EVENT_DESC_WIFI_VHT_RATE,        /**< Wi-Fi VHT Rate Type ID. */
} cpl_event_wifi_rate_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/

/**
 * Use below function to tap and get the power events generated
 *
 * @param           proc_id : Processor ID
 * @param           event_id : Event ID
 * @param           event_state : Descriptor ID
 * @return          NO return value.
 */

#ifdef WICED_POWER_LOGGER_ENABLE
void cpl_event_state_update( uint8_t proc_id, uint8_t event_id, uint8_t event_state );
#define WICED_POWER_LOGGER( proc_id, event_id, event_state ) cpl_event_state_update( proc_id, event_id, event_state )
#else
#define WICED_POWER_LOGGER( proc_id, event_id, event_state )
#endif

/**
 * Use below function to start the target WPL
 *
 * @param           NO parameters required.
 * @return          NO return value.
 */
#ifdef WICED_POWER_LOGGER_ENABLE
void wpl_start(void);
#define wpl_start() wpl_start()
#else
#define wpl_start()
#endif

#ifdef __cplusplus
extern "C" }
#endif

#endif
