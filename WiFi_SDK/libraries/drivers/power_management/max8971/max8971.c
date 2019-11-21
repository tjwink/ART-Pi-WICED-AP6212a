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

/** @file Maxim8971 Library Functions
 *
 */

#include "max8971.h"
#include <stdio.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RETRIES 5

/* Maxim8971 register addresses */
#define INTERRUPT_REGISTER_ADDRESS     0x0F
#define STATUS_REGISTER_ADDRESS        0x02
#define DETAILS1_REGISTER_ADDRESS      0x03
#define DETAILS2_REGISTER_ADDRESS      0x04
#define FAST_CHARGE_REGISTER_ADDRESS   0x06
#define CURRENT_LIMIT_REGISTER_ADDRESS 0x07
#define TOPOFF_REGISTER_ADDRESS        0x08
#define CHARGER_LOCK_REGISTER_ADDRESS  0x0A

/* Overview of charger status */
#define STATUS_VOLTAGE_IN_ERROR 0x80
#define STATUS_CURRENT_IN_ERROR 0x40
#define STATUS_CHARGER_ERROR    0x08
#define STATUS_BATTERY_ERROR    0x04
#define STATUS_THERMISTOR_ERROR 0x02

/* Specific battery and charger status */
#define DETAILS2_BATTERY_DETAILS        0x30
#define DETAILS2_BATTERY_DETAILS_OFFSET 4
#define DETAILS2_BATTERY_MESSAGE_COUNT  4
#define DETAILS2_CHARGER_MESSAGE_COUNT  10
#define DETAILS2_CHARGER_DETAILS        0x0F

/* Specific thermistor status */
#define DETAILS1_THERMISTOR_DETAILS       0x07
#define DETAILS1_THERMISTOR_MESSAGE_COUNT 5

/* Set target charge current
 *
 * Fast charge Amp <= Current limit Amp
 */
#define FAST_CHARGE_SETTINGS_SLOW   0x4A /* 5 hour timer, fast charge at 0.5 Amp */
#define FAST_CHARGE_SETTINGS_MEDIUM 0x54 /* 5 hour timer, fast charge at 1.0 Amp */
#define FAST_CHARGE_SETTINGS_FAST   0x5E /* 5 hour timer, fast charge at 1.5 Amp */

/* Set target input current
 *
 * Fast charge Amp <= Current limit Amp
 */
#define CURRENT_LIMIT_SETTINGS_LOW    0x14 /* Input current limit = 0.5 Amp */
#define CURRENT_LIMIT_SETTINGS_MEDIUM 0x28 /* Input current limit = 1.0 Amp */
#define CURRENT_LIMIT_SETTINGS_HIGH   0x3C /* Input current limit = 1.5 Amp */

/* 100mA threshold charges more completely,
 * 200mA threshold charges faster.
 */
#define TOPOFF_TIMER_SETTINGS 0x24 /* Topoff timer = 10 minutes, 100mA threshold */
//#define TOPOFF_TIMER_SETTINGS 0x2C /* Topoff timer = 10 minutes, 200mA threshold */

/* Allows settings to be changed */
#define CHARGER_PROTECTION_LOCK   0x00
#define CHARGER_PROTECTION_UNLOCK 0x0C

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_i2c_device_t maxim8971 =
{
    .port          = WICED_I2C_1,
    .address       = 0x35,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .flags         = 0,
    .speed_mode    = I2C_HIGH_SPEED_MODE,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t max8971_initialize()
{
    if ( ( wiced_i2c_init(&maxim8971) != WICED_SUCCESS ) ||
         ( wiced_i2c_probe_device(&maxim8971, RETRIES) != WICED_TRUE ) )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Sequence to read data from specified registers on maxim chips.
 *
 * This consists of [Start] [Write (slave address) (register address)] [Repeat-Start] [Read] [Stop].
 */
static wiced_result_t maxim_i2c_reg_read(uint8_t* write_buffer, uint8_t write_buffer_size, uint8_t* read_buffer, uint8_t read_buffer_size)
{
    /* Write slave address of maxim and register address to read from. */
    if (wiced_i2c_write(&maxim8971, WICED_I2C_START_FLAG, write_buffer, write_buffer_size) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Read bytes from maxim register address into read_buffer. */
    if (wiced_i2c_read(&maxim8971, WICED_I2C_REPEATED_START_FLAG | WICED_I2C_STOP_FLAG, read_buffer, read_buffer_size) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Allows data to be written to register addresses 0x06, 0x07, 0x08, and 0x09. */
static void unlock_charger_settings()
{
    uint8_t write_data[2] = {CHARGER_LOCK_REGISTER_ADDRESS, CHARGER_PROTECTION_UNLOCK};
    wiced_i2c_message_t message;

    if ( ( wiced_i2c_init_tx_message(&message, write_data, 2, RETRIES, I2C_DEVICE_NO_DMA) != WICED_SUCCESS ) ||
         ( wiced_i2c_transfer(&maxim8971, &message, 1) != WICED_SUCCESS ) )
    {
        printf("Failed to unlock charger protection register\n");
    }
}

/* Prevents data from being written to register addresses 0x06, 0x07, 0x08, and 0x09. */
static void lock_charger_settings()
{
    uint8_t write_data[2] = {CHARGER_LOCK_REGISTER_ADDRESS, CHARGER_PROTECTION_LOCK};
    wiced_i2c_message_t message;

    if ( ( wiced_i2c_init_tx_message(&message, write_data, 2, RETRIES, I2C_DEVICE_NO_DMA) != WICED_SUCCESS ) ||
         ( wiced_i2c_transfer(&maxim8971, &message, 1) != WICED_SUCCESS ) )
    {
        printf("Failed to lock charger protection register\n");
    }
}

uint8_t max8971_get_thermistor_details()
{
    uint8_t det1_reg = DETAILS1_REGISTER_ADDRESS;
    uint8_t details1 = 0x00;

    maxim_i2c_reg_read(&det1_reg, 1, &details1, 1);

    return details1;
}

void max8971_print_thermistor_details(uint8_t details)
{
    char* thermistor_messages[DETAILS1_THERMISTOR_MESSAGE_COUNT + 1] =
    {
        "ERROR\n", "Low temperature - Not Charging\n", "Low temperature\n", "Standard temperature\n",
        "High temperature\n", "High temperature - Not Charging\n"
    };

    printf(thermistor_messages[details & DETAILS1_THERMISTOR_DETAILS]);
}

uint8_t max8971_get_charger_and_battery_details()
{
    uint8_t det2_reg = DETAILS2_REGISTER_ADDRESS;
    uint8_t details2 = 0x00;

    maxim_i2c_reg_read(&det2_reg, 1, &details2, 1);

    return details2;
}

void max8971_print_charger_and_battery_details(uint8_t details)
{
    char* battery_messages[DETAILS2_BATTERY_MESSAGE_COUNT] =
    {
        "Battery is dead\n", "Slow to charge - Battery may be damaged\n", "Battery is good\n", "Battery is overcharged\n"
    };
    char* charger_messages[DETAILS2_CHARGER_MESSAGE_COUNT] =
    {
        "Charging\n", "Preparing to charge\n", "Fast Charging - Constant current\n", "Fast Charging - Constant voltage\n", "Topping off\n", "Done charging\n",
        "Not Charging - Charge duration exceeded timer\n", "Not Charging - Outside temperature operating range\n",
        "Not Charging - Charger is not connected\n", "Preparing to fast charge or top-off\n"
    };

    printf(battery_messages[(details & DETAILS2_BATTERY_DETAILS) >> DETAILS2_BATTERY_DETAILS_OFFSET]);
    printf(charger_messages[details & DETAILS2_CHARGER_DETAILS]);
}

uint8_t max8971_get_charger_status()
{
    uint8_t status_register = STATUS_REGISTER_ADDRESS;
    uint8_t status_byte = 0x00;

    maxim_i2c_reg_read(&status_register, 1, &status_byte, 1);

    return status_byte;
}

void max8971_print_charger_status(uint8_t status)
{
    if ( ( ( status & STATUS_VOLTAGE_IN_ERROR ) != 0 ) ||
           ( status & STATUS_CURRENT_IN_ERROR ) != 0 )
    {
        printf("Input out of expected range\n");
    }

    max8971_print_charger_and_battery_details(max8971_get_charger_and_battery_details());
    max8971_print_thermistor_details(max8971_get_thermistor_details());

}

uint8_t max8971_get_interrupt_requests()
{
    uint8_t interrupt_register = INTERRUPT_REGISTER_ADDRESS;
    uint8_t interrupt_byte = 0x00;

    maxim_i2c_reg_read(&interrupt_register, 1, &interrupt_byte, 1);

    return interrupt_byte;
}

/* Sets maximum possible charging speed (0.5 Amp - 1.5 Amp).
 *
 * Note: Actual charging cannot exceed input amp limit.
 */
static void max8971_set_fast_charge(uint8_t speed_setting)
{
    uint8_t write_data[2] = {FAST_CHARGE_REGISTER_ADDRESS, speed_setting};
    wiced_i2c_message_t message;

    unlock_charger_settings();

    if ( ( wiced_i2c_init_tx_message(&message, write_data, 2, RETRIES, I2C_DEVICE_NO_DMA) != WICED_SUCCESS ) ||
         ( wiced_i2c_transfer(&maxim8971, &message, 1) != WICED_SUCCESS ) )
    {
        printf("Failed to set fast-charge speed\n");
    }

    lock_charger_settings();
}

/* Sets maximum possible input current (0.5 - 1.5 Amp). */
static void max8971_set_current_limit(uint8_t current_setting)
{
    uint8_t write_data[2] = {CURRENT_LIMIT_REGISTER_ADDRESS, current_setting};
    wiced_i2c_message_t message;

    unlock_charger_settings();

    if ( ( wiced_i2c_init_tx_message(&message, write_data, 2, RETRIES, I2C_DEVICE_NO_DMA) != WICED_SUCCESS ) ||
         ( wiced_i2c_transfer(&maxim8971, &message, 1) != WICED_SUCCESS ) )
    {
        printf("Failed to set input current limit\n");
    }

    lock_charger_settings();
}

void max8971_set_topoff_timer()
{
    uint8_t write_data[2] = {TOPOFF_REGISTER_ADDRESS, TOPOFF_TIMER_SETTINGS};
    wiced_i2c_message_t message;

    unlock_charger_settings();

    if ( ( wiced_i2c_init_tx_message(&message, write_data, 2, RETRIES, I2C_DEVICE_NO_DMA) != WICED_SUCCESS ) ||
         ( wiced_i2c_transfer(&maxim8971, &message, 1) != WICED_SUCCESS ) )
    {
        printf("Failed to set topoff timer\n");
    }

    lock_charger_settings();
}

void max8971_set_charge_speed(maxim_charge_speed_t speed)
{
    uint8_t fast_charge_settings, current_limit_settings;

    switch (speed)
    {
        case MAXIM_CHARGE_SPEED_MEDIUM:
            fast_charge_settings = FAST_CHARGE_SETTINGS_MEDIUM;
            current_limit_settings = CURRENT_LIMIT_SETTINGS_MEDIUM;
            break;
        case MAXIM_CHARGE_SPEED_FAST:
            fast_charge_settings = FAST_CHARGE_SETTINGS_FAST;
            current_limit_settings = CURRENT_LIMIT_SETTINGS_HIGH;
            break;
        default:
            fast_charge_settings = FAST_CHARGE_SETTINGS_SLOW;
            current_limit_settings = CURRENT_LIMIT_SETTINGS_LOW;
    }

    max8971_set_fast_charge(fast_charge_settings);
    max8971_set_current_limit(current_limit_settings);
}

void max8971_print_all_registers_debug()
{
    uint8_t register_address = 0x0F;
    uint8_t register_buffer[11] = {0};

    maxim_i2c_reg_read(&register_address, 1, register_buffer, 1);
    for (register_address = 0x01; register_address < 0x0B; ++register_address)
    {
        maxim_i2c_reg_read(&register_address, 1, &register_buffer[register_address], 1);
    }

    for (register_address = 0; register_address < 11; ++register_address)
    {
        if (register_address == 0)
        {
            printf("Reg: 0x0F - 0x%02X\n", register_buffer[register_address]);
        }
        else
        {
            printf("Reg: 0x%02X - 0x%02X\n", register_address, register_buffer[register_address]);
        }
    }
}
