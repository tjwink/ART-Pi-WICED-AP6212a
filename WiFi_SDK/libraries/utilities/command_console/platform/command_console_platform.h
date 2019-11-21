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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef BCM43909
#define PLATFORM_COMMANDS_BCM43909_SPECIFIC \
    { "hibernation",         hibernation_console_command,         1, NULL, NULL, "<sleep_ms>", "Force chip to hibernate for specified amount of milliseconds"}, \
    { "mcu_powersave_clock", mcu_powersave_clock_console_command, 2, NULL, NULL, "<0|1> <0|1|2|3|4>", "<Clock request or release: 0 - release, 1 - request> <Which clock: 0 - ALP available, 1 - HT available, 2 - have at least ILP on backplane, 3 - at least ALP, 4 - at least HT>"}, \
    { "mcu_powersave_tick",  mcu_powersave_tick_console_command,  1, NULL, NULL, "<0|1|2>", "<RTOS tick mode: 0 - always tickless, 1 - never tickless, 2 - tickless if MCU power-save enabled>"}, \
    { "mcu_powersave_freq",  mcu_powersave_freq_console_command,  1, NULL, NULL, "<freq_mode>", "<CPU/backplane frequency mode>"}, \
    { "mcu_powersave_sleep", mcu_powersave_sleep_console_command, 2, NULL, NULL, "<0|1> <sleep_ms>", "0 - RTOS sleeping with CPU can wake-up earlier if requested by other threads though current one remain in sleep state, 1 - forced sleeping where platform forced to ignore all interrupts except timer and sleep specified amount of time"}, \
    { "mcu_powersave_info",  mcu_powersave_info_console_command,  0, NULL, NULL, NULL, "Print powersave information"}, \
    { "mcu_wlan_powersave_stats",          mcu_wlan_powersave_stats_console_command,  0, NULL, NULL, NULL, "Print WLAN powersave statistics"}, \
    { "mcu_powersave_gpio_wakeup_enable",  mcu_powersave_gpio_wakeup_enable_console_command,  2, NULL, NULL, "<input_pin_pull_mode> <trigger>", "Enable wakening up from deep-sleep via GPIO"}, \
    { "mcu_powersave_gpio_wakeup_disable", mcu_powersave_gpio_wakeup_disable_console_command, 0, NULL, NULL, NULL, "Disable wakening up from deep-sleep via GPIO"}, \
    { "mcu_powersave_gpio_wakeup_ack",     mcu_powersave_gpio_wakeup_ack_console_command,     0, NULL, NULL, NULL, "If GPIO generated wake up event it remain triggered till acked"}, \
    { "mcu_powersave_gci_gpio_wakeup_enable",  mcu_powersave_gci_gpio_wakeup_enable_console_command,  3, NULL, NULL, "<pin> <input_pin_pull_mode> <trigger>", "Enable wakening up from deep-sleep via GPIO"}, \
    { "mcu_powersave_gci_gpio_wakeup_disable", mcu_powersave_gci_gpio_wakeup_disable_console_command, 1, NULL, NULL, "<pin>", "Disable wakening up from deep-sleep via GPIO"}, \
    { "mcu_powersave_gci_gpio_wakeup_ack",     mcu_powersave_gci_gpio_wakeup_ack_console_command,     1, NULL, NULL, "<pin>", "If GPIO generated wake up event it remain triggered till acked"},
#else
#define PLATFORM_COMMANDS_BCM43909_SPECIFIC
#endif

#if defined(MCU_BCM920739) || defined(BCM43909)
#define PLATFORM_COMMANDS_FOR_POWERSAVE \
        { "mcu_powersave_mode",  mcu_powersave_mode_console_command,  1, NULL, NULL, "<0|1>", "<MCU powersave mode: 0 - deep-sleep, 1 - normal sleep>"},
#else
#define PLATFORM_COMMANDS_FOR_POWERSAVE
#endif

#define PLATFORM_COMMANDS \
    { "reboot",             reboot_console_command,        0, NULL, NULL, NULL,    "Reboot the device"}, \
    { "get_time",           get_time_console_command,      0, NULL, NULL, NULL,    "Print current time"}, \
    { "sleep",              sleep_console_command,         1, NULL, NULL, NULL,    "Sleep number of milliseconds"}, \
    { "prng_bit_dump",      prng_bit_dump_command,         1, NULL, NULL, "<num>", "Number bytes to print using 1s and 0s"}, \
    { "prng",               prng_console_command,          1, NULL, NULL, "<num>", "Bytes number"}, \
    { "prng_set_algorithm", prng_set_algorithm_command,    1, NULL, NULL, "<string>", "Name of algorithm from prng_get_algorithms"}, \
    { "prng_get_algorithms",prng_get_algorithms_command,   0, NULL, NULL, "", ""}, \
    { "mcu_powersave",      mcu_powersave_console_command, 1, NULL, NULL, "<0|1>", "Enable/disable MCU powersave"}, \
    { "wiced_init",         wiced_init_console_command,    1, NULL, NULL, "<0|1>", "Call wiced_deinit/wiced_init"}, \
    { "loglevel_set",       wiced_log_set_level_console_command,  1, NULL, NULL, "[facility] <loglevel>", "set new wiced logging level"}, \
    { "loglevel_get",       wiced_log_get_level_console_command,  0, NULL, NULL, NULL, "get current wiced logging levels for all facilities"}, \
    PLATFORM_COMMANDS_FOR_POWERSAVE \
    PLATFORM_COMMANDS_BCM43909_SPECIFIC

#ifdef WICED_PLATFORM_INCLUDES_SPI_FRAM

#define FRAM_COMMANDS \
    { (char*) "fram_read", fram_read, 2, NULL, NULL, (char*) " [address] [No. of Kbytes to read]", (char*) "Dump memory"}, \
    { (char*) "fram_write", fram_write, 2, NULL, NULL, (char*) " [address] [No. of Kbytes to write]", (char*) "Write memory"}, \

#else
#define FRAM_COMMANDS
#endif


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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

int reboot_console_command( int argc, char* argv[] );
int get_time_console_command( int argc, char* argv[] );
int sleep_console_command( int argc, char* argv[] );
int prng_console_command( int argc, char* argv[] );
int prng_bit_dump_command( int argc, char* argv[] );
int prng_set_algorithm_command( int argc, char* argv[] );
int prng_get_algorithms_command( int argc, char* argv[] );
int mcu_powersave_console_command( int argc, char *argv[] );
int wiced_init_console_command( int argc, char *argv[] );
int wiced_log_set_level_console_command( int argc, char *argv[] );
int wiced_log_get_level_console_command( int argc, char *argv[] );

int hibernation_console_command( int argc, char *argv[] );
int mcu_powersave_clock_console_command( int argc, char *argv[] );
int mcu_powersave_tick_console_command( int argc, char *argv[] );
int mcu_powersave_mode_console_command( int argc, char *argv[] );
int mcu_powersave_freq_console_command( int argc, char *argv[] );
int mcu_powersave_sleep_console_command( int argc, char *argv[] );
int mcu_powersave_info_console_command( int argc, char *argv[] );
int mcu_powersave_gpio_wakeup_enable_console_command( int argc, char *argv[] );
int mcu_powersave_gpio_wakeup_disable_console_command( int argc, char *argv[] );
int mcu_powersave_gpio_wakeup_ack_console_command( int argc, char *argv[] );
int mcu_powersave_gci_gpio_wakeup_enable_console_command( int argc, char *argv[] );
int mcu_powersave_gci_gpio_wakeup_disable_console_command( int argc, char *argv[] );
int mcu_powersave_gci_gpio_wakeup_ack_console_command( int argc, char *argv[] );
int mcu_wlan_powersave_stats_console_command( int argc, char *argv[] );

int fram_read( int argc, char* argv[] );
int fram_write( int argc, char* argv[] );
#ifdef __cplusplus
} /*extern "C" */
#endif
